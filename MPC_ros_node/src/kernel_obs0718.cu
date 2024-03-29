#include <iostream>
#include <ctime>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <stdlib.h>
#include <stdio.h>
#include <fstream>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <curand_kernel.h>
#include <curand.h>
#include <chrono>

using namespace std;

#define SIZE		256
#define SHMEM_SIZE	512 
#define DSIZE		102400
#define USIZE		40

#define CONTROLSIZE   2				// Number of controls u = [vd,wd]
#define STATESIZE     6				// Number of states   x = [x,y,phi,v0,theta,theta_dot,delta]
#define T_HRZ         3.0f			// Prediction time horizon 3.2 (REMEMBER TO ALSO CHANGE MAIN.CPP)
#define F_STP		      20.0f			// Frequency of prediction (control freq) 20
#define T_STP		      1/F_STP			// Period of prediction
#define N_HRZ         T_HRZ*F_STP		// N steps in a time horizon
#define K             1024            // K-rollout predictions
#define V_MAX         2.5f			// Velocity command upper-bound
#define V_MIN         1.8f			// Velocity command lower-bound
#define W_MAX		      1.0f			// Angular acceleration command upper-bound
#define W_MIN		      -1.0f			// Angular acceleration command lower-bound
#define ROAD_WIDTH    6.0f			// Width of road (unit: m) 3.5 works
// #define LAMBDA        350.0f //350
#define V_RAND_SCALAR 0.1f
#define W_RAND_SCALAR 0.6f //0.5 // 0.6
#define W_DIFF        5.0f //0.04f  //0.4
#define SCALE_MODEL   1.0f //0.61f  //0.72f
#define RHO           0.08f//0.08f
#define DELTA_MAX     0.6f
#define DELTA_MIN     -0.6f

// #define K1            30.824f
// #define K2            11.3638f
// #define K3            0.000637f
// #define K1            32.4522f
// #define K2            7.0690f
// #define K3            0.0207f
// #define K1            32.8939f
// #define K2            6.5065f
// #define K3            -0.0010786f
// #define K1            29.6212f
// #define K2            9.1605f
// #define K3            -0.1089f

#define K1            38.9562f
#define K2            11.3425f
#define K3            -0.000976f

// #define K1            42.3960f
// #define K2            9.8507f
// #define K3            -1.5682f

#define FG_RAD        8.0f
#define GH_RAD		    8.0f
#define HI_RAD        8.0f
#define IF_RAD        8.0f

// good: 200 800 5 | 1 2 5 (W:1.1)
// 50 400 300 l350
// #define OFF_ROAD_COST	  280.0f		//-100- // 300 Penalty for leaving the road
// #define COLLISION_COST  100.0f		//-500- //200 80.0f Penalty for colliding into an obstacle 100  140
// #define TRACK_ERR_COST	13.0f		//-100- //15.0f// Penalty for tracking error (parting from the middle) 3.0 3.0 20
// #define OBS_SIZE        1.0f
// #define LAMBDA        500.0f //350
// #define OFF_ROAD_COST	  20.0f		//-100- // 300 Penalty for leaving the road
// #define COLLISION_COST  300.0f		//-500- //200 80.0f Penalty for colliding into an obstacle 100  140
// #define TRACK_ERR_COST	100.0f		//-100- //15.0f// Penalty for tracking error (parting from the middle) 3.0 3.0 20
// #define OBS_SIZE        1.0f
#define LAMBDA        300.0f //350
#define OFF_ROAD_COST	  200.0f		//-100- // 300 Penalty for leaving the road
#define COLLISION_COST  300.0f		//-500- //200 80.0f Penalty for colliding into an obstacle 100  140
#define TRACK_ERR_COST	100.0f		//-100- //15.0f// Penalty for tracking error (parting from the middle) 3.0 3.0 20
#define OBS_SIZE        1.0f


struct Control {
  float vd, wd;
};
struct State {
  float x, y, phi, v0, theta, theta_dot, delta;
};
struct Track {
  // Track fcn in the form of by + ax + c =0
  float b, a, c;
};
struct Pos {
  float x, y;
};

struct Obs {
  float x, y, r;
};

__global__ void sum_reduction(float* v, float* v_r) {
  /*  Perform sum reduction to an array of floats (here since the for
    loop iterates with a division of 2, the blockDim (#of threads) 
    must be 2 to the nth power) !!Important!! */

  __shared__ float partial_sum[64];

  int i = blockIdx.x * (blockDim.x * 2) + threadIdx.x;

  partial_sum[threadIdx.x] = v[i] + v[i + blockDim.x];
  __syncthreads();

  for (int s = blockDim.x / 2; s > 0; s >>= 1) {
    if (threadIdx.x < s) {
      partial_sum[threadIdx.x] += partial_sum[threadIdx.x + s];
    }
    __syncthreads();
  }

  if (threadIdx.x == 0) {
    v_r[blockIdx.x] = partial_sum[0];
    //printf("result2: %.3f\n", partial_sum[0]);
    //printf("%4d::%.3f\n", blockIdx.x, v_r[blockIdx.x]);
  }
}

__host__ __device__ State bicycleModelold(State state, Control u) {
/*  Linear bicycle model, which geneates state_dot. Then by integrating the state_dot, 
  we can then get the nest state x_k+1*/
  static float wd_old = 0;
  if(u.wd > (wd_old + W_DIFF)){u.wd = wd_old + W_DIFF;}
  if(u.wd < (wd_old - W_DIFF)){u.wd = wd_old - W_DIFF;}
  wd_old = u.wd;

  // u.wd = 0.7*u.wd;
  // delta_d = atan(L_B*u.wd/u.vd)/sin(EPSILON);

  State state_dot;
  State state_next;
  float Ka = 2.54;
  state_dot.x = state.v0 * cosf(state.phi);
  state_dot.y = state.v0 * sinf(state.phi);
  state_dot.phi = u.wd;
  state_dot.v0 = Ka * state.v0 * (u.vd - state.v0);
  state_next.x = state.x + state_dot.x * T_STP;
  state_next.y = state.y + state_dot.y * T_STP;
  state_next.phi = state.phi + state_dot.phi * T_STP;
  state_next.v0 = state.v0 + state_dot.v0 * T_STP;
  return state_next;
}

__host__ __device__ State bicycleModel(State state, Control u) {
/*  Linear bicycle model, which geneates state_dot. Then by integrating the state_dot, 
  we can then get the nest state x_k+1*/
  
  // 1. Compute delta with discrete linear state space model
  //    1.1 Perform Full state feedback to get command u
  //    1.2 Obtain theta[k+1], theta_dot[k+1], delta[k+1]
  //
  // 2. Compute the current phi_dot with delta[k+1] as the actual omega

  // Setup parameters
  float pi      = 3.14159265359;
  float epsilon = 70*pi/180;
  float l_a     = 0.395;
  float l_b     = 1.053;
  float g       = 9.81;
  float Ka      = 2.54;
  float h       = 0.6;

  // float A[9] = {1.0232, 0.0504, 0, 
  //               0.9326, 1.0232, 0,
  //               0, 0, exp(-10*state.v0/79)};
  // float A[9] = {1.3713, 0.2507, 0, 
  //             3.5127, 1.0176, 0,
  //             0, 0, exp(-10*state.v0/79)};

  // float B[3] = {0.0017, 0.0670, -79*(exp(-10*state.v0/79)-1)/200/state.v0};
  // float B[3] = {0.0283*state.v0, 0.098*state.v0, -79*(exp(-10*state.v0/79)-1)/200/state.v0};

  float A[9] = {1,        T_STP,    0,
                g/h*T_STP,    1,    0,
                0,            0,  1-state.v0/l_a*T_STP};

  float B[3] = {0, l_a*state.v0*sin(epsilon)*T_STP/l_b/h, T_STP};

  // Full state feedback to compute u_bar
  float delta_d   = 1/std::sin(epsilon)*std::atan(l_b*u.wd/u.vd);
  // float K_gain[3] = {25.1229, 5.2253, 0.016359};
  // float K_gain[3] = {32.4522, 7.0690, 0.0207};
  float K_gain[3] = {K1, K2, K3};
  float x[3]      = {state.theta, state.theta_dot, state.delta};
  float x_d[3]    = {(-u.vd*u.vd*std::sin(epsilon))/(g*l_b)*delta_d, 0, delta_d};
  float u_d       = (u.vd/l_a)*delta_d*SCALE_MODEL;
  float u_bar     = 0;
  float u_fb      = 0;

  for(int i=0; i<3; i++){
    x[i] = x[i] - x_d[i];  
  }
  for(int i=0; i<3; i++){
    u_fb -= K_gain[i]*x[i]; 
  }
  u_bar = u_fb + u_d;
  // u_bar = u_fb;

  // Obtain state[k+1]
  State state_next      = {0};
  State state_dot       = {0};
  state_dot.x           = state.v0 * cosf(state.phi);
  state_dot.y           = state.v0 * sinf(state.phi);
  state_dot.v0          = Ka * state.v0 * (u.vd - state.v0);
  state_next.theta      = A[0]*state.theta + A[1]*state.theta_dot + A[2]*state.delta + B[0]*u_bar;
  state_next.theta_dot  = A[3]*state.theta + A[4]*state.theta_dot + A[5]*state.delta + B[1]*u_bar;
  state_next.delta      = A[6]*state.theta + A[7]*state.theta_dot + A[8]*state.delta + B[2]*u_bar;
  if(state_next.delta > DELTA_MAX){state_next.delta = DELTA_MAX;}
  if(state_next.delta < DELTA_MIN){state_next.delta = DELTA_MIN;}
  state_next.x          = state.x + state_dot.x * T_STP;
  state_next.y          = state.y + state_dot.y * T_STP;
  state_next.v0         = state.v0 + state_dot.v0 * T_STP;
  state_dot.phi         = std::tan(state_next.delta*std::sin(epsilon))/l_b*state_next.v0;
  // if(state_dot.phi > W_MAX){state_dot.phi = W_MAX;}
  // if(state_dot.phi < W_MIN){state_dot.phi = W_MIN;}
  state_next.phi        = state.phi + state_dot.phi * T_STP;
  
  return state_next;
}

__host__ __device__ State bicycleModelConti(State state, Control u) {
/*  Linear bicycle model, which geneates state_dot. Then by integrating the state_dot, 
  we can then get the nest state x_k+1*/
  
  // 1. Compute delta with discrete linear state space model
  //    1.1 Perform Full state feedback to get command u
  //    1.2 Obtain theta[k+1], theta_dot[k+1], delta[k+1]
  //
  // 2. Compute the current phi_dot with delta[k+1] as the actual omega

  // Setup parameters
  float pi      = 3.14159265359;
  float epsilon = 70*pi/180;
  float l_a     = 0.395;
  float l_b     = 1.053;
  float g       = 9.81;
  float Ka      = 2.54;
  float h       = 0.45;

  float A[9] = {0,   1,  0,
                g/h, 0,  0,
                0,   0, -state.v0/l_a};

  float B[3] = {0, l_a*state.v0*std::sin(epsilon)/l_b/h, 1};

  // Full state feedback to compute u_bar
  float delta_d   = 1/std::sin(epsilon)*std::atan(l_b*u.wd/u.vd);
  // float K_gain[3] = {-32.4522, -7.0690, -0.00207};
  float K_gain[3] = {K1, K2, K3};
  float x[3]      = {state.theta, state.theta_dot, state.delta};
  float x_d[3]    = {(-u.vd*u.vd*std::sin(epsilon))/(g*l_b)*delta_d, 0, delta_d};
  float u_d       = (u.vd/l_a)*delta_d;
  float u_bar     = 0;
  float u_fb      = 0;

  for(int i=0; i<3; i++){
    x[i] = x[i] - x_d[i];  
  }
  for(int i=0; i<3; i++){
    u_fb -= K_gain[i]*x[i]; 
  }
  u_bar = u_fb + u_d;

  // Obtain state[k+1]
  State state_next      = {0};
  State state_dot       = {0};
  state_dot.x           = state.v0 * cosf(state.phi);
  state_dot.y           = state.v0 * sinf(state.phi);
  state_dot.v0          = Ka * state.v0 * (u.vd - state.v0);
  state_dot.theta       = A[0]*state.theta + A[1]*state.theta_dot + A[2]*state.delta + B[0]*u_bar;
  state_dot.theta_dot   = A[3]*state.theta + A[4]*state.theta_dot + A[5]*state.delta + B[1]*u_bar;
  state_dot.delta       = A[6]*state.theta + A[7]*state.theta_dot + A[8]*state.delta + B[2]*u_bar;
  state_next.x          = state.x + state_dot.x * T_STP;
  state_next.y          = state.y + state_dot.y * T_STP;
  state_next.v0         = state.v0 + state_dot.v0 * T_STP;
  state_next.theta      = state.theta + state_dot.theta * T_STP;
  state_next.theta_dot  = state.theta_dot + state_dot.theta_dot * T_STP;
  state_next.delta      = state.delta + state_dot.delta * T_STP;
  state_dot.phi         = std::tan(state_next.delta*std::sin(epsilon))/l_b*state_next.v0;
  if(state_dot.phi > W_MAX){state_dot.phi = W_MAX;}
  if(state_dot.phi < W_MIN){state_dot.phi = W_MIN;}
  state_next.phi        = state.phi + state_dot.phi * T_STP;
  
  return state_next;
}

__device__ __host__ void clampingFcn(Control* u_in, float scale) {
  /*	Clamping fcn for the perturbated command, acting as a
    hard contrain*/
  if (u_in->vd > V_MAX) { u_in->vd = V_MAX; }
  else if (u_in->vd < V_MIN) { u_in->vd = V_MIN; }
  if (u_in->wd > W_MAX*scale) { u_in->wd = W_MAX*scale; }
  else if (u_in->wd < W_MIN*scale) { u_in->wd = W_MIN*scale; }
}

__device__ float distanceFromTrack_(float inner_f, float inner_g, float inner_h,
  float inner_i, Track* inner_fcn) {
  /*  Calculates the distance from the middle of a designated path 
    (Here, from the Tetragon imposing the road infront of the schools library)*/

  float slope[] = { -inner_fcn[0].a, -inner_fcn[1].a, -inner_fcn[2].a, -inner_fcn[3].a };
  float distance = 0.0f;
  if (inner_f < 0 && inner_g < 0 && inner_h < 0 && inner_i > 0) {
    distance = fabs(fabs(inner_f) - 0.5 * ROAD_WIDTH / cosf(atanf(slope[0])));
  }
  else if (inner_f < 0 && inner_g > 0 && inner_h < 0 && inner_i > 0) {
    distance = fabs(sqrtf(pow(inner_f, 2) + pow(inner_g, 2)) - 0.5 * ROAD_WIDTH);
  }
  else if (inner_f > 0 && inner_g > 0 && inner_h < 0 && inner_i > 0) {
    distance = fabs(fabs(inner_g) - 0.5 * ROAD_WIDTH / cosf(atanf(slope[1])));
  }
  else if (inner_f > 0 && inner_g > 0 && inner_h > 0 && inner_i > 0) {
    distance = fabs(sqrtf(pow(inner_g, 2) + pow(inner_h, 2)) - 0.5 * ROAD_WIDTH);
  }
  else if (inner_f > 0 && inner_g < 0 && inner_h>0 && inner_i > 0) {
    distance = fabs(fabs(inner_h) - 0.5 * ROAD_WIDTH / cosf(atanf(slope[2])));
  }
  else if (inner_f > 0 && inner_g < 0 && inner_h>0 && inner_i < 0) {
    distance = fabs(sqrtf(pow(inner_h, 2) + pow(inner_i, 2)) - 0.5 * ROAD_WIDTH);
  }
  else if (inner_f > 0 && inner_g < 0 && inner_h < 0 && inner_i < 0) {
    distance = fabs(fabs(inner_i) - 0.5 * ROAD_WIDTH / cosf(atanf(slope[3])));
  }
  else if (inner_f > 0 && inner_g < 0 && inner_h < 0 && inner_i < 0) {
    distance = fabs(sqrtf(pow(inner_i, 2) + pow(inner_f, 2)) - 0.5 * ROAD_WIDTH);
  }
  else if (inner_f == 0 || inner_g == 0 || inner_h == 0 || inner_i == 0) {
    float inner_min = 0.0f;
    (fabs(inner_f) > fabs(inner_g)) ? inner_min = fabs(inner_g) : inner_min = fabs(inner_f);
    (inner_min > fabs(inner_h)) ? inner_min = fabs(inner_h) : inner_min = inner_min;
    (inner_min > fabs(inner_i)) ? inner_min = fabs(inner_i) : inner_min = inner_min;
    distance = inner_min - 0.5 * ROAD_WIDTH;
  }
  if (distance > ROAD_WIDTH / 2) {
    distance = ROAD_WIDTH; // Important to set it to 2 instead of 0 to ensure the vehicle tracks the designated path
  }

  return distance;
}

__device__ float pointToLineDist(float inner, Track line) {
	float a = line.a;
	float b = line.b;
	return fabs(inner) / sqrtf(a * a + b * b);
}

__device__ float distanceFromTrack(float* inner, float* nominal, Track* inner_fcn, Pos* turn_circle_intersec, State* state) {
	/*  Calculates the distance from the middle of a designated path
		(Here, from the Tetragon imposing the road infront of the schools library)*/

	float slope[] = {-inner_fcn[0].a, -inner_fcn[1].a, -inner_fcn[2].a, -inner_fcn[3].a};
  float distance = 0;
  float OFF_ROAD_RENALTY = 0.0;

	// Distance from the curves 
	if (nominal[0] >= 0 && nominal[1] <= 0) {  // curve fg
    // printf("%.3f\n", fabs(sqrtf((state->x - turn_circle_intersec[0].x) * (state->x - turn_circle_intersec[0].x)
		// 	+ (state->y - turn_circle_intersec[0].y) * (state->y - turn_circle_intersec[0].y)) - FG_RAD));
		return fabs(sqrtf((state->x - turn_circle_intersec[0].x) * (state->x - turn_circle_intersec[0].x)
			+ (state->y - turn_circle_intersec[0].y) * (state->y - turn_circle_intersec[0].y)) - FG_RAD);
    // if(distance > ROAD_WIDTH/2){distance += }
  }
	if (nominal[2] >= 0 && nominal[3] >= 0) {  // curve gh
    // printf("%.3f\n", fabs(sqrtf((state->x - turn_circle_intersec[1].x) * (state->x - turn_circle_intersec[1].x)
		// 	+ (state->y - turn_circle_intersec[1].y) * (state->y - turn_circle_intersec[1].y)) - GH_RAD));
		return fabs(sqrtf((state->x - turn_circle_intersec[1].x) * (state->x - turn_circle_intersec[1].x)
			+ (state->y - turn_circle_intersec[1].y) * (state->y - turn_circle_intersec[1].y)) - GH_RAD);
	}
	if (nominal[4] <= 0 && nominal[5] >= 0) {  // curve hi
    // printf("%.3f\n", fabs(sqrtf((state->x - turn_circle_intersec[2].x) * (state->x - turn_circle_intersec[2].x)
		// 	+ (state->y - turn_circle_intersec[2].y) * (state->y - turn_circle_intersec[2].y)) - HI_RAD));
		return fabs(sqrtf((state->x - turn_circle_intersec[2].x) * (state->x - turn_circle_intersec[2].x)
			+ (state->y - turn_circle_intersec[2].y) * (state->y - turn_circle_intersec[2].y)) - HI_RAD);
	}
	if (nominal[6] <= 0 && nominal[7] <= 0) {  // curve if
    // printf("%.3f\n", fabs(sqrtf((state->x - turn_circle_intersec[3].x) * (state->x - turn_circle_intersec[3].x)
		// 	+ (state->y - turn_circle_intersec[3].y) * (state->y - turn_circle_intersec[3].y)) - IF_RAD));
		return fabs(sqrtf((state->x - turn_circle_intersec[3].x) * (state->x - turn_circle_intersec[3].x)
			+ (state->y - turn_circle_intersec[3].y) * (state->y - turn_circle_intersec[3].y)) - IF_RAD);
	}

	// Distance from the tracks (outside of the inner functions)
	if(inner[0] < 0 && nominal[0] <= 0 && nominal[7] >= 0){ 
		return fabs(pointToLineDist(inner[0], inner_fcn[0]) - 0.5 * ROAD_WIDTH);  // respect to line: f
	}
	if (inner[1] > 0 && nominal[1] >= 0 && nominal[2] <= 0) {
		return fabs(pointToLineDist(inner[1], inner_fcn[1]) - 0.5 * ROAD_WIDTH);  // respect to line: g
	}
	if (inner[2] > 0 && nominal[3] <= 0 && nominal[4] >= 0) {
		return fabs(pointToLineDist(inner[2], inner_fcn[2]) - 0.5 * ROAD_WIDTH);  // respect to line: h
	}
	if (inner[3] < 0 && nominal[5] <= 0 && nominal[6] >= 0) {
		return fabs(pointToLineDist(inner[3], inner_fcn[3]) - 0.5 * ROAD_WIDTH);  // respect to line: i
	}

	 //Distance from the tracks (inside of the inner functions)
	if (inner[0] > 0 && inner[1] < 0 && inner[2] < 0 && inner[3] > 0) {
		float dist_f = fabs(pointToLineDist(inner[0], inner_fcn[0]) + 0.5 * ROAD_WIDTH);
		float dist_g = fabs(pointToLineDist(inner[1], inner_fcn[1]) + 0.5 * ROAD_WIDTH);
		float dist_h = fabs(pointToLineDist(inner[2], inner_fcn[2]) + 0.5 * ROAD_WIDTH);
		float dist_i = fabs(pointToLineDist(inner[3], inner_fcn[3]) + 0.5 * ROAD_WIDTH);
		float min1, min2;
		(dist_f > dist_g) ? min1 = dist_g : min1 = dist_f;
		(dist_h > dist_i) ? min2 = dist_i : min2 = dist_h;
		if (min1 > min2){ 
			return min2; 
		}
		else{ 
			return min1; 
		}
	}

	// If point lies on inner function(s)
	if (inner[0] == 0 || inner[1] == 0 || inner[2] == 0 || inner[3] == 0) {
		float inner_min = 0.0f;
		(fabs(inner[0]) > fabs(inner[1])) ? inner_min = fabs(inner[1]) : inner_min = fabs(inner[0]);
		(inner_min > fabs(inner[2])) ? inner_min = fabs(inner[2]) : inner_min = inner_min;
		(inner_min > fabs(inner[3])) ? inner_min = fabs(inner[3]) : inner_min = inner_min;
		return fabs(inner_min + 0.5 * ROAD_WIDTH);
	}
	return 0.0;
}

__device__ float obstacleCollision(State* state, Obs* obstacle) {
  /*  Calculate the relative distace from the middle of the obstacle 
    (if output == 0, no collision)*/

  float output = 0.0, obs_fcn = 0.0;
  obs_fcn = powf(state->x - obstacle->x, 2) + powf(state->y - obstacle->y, 2) - powf(obstacle->r, 2);
  if (obs_fcn < 0) {
    output = -obs_fcn / obstacle->r;
  }
  return output;
}

// __device__ float calculateCost(State* state, Track* outer_fcn, Track* inner_fcn) {
//   /*  Calculate the cost of a current state (obstacle collision part added)*/

//   float state_cost = 0.0f;
//   float outer_f = outer_fcn[0].b * state->y + outer_fcn[0].a * state->x + outer_fcn[0].c;
//   float outer_g = outer_fcn[1].b * state->y + outer_fcn[1].a * state->x + outer_fcn[1].c;
//   float outer_h = outer_fcn[2].b * state->y + outer_fcn[2].a * state->x + outer_fcn[2].c;
//   float outer_i = outer_fcn[3].b * state->y + outer_fcn[3].a * state->x + outer_fcn[3].c;

//   float inner_f = inner_fcn[0].b * state->y + inner_fcn[0].a * state->x + inner_fcn[0].c;
//   float inner_g = inner_fcn[1].b * state->y + inner_fcn[1].a * state->x + inner_fcn[1].c;
//   float inner_h = inner_fcn[2].b * state->y + inner_fcn[2].a * state->x + inner_fcn[2].c;
//   float inner_i = inner_fcn[3].b * state->y + inner_fcn[3].a * state->x + inner_fcn[3].c;

//   float distance = distanceFromTrack(inner_f, inner_g, inner_h, inner_i, inner_fcn);
//   if ((outer_f > 0 && outer_g < 0 && outer_h < 0 && outer_i>0) &&
//     !(inner_f > 0 && inner_g < 0 && inner_h < 0 && inner_i>0)) {
//     state_cost += 0;
//   }
//   else {
//     state_cost += OFF_ROAD_COST;
//   }
//   state_cost += distance / (ROAD_WIDTH / 2) * TRACK_ERR_COST;
//   return state_cost;
// }

__device__ float calculateCost(State* state, Track* outer_fcn, Track* inner_fcn, 
  Track* nominal_fcn, Obs* obstacle, Pos* turn_circle_intersec) {
	/*  Calculate all the state costs in the prediction horizon
		for each rollout k */
	/* Calculate the cost of a current state (obstacle collision part added)
	* index of the path names
	*          g
	*        ______
	*       /     /
	*    h /     / f
	*     /     /
	*     ------
	*       i
	* f, g intersection : [x1, y1]
	* g, h intersection : [x2, y2]
	* h, i intersection : [x3, y3]
	* i, f intersection : [x4, y4]*/
  float state_cost = 0;
	
	float outer[4] = {
		outer_fcn[0].b * state->y + outer_fcn[0].a * state->x + outer_fcn[0].c,
		outer_fcn[1].b * state->y + outer_fcn[1].a * state->x + outer_fcn[1].c,
		outer_fcn[2].b * state->y + outer_fcn[2].a * state->x + outer_fcn[2].c,
		outer_fcn[3].b * state->y + outer_fcn[3].a * state->x + outer_fcn[3].c
	};

	float inner[4] = {
		inner_fcn[0].b * state->y + inner_fcn[0].a * state->x + inner_fcn[0].c,
		inner_fcn[1].b * state->y + inner_fcn[1].a * state->x + inner_fcn[1].c,
		inner_fcn[2].b * state->y + inner_fcn[2].a * state->x + inner_fcn[2].c,
		inner_fcn[3].b * state->y + inner_fcn[3].a * state->x + inner_fcn[3].c
	};

	float nominal[8] = {
		nominal_fcn[0].b * state->y + nominal_fcn[0].a * state->x + nominal_fcn[0].c,
		nominal_fcn[1].b * state->y + nominal_fcn[1].a * state->x + nominal_fcn[1].c,
		nominal_fcn[2].b * state->y + nominal_fcn[2].a * state->x + nominal_fcn[2].c,
		nominal_fcn[3].b * state->y + nominal_fcn[3].a * state->x + nominal_fcn[3].c,
		nominal_fcn[4].b * state->y + nominal_fcn[4].a * state->x + nominal_fcn[4].c,
		nominal_fcn[5].b * state->y + nominal_fcn[5].a * state->x + nominal_fcn[5].c,
		nominal_fcn[6].b * state->y + nominal_fcn[6].a * state->x + nominal_fcn[6].c,
		nominal_fcn[7].b * state->y + nominal_fcn[7].a * state->x + nominal_fcn[7].c
	};
	float distance = distanceFromTrack(inner, nominal, inner_fcn, turn_circle_intersec, state);
  // float distance = distanceFromTrack(inner[0], inner[1], inner[2], inner[3], inner_fcn);
	if ((nominal[0] >= 0 && nominal[1] <= 0) || (nominal[2] >= 0 && nominal[3] >= 0) || (nominal[4] <= 0 && nominal[5] >= 0) || (nominal[6] <= 0 && nominal[7] <= 0)) {
		if (distance > ROAD_WIDTH / 2) {
			state_cost += OFF_ROAD_COST;
		}
	}
	else {
    if ((outer[0] > 0 && outer[1] < 0 && outer[2] < 0 && outer[3] > 0) &&
      !(inner[0] > 0 && inner[1] < 0 && inner[2] < 0 && inner[3] > 0)) {
      state_cost += 0;
    }
    else {
      state_cost += OFF_ROAD_COST;
    }
	}
	state_cost += distance / (ROAD_WIDTH / 2) * TRACK_ERR_COST;
  // state_cost += distance*distance/(ROAD_WIDTH*ROAD_WIDTH/4) * TRACK_ERR_COST;
	
	// Obstacle avoidance penalty

	float collision = 0.0;
	for (int i = 0; obstacle[i].x != NULL; i++) {
		collision = fabs(obstacleCollision(state, &obstacle[i]));
		state_cost += collision * COLLISION_COST;
    }
	return state_cost;
}

__global__ void initCurand(curandState* state, uint64_t seed) {
  /*	Each thread gets same seed, a different sequence
    number, no offset */

  int idx = threadIdx.x + blockIdx.x * blockDim.x;
  curand_init(seed, idx, 0, &state[idx]);
}

__global__ void normalRand(curandState* state, float* rand, float scalar) {
  /*  Generate the random number with mean 0.0 and standard
    deviation 1.0, scalar: stdev scalar */

  int idx = threadIdx.x + blockIdx.x * blockDim.x;
	curandState localState = state[idx];
	rand[idx] = curand_normal(&localState) * scalar;
}

__device__ float getRandom(uint64_t seed, int tid, int shift) {
	curandState s;
	curand_init(seed + tid + shift, 0, 0, &s);
	return curand_normal(&s);
}

__global__ void genControlOld(curandState* state, Control* rand, float* u, float scalar) {
  int idx = threadIdx.x + blockIdx.x * blockDim.x;
  // Generate the random number with mean 0.0 and standard deviation 1.0, scalar: stdev scalar
  rand[idx].vd = curand_normal(&state[idx]) * scalar + u[0];
  rand[idx].wd = curand_normal(&state[idx]) * scalar + u[1];
  clampingFcn(&rand[idx],1);
}

__global__ void genControl(curandState* state, Control* rand, Control* u, float scalar) {
  /*  Generate V from nominal control U and the generated pertubations E (V = U + E)
    mean 0.0 and standard deviation 1.0, scalar: stdev scalar*/

  int idx = threadIdx.x + blockIdx.x * blockDim.x;
	curandState localState = state[idx];
	rand[idx].vd = curand_normal(&localState) * scalar + u[threadIdx.x].vd;
	rand[idx].wd = curand_normal(&localState) * scalar + u[threadIdx.x].wd;
  clampingFcn(&rand[idx],1);
}

__global__ void genControlNew(uint64_t seed, Control* rand, Control* u) {
	/*  Generate V from nominal control U and the generated pertubations E (V = U + E)
		mean 0.0 and standard deviation 1.0, scalar: stdev scalar*/

	int idx = threadIdx.x + blockIdx.x * blockDim.x;
  float rand1 = getRandom(seed, idx, 0);
  float rand2 = getRandom(seed, idx, 10);
  if(rand1 < 0.02 && rand1 > -0.02){
    rand1 = 0;
  }
  if(rand2 < 0.02 && rand2 > -0.02){
    rand2 = 0;
  }
	rand[idx].vd = rand1 * V_RAND_SCALAR + u[threadIdx.x].vd;
	rand[idx].wd = rand2 * W_RAND_SCALAR + u[threadIdx.x].wd;
  // if(rand2==0){
  //   printf("rand: %.3f", rand2);
  // }
  // rand[idx].vd = 2;
  // rand[idx].wd = 1.0;
	clampingFcn(&rand[idx],1);
}

__global__ void genState(State* state_list, State* init_state, Control* pert_control) {
  /*  Generate all the states in the prediction horizon for
    each rollout k
    - init_state is k*1 elements long (k-rollouts)
    - need to build k*n list for each prediction state
    - genState <<< K / 256, K / 4 >>> idx is 0-1024
  */
  int idx = threadIdx.x + blockIdx.x * blockDim.x;
  State state_temp = *init_state;
  for (int n = 0; n < N_HRZ; n++) {
    state_temp = bicycleModel(state_temp, pert_control[n + int(N_HRZ) * idx]); //////???????????????????
    int listidx = n + (N_HRZ) * idx;
    state_list[listidx] = state_temp;
  }
  
}  

// __global__ void costFcn(float* cost_list, State* state_list,  
//   Track* outer_fcn, Track* inner_fcn, Obs* obstacle) {
//   /*  Calculate all the state costs in the prediction horizon 
//     for each rollout k */
//   int idx = threadIdx.x + blockIdx.x * blockDim.x;
//   float state_cost = 0.0f;
//   State state = state_list[idx];

//   // Tracking penatlty
//   float outer_f = outer_fcn[0].b * state.y + outer_fcn[0].a * state.x + outer_fcn[0].c;  
//   float outer_g = outer_fcn[1].b * state.y + outer_fcn[1].a * state.x + outer_fcn[1].c;
//   float outer_h = outer_fcn[2].b * state.y + outer_fcn[2].a * state.x + outer_fcn[2].c;
//   float outer_i = outer_fcn[3].b * state.y + outer_fcn[3].a * state.x + outer_fcn[3].c;

//   float inner_f = inner_fcn[0].b * state.y + inner_fcn[0].a * state.x + inner_fcn[0].c;
//   float inner_g = inner_fcn[1].b * state.y + inner_fcn[1].a * state.x + inner_fcn[1].c;
//   float inner_h = inner_fcn[2].b * state.y + inner_fcn[2].a * state.x + inner_fcn[2].c;
//   float inner_i = inner_fcn[3].b * state.y + inner_fcn[3].a * state.x + inner_fcn[3].c;
//   float distance = distanceFromTrack(inner_f, inner_g, inner_h, inner_i, inner_fcn);

//   if ((outer_f > 0 && outer_g < 0 && outer_h < 0 && outer_i>0) && 
//     !(inner_f > 0 && inner_g < 0 && inner_h < 0 && inner_i > 0)) {
//     state_cost += 0;
//   }
//   else {
//     state_cost += OFF_ROAD_COST;
//   }
//   // state_cost += distance / (ROAD_WIDTH / 2) * TRACK_ERR_COST;
//   state_cost += (distance / (ROAD_WIDTH / 2)) * TRACK_ERR_COST;

//   // Obstacle avoidance penalty

//   float collision = 0.0;
//   for (int i = 0; obstacle[i].x != NULL; i++) {
//     collision = fabs(obstacleCollision(&state, &obstacle[i]));
//     state_cost += collision * COLLISION_COST;
//   }
//   cost_list[idx] = state_cost;
// }

__global__ void costFcn(float* cost_list, State* state_list,
	Track* outer_fcn, Track* inner_fcn, Track* nominal_fcn, Obs* obstacle, Pos* turn_circle_intersec) {
	/*  Calculate all the state costs in the prediction horizon
		for each rollout k */
	/* Calculate the cost of a current state (obstacle collision part added)
	* index of the path names
	*          g
	*        ______
	*       /     /
	*    h /     / f
	*     /     /
	*     ------
	*       i
	* f, g intersection : [x1, y1]
	* g, h intersection : [x2, y2]
	* h, i intersection : [x3, y3]
	* i, f intersection : [x4, y4]*/

	int idx = threadIdx.x + blockIdx.x * blockDim.x;
  /* <<< M blocks , N threads >>>: M-> Grid size, N-> Block size */
  // int horizon_idx = blockIdx.x;
  // float prediction_decay = 1 - 0*horizon_idx/gridDim.x; 

	float state_cost = 0.0f;
	State state = state_list[idx];
	
	float outer[4] = {
		outer_fcn[0].b * state.y + outer_fcn[0].a * state.x + outer_fcn[0].c,
		outer_fcn[1].b * state.y + outer_fcn[1].a * state.x + outer_fcn[1].c,
		outer_fcn[2].b * state.y + outer_fcn[2].a * state.x + outer_fcn[2].c,
		outer_fcn[3].b * state.y + outer_fcn[3].a * state.x + outer_fcn[3].c
	};

	float inner[4] = {
		inner_fcn[0].b * state.y + inner_fcn[0].a * state.x + inner_fcn[0].c,
		inner_fcn[1].b * state.y + inner_fcn[1].a * state.x + inner_fcn[1].c,
		inner_fcn[2].b * state.y + inner_fcn[2].a * state.x + inner_fcn[2].c,
		inner_fcn[3].b * state.y + inner_fcn[3].a * state.x + inner_fcn[3].c
	};

	float nominal[8] = {
		nominal_fcn[0].b * state.y + nominal_fcn[0].a * state.x + nominal_fcn[0].c,
		nominal_fcn[1].b * state.y + nominal_fcn[1].a * state.x + nominal_fcn[1].c,
		nominal_fcn[2].b * state.y + nominal_fcn[2].a * state.x + nominal_fcn[2].c,
		nominal_fcn[3].b * state.y + nominal_fcn[3].a * state.x + nominal_fcn[3].c,
		nominal_fcn[4].b * state.y + nominal_fcn[4].a * state.x + nominal_fcn[4].c,
		nominal_fcn[5].b * state.y + nominal_fcn[5].a * state.x + nominal_fcn[5].c,
		nominal_fcn[6].b * state.y + nominal_fcn[6].a * state.x + nominal_fcn[6].c,
		nominal_fcn[7].b * state.y + nominal_fcn[7].a * state.x + nominal_fcn[7].c
	};
	float distance = distanceFromTrack(inner, nominal, inner_fcn, turn_circle_intersec, &state);
  // printf("%.3f\n",distance);
  // float distance = distanceFromTrack_(inner[0], inner[1], inner[2], inner[3], inner_fcn);

	// if ((nominal[0] >= 0 && nominal[1] <= 0) || (nominal[2] >= 0 && nominal[3] >= 0) || (nominal[4] <= 0 && nominal[5] >= 0) || (nominal[6] <= 0 && nominal[7] <= 0)) {
	// 	if (distance > ROAD_WIDTH / 2) {
	// 		state_cost += OFF_ROAD_COST;
	// 	}
	// }
	// else {
  //   if ((outer[0] > 0 && outer[1] < 0 && outer[2] < 0 && outer[3] > 0) &&
  //     !(inner[0] > 0 && inner[1] < 0 && inner[2] < 0 && inner[3] > 0)) {
  //     state_cost += 0;
  //   }
  //   else {
  //     state_cost += OFF_ROAD_COST;
  //   }
	// }

  if(distance >= ROAD_WIDTH/2){
    state_cost += OFF_ROAD_COST;
    // distance += 0.5;
  }

	// state_cost += distance / (ROAD_WIDTH / 2) * TRACK_ERR_COST;
  state_cost += distance*distance/(ROAD_WIDTH*ROAD_WIDTH/4) * TRACK_ERR_COST;
    // state_cost += distance*distance* TRACK_ERR_COST*prediction_decay;

	// state_cost += 1.3*distance*distance/(ROAD_WIDTH/4*ROAD_WIDTH) * TRACK_ERR_COST;
	// Obstacle avoidance penalty

	float collision = 0.0;
	for (int i = 0; obstacle[i].x != NULL; i++) {
		collision = fabs(obstacleCollision(&state, &obstacle[i]));
		state_cost += collision * COLLISION_COST;
    }
	cost_list[idx] = state_cost;
}


__global__ void calcRolloutCost(float* input, float* output) {
  /*  Calculate the total cost of each rollout and store it into an 
    array with reduced size (reduced sum no bank conflict)*/
    // Allocate shared memory
  __shared__ float partial_sum[SHMEM_SIZE];

  int tid = blockIdx.x * blockDim.x + threadIdx.x;

  partial_sum[threadIdx.x] = input[tid];
  __syncthreads();

  for (int s = blockDim.x / 2; s > 0; s >>= 1) {
    if (threadIdx.x < s) {
      partial_sum[threadIdx.x] += partial_sum[threadIdx.x + s];
    }
    __syncthreads();
  }

  if (threadIdx.x == 0) {
    output[blockIdx.x] = partial_sum[0];
  }
}

__global__ void calcRolloutCost2(float* input, float* output) {
	/*  Sequential addressing (reduction sum)*/
  __shared__ float partial_sum[SHMEM_SIZE];

  int tid = blockIdx.x * blockDim.x + threadIdx.x;

  partial_sum[threadIdx.x] = input[tid];
  __syncthreads();

  for (int s = 1; s < blockDim.x; s *= 2) {
    int index = 2 * s * threadIdx.x;

    if (index < blockDim.x) {
      partial_sum[index] += partial_sum[index + s];
    }
    __syncthreads();
  }

  if (threadIdx.x == 0) {
    output[blockIdx.x] = partial_sum[0];
  }
}

__global__ void min_reduction(float* input, float* output) {
  /*  Find the rollout with the minimum cost using sum reduction methods, 
    but changing the "sum" part to "min"*/

  __shared__ float partial_sum[K/32];
  int i = blockIdx.x * (blockDim.x * 2) + threadIdx.x;

  partial_sum[threadIdx.x] = min(input[i], input[i + blockDim.x]);
  __syncthreads();

  for (int s = blockDim.x / 2; s > 0; s >>= 1) {
    if (threadIdx.x < s) {
      partial_sum[threadIdx.x] = min(partial_sum[threadIdx.x], partial_sum[threadIdx.x + s]);
    }
    __syncthreads();
  }

  if (threadIdx.x == 0) {
    output[blockIdx.x] = partial_sum[0];
    //printf("result: %.3f\n", partial_sum[0]);
  }
}

__global__ void min_reduction2(float* input, float* output) {
  /*  Find the rollout with the minimum cost using sum reduction methods,
    but changing the "sum" part to "min"*/

  __shared__ float partial_sum[K / 32];
  int i = blockIdx.x * blockDim.x + threadIdx.x;

  partial_sum[threadIdx.x] = input[i];
  __syncthreads();

  for (int s = blockDim.x /4; s > 0; s >>= 1) {
    if (threadIdx.x < s) {
      partial_sum[threadIdx.x] = min(partial_sum[threadIdx.x], partial_sum[threadIdx.x + s]);
    }
    __syncthreads();
  }

  if (threadIdx.x == 0) {
    output[blockIdx.x] = partial_sum[0];
  }
}

__global__ void calcWTilde(float* w_tilde, float* rho, float* cost_list) {
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  w_tilde[idx] = __expf(-1/LAMBDA*(cost_list[idx]-rho[0]));

  // if(threadIdx.x==0){
  //   printf("rho=%4.2f \n", rho[0]);
  // }
}

__global__ void genW(Control* u_opt, float* eta, float* w_tilde, Control* V) {
  /*  multiplies the calculated weight of each rollout to its designated predicted controls (N_HRZ)
    u_opt [K,N_HRZ], eta[K], w_tilde[K], V[L, N_HRZ]*/

  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  __shared__ float w;
  w = w_tilde[blockIdx.x] / eta[0];
  u_opt[idx].vd = V[idx].vd * w;
  u_opt[idx].wd = V[idx].wd * w;
  //if(blockIdx.x>1000)printf("%d %dw: %.3f\n", blockIdx.x, idx, w);
}

__global__ void genWCorrect(Control* u_opt, float* eta, float* w_tilde, Control* V, Control* U) {
	/*  multiplies the calculated weight of each rollout to the perturbation
		Note that u_opt is completed with U added */

	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	__shared__ float w;
	w = w_tilde[blockIdx.x] / eta[0];
	u_opt[idx].vd = (V[idx].vd - U[threadIdx.x].vd) * w;
	u_opt[idx].wd = (V[idx].wd - U[threadIdx.x].wd) * w;
}

__global__ void wsum_reduction(Control *input, Control* output) {
  /*  Calculate the total weighted control of every prediction horizon. 
    Input [K,N_HRZ]; Output [N_HRZ]*/
  
  __shared__ Control partial_sum[1024];

  int tid = threadIdx.x * gridDim.x + blockIdx.x;
  partial_sum[threadIdx.x] = input[tid];
  __syncthreads();

  for (int s = blockDim.x / 2; s > 0; s >>= 1) {
    if (threadIdx.x < s) {
      partial_sum[threadIdx.x].vd += partial_sum[threadIdx.x + s].vd;
      partial_sum[threadIdx.x].wd += partial_sum[threadIdx.x + s].wd;
    }
    __syncthreads();
  }

  if (threadIdx.x == 0) {
    output[blockIdx.x] = partial_sum[0];
  }
}

__global__ void sumControl(Control* input, Control* output) {
  /*  Function that sums the input with the output to the output 
    (output = input + output)*/

  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  output[idx].vd += input[idx].vd;
  output[idx].wd += input[idx].wd;
}

__global__ void wsum_reduction_partial(Control* input, Control* output, int shift) {
  /*  Sum partial (used when the rollout predictions acceed harware constrains, when #threads too large) 
    weighted controls of the prediction horizon (sum along columns), can use var shift 
    to shift the starting tid*/

  __shared__ Control partial_sum[1024];

  int tid = (threadIdx.x + shift) * gridDim.x + blockIdx.x;  // top-down (column-wise) order
  partial_sum[threadIdx.x] = input[tid];
  __syncthreads();

  for (int s = blockDim.x / 2; s > 0; s >>= 1) {
    if (threadIdx.x < s) {
      partial_sum[threadIdx.x].vd += partial_sum[threadIdx.x + s].vd;
      partial_sum[threadIdx.x].wd += partial_sum[threadIdx.x + s].wd;
    }
    __syncthreads();
  }

  if (threadIdx.x == 0) {
    output[blockIdx.x] = partial_sum[0];
  }
}

__global__ void PertCost(float* costList, Control* rand, Control* u) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  // costList[i] += LAMBDA * (u[threadIdx.x].vd * V_RAND_SCALAR * rand[i].vd
  //                + u[threadIdx.x].wd * W_RAND_SCALAR * rand[i].wd); // Performs great but wrong?
  // costList[i] += LAMBDA *0.05 * (fabs(u[threadIdx.x].vd) / V_RAND_SCALAR * fabs(rand[i].vd-u[threadIdx.x].vd)
  //                + fabs(u[threadIdx.x].wd) / W_RAND_SCALAR * fabs(rand[i].wd-u[threadIdx.x].wd));

  // costList[i] += LAMBDA *0.007 * (fabs(rand[i].vd-u[threadIdx.x].vd) / V_RAND_SCALAR * fabs(rand[i].vd-u[threadIdx.x].vd)
  //                + fabs(rand[i].wd-u[threadIdx.x].wd) / W_RAND_SCALAR * fabs(rand[i].wd-u[threadIdx.x].wd));
  // costList[i] += LAMBDA * 0.001* (fabs(u[threadIdx.x].vd) / V_RAND_SCALAR * fabs(rand[i].vd-u[threadIdx.x].vd)
  //                + (fabs(u[threadIdx.x].wd)+2) / W_RAND_SCALAR * (fabs(rand[i].wd-u[threadIdx.x].wd)));
  // if(i<100){
  //   printf("index: %d %d \n", i, threadIdx.x);
  // }
  // if(i<100){
  //   printf("-> %.3f %.3f %.3f\n", rand[i].vd,u[threadIdx.x].vd,((u[threadIdx.x].vd) / V_RAND_SCALAR * (rand[i].vd-u[threadIdx.x].vd)
  //                + ((u[threadIdx.x].wd)) / W_RAND_SCALAR * ((rand[i].wd-u[threadIdx.x].wd))));
  // }
  // costList[i] += LAMBDA * 0.0001* ((u[threadIdx.x].vd) / V_RAND_SCALAR * (rand[i].vd-u[threadIdx.x].vd)
  //                + ((u[threadIdx.x].wd)) / W_RAND_SCALAR * ((rand[i].wd-u[threadIdx.x].wd)));    

  // costList[i] += LAMBDA *0.0001* fabs((u[threadIdx.x].vd) / V_RAND_SCALAR / V_RAND_SCALAR * fabs(rand[i].vd-u[threadIdx.x].vd)
  //                + fabs((u[threadIdx.x].wd)) / W_RAND_SCALAR / W_RAND_SCALAR * fabs((rand[i].wd-u[threadIdx.x].wd)));      
  float inv1 = 1/(W_RAND_SCALAR*W_RAND_SCALAR);
  float inv2 = 1/(V_RAND_SCALAR*V_RAND_SCALAR);
  costList[i] += LAMBDA * 0.001*((u[threadIdx.x].vd) * inv1 * (rand[i].vd-u[threadIdx.x].vd)
                 + ((u[threadIdx.x].wd)) * inv2 * ((rand[i].wd-u[threadIdx.x].wd)));            


  // costList[i] += LAMBDA *0.1* (rand[i].vd / V_RAND_SCALAR * rand[i].vd
  //               + rand[i].wd / W_RAND_SCALAR * rand[i].wd;
  // costList[i] += LAMBDA * ((rand[i].vd-u[threadIdx.x].vd) * V_RAND_SCALAR * (rand[i].vd-u[threadIdx.x].vd)
  //               + (rand[i].wd-u[threadIdx.x].wd) * W_RAND_SCALAR * (rand[i].wd-u[threadIdx.x].wd));
}

__global__ void printControl(Control* u) {
  printf("%.3f %.3f\n", u[0].vd, u[0].wd);
}
__global__ void printMinCost(float* c) {
  printf("min cost: %.3f\n", c[0]);
}

__global__ void line() {
  printf("=====================\n");
}

__global__ void printfloat(float* f) {
  int i = blockIdx.x * (blockDim.x * 2) + threadIdx.x;
  printf("float: %.3f\n", f[i]);
}

__host__ Pos getIntersec(Track line1, Track line2) {
  /*  Calculate the intersection point of two line fcns*/
  Pos intersec;
  intersec.x = (line1.b * line2.c - line2.b * line1.c) / (line1.a * line2.b - line2.a * line1.b);
  intersec.y = (line1.c * line2.a - line2.c * line1.a) / (line1.a * line2.b - line2.a * line1.b);
  return intersec;
}

// __host__ void build_track_(Track* inner_fcn, Track* outer_fcn, Pos* mid_intersec, Pos* inner_intersec,
//   Pos* outer_intersec, Track* mid_fcn) {
//   /*  Build the boundaries of a designated path, also return 
//     the intersection points of the middle path and boundaries*/
//   float mid_m[4] = { -mid_fcn[0].a, -mid_fcn[1].a, -mid_fcn[2].a, -mid_fcn[3].a };
  
//   memcpy(outer_fcn, mid_fcn, 4 * sizeof(Track));
//   outer_fcn[0].c += (ROAD_WIDTH / 2) / cos(atan(mid_m[0]));
//   outer_fcn[1].c -= (ROAD_WIDTH / 2) / cos(atan(mid_m[1]));
//   outer_fcn[2].c -= (ROAD_WIDTH / 2) / cos(atan(mid_m[2]));
//   outer_fcn[3].c += (ROAD_WIDTH / 2) / cos(atan(mid_m[3]));
  
//   memcpy(inner_fcn, mid_fcn, 4 * sizeof(Track));
//   inner_fcn[0].c -= (ROAD_WIDTH / 2) / cos(atan(mid_m[0]));
//   inner_fcn[1].c += (ROAD_WIDTH / 2) / cos(atan(mid_m[1]));
//   inner_fcn[2].c += (ROAD_WIDTH / 2) / cos(atan(mid_m[2]));
//   inner_fcn[3].c -= (ROAD_WIDTH / 2) / cos(atan(mid_m[3]));

//   mid_intersec[0] = getIntersec(mid_fcn[0], mid_fcn[1]);
//   mid_intersec[1] = getIntersec(mid_fcn[1], mid_fcn[2]);
//   mid_intersec[2] = getIntersec(mid_fcn[2], mid_fcn[3]);
//   mid_intersec[3] = getIntersec(mid_fcn[3], mid_fcn[0]);

//   inner_intersec[0] = getIntersec(inner_fcn[0], inner_fcn[1]);
//   inner_intersec[1] = getIntersec(inner_fcn[1], inner_fcn[2]);
//   inner_intersec[2] = getIntersec(inner_fcn[2], inner_fcn[3]);
//   inner_intersec[3] = getIntersec(inner_fcn[3], inner_fcn[0]);

//   outer_intersec[0] = getIntersec(outer_fcn[0], outer_fcn[1]);
//   outer_intersec[1] = getIntersec(outer_fcn[1], outer_fcn[2]);
//   outer_intersec[2] = getIntersec(outer_fcn[2], outer_fcn[3]);
//   outer_intersec[3] = getIntersec(outer_fcn[3], outer_fcn[0]);
// }

__host__ Track getNominal(Track line, Pos point) {
	Track nominal;
	nominal.a = -1 / line.a;
	nominal.b = line.b;
	nominal.c = -(nominal.a * point.x + nominal.b * point.y);
	return nominal;
}

__host__ void build_track(Track* inner_fcn, Track* outer_fcn, Pos* mid_intersec, Pos* inner_intersec,
	Pos* outer_intersec, Track* mid_fcn, Pos* turn_center_intersec, Track* nominal_fcn) {
	/*  Build the boundaries of a designated path, also return
	the intersection points of the middle path and boundaries
 *  Index of the path names
 *          g
 *        ______
 *       /     /
 *    h /     / f
 *     /     /
 *     ------
 *        i
 *  f, g intersection : [x1, y1] f: 0
 *  g, h intersection : [x2, y2] g: 1
 *  h, i intersection : [x3, y3] h: 2
 *  i, f intersection : [x4, y4] i: 3 */

	float mid_m[4] = { -mid_fcn[0].a, -mid_fcn[1].a, -mid_fcn[2].a, -mid_fcn[3].a };

	memcpy(outer_fcn, mid_fcn, 4 * sizeof(Track));
	outer_fcn[0].c += (ROAD_WIDTH / 2) / cos(atan(mid_m[0]));
	outer_fcn[1].c -= (ROAD_WIDTH / 2) / cos(atan(mid_m[1]));
	outer_fcn[2].c -= (ROAD_WIDTH / 2) / cos(atan(mid_m[2]));
	outer_fcn[3].c += (ROAD_WIDTH / 2) / cos(atan(mid_m[3]));

	memcpy(inner_fcn, mid_fcn, 4 * sizeof(Track));
	inner_fcn[0].c -= (ROAD_WIDTH / 2) / cos(atan(mid_m[0]));
	inner_fcn[1].c += (ROAD_WIDTH / 2) / cos(atan(mid_m[1]));
	inner_fcn[2].c += (ROAD_WIDTH / 2) / cos(atan(mid_m[2]));
	inner_fcn[3].c -= (ROAD_WIDTH / 2) / cos(atan(mid_m[3]));

	mid_intersec[0] = getIntersec(mid_fcn[0], mid_fcn[1]);
	mid_intersec[1] = getIntersec(mid_fcn[1], mid_fcn[2]);
	mid_intersec[2] = getIntersec(mid_fcn[2], mid_fcn[3]);
	mid_intersec[3] = getIntersec(mid_fcn[3], mid_fcn[0]);

	inner_intersec[0] = getIntersec(inner_fcn[0], inner_fcn[1]);
	inner_intersec[1] = getIntersec(inner_fcn[1], inner_fcn[2]);
	inner_intersec[2] = getIntersec(inner_fcn[2], inner_fcn[3]);
	inner_intersec[3] = getIntersec(inner_fcn[3], inner_fcn[0]);

	outer_intersec[0] = getIntersec(outer_fcn[0], outer_fcn[1]);
	outer_intersec[1] = getIntersec(outer_fcn[1], outer_fcn[2]);
	outer_intersec[2] = getIntersec(outer_fcn[2], outer_fcn[3]);
	outer_intersec[3] = getIntersec(outer_fcn[3], outer_fcn[0]);

	// Setup the curves
	Track fg_inner_tmp[2] = { mid_fcn[0], mid_fcn[1] };
	Track gh_inner_tmp[2] = { mid_fcn[1], mid_fcn[2] };
	Track hi_inner_tmp[2] = { mid_fcn[2], mid_fcn[3] };
	Track if_inner_tmp[2] = { mid_fcn[3], mid_fcn[0] };

	// Shift the mid fcn inwards by the turning radius
	fg_inner_tmp[0].c -= (FG_RAD) / cos(atan(mid_m[0]));
	fg_inner_tmp[1].c += (FG_RAD) / cos(atan(mid_m[1]));

	gh_inner_tmp[0].c += (GH_RAD) / cos(atan(mid_m[1]));
	gh_inner_tmp[1].c += (GH_RAD) / cos(atan(mid_m[2]));

	hi_inner_tmp[0].c += (HI_RAD) / cos(atan(mid_m[2]));
	hi_inner_tmp[1].c -= (HI_RAD) / cos(atan(mid_m[3]));

	if_inner_tmp[0].c -= (IF_RAD) / cos(atan(mid_m[3]));
	if_inner_tmp[1].c -= (IF_RAD) / cos(atan(mid_m[0]));

	// Get the center of the turning curves
	turn_center_intersec[0] = getIntersec(fg_inner_tmp[0], fg_inner_tmp[1]);
	turn_center_intersec[1] = getIntersec(gh_inner_tmp[0], gh_inner_tmp[1]);
	turn_center_intersec[2] = getIntersec(hi_inner_tmp[0], hi_inner_tmp[1]);
	turn_center_intersec[3] = getIntersec(if_inner_tmp[0], if_inner_tmp[1]);

	nominal_fcn[0] = getNominal(mid_fcn[0], turn_center_intersec[0]);
	nominal_fcn[1] = getNominal(mid_fcn[1], turn_center_intersec[0]);
	nominal_fcn[2] = getNominal(mid_fcn[1], turn_center_intersec[1]);
	nominal_fcn[3] = getNominal(mid_fcn[2], turn_center_intersec[1]);
	nominal_fcn[4] = getNominal(mid_fcn[2], turn_center_intersec[2]);
	nominal_fcn[5] = getNominal(mid_fcn[3], turn_center_intersec[2]);
	nominal_fcn[6] = getNominal(mid_fcn[3], turn_center_intersec[3]);
	nominal_fcn[7] = getNominal(mid_fcn[0], turn_center_intersec[3]);
}

__device__ Track* dev_in_fcn;
__device__ Track* dev_out_fcn;
__device__ Track* dev_nominal_fcn;
__device__ Pos* dev_turn_circle_intersec;

void trackInit(){
  // Build track
  Track host_mid_fcn[] = { 
  {1.0000, -1.33200, 82.62978},
  {1.0000, 0.75640, -240.86623},
  {1.0000, -1.36070, -33.13473},
  {1.0000, 0.47203, -35.00739}
  };
  Track* dev_mid_fcn,* host_in_fcn, * host_out_fcn, *host_nominal_fcn;
  Pos* host_mid_intersec, * host_in_intersec, * host_out_intersec, *host_turn_circle_intersec;
  Pos* dev_mid_intersec, * dev_in_intersec, * dev_out_intersec;    

  host_in_fcn = (Track*)malloc(4 * sizeof(Track));
  host_out_fcn = (Track*)malloc(4 * sizeof(Track));
  host_nominal_fcn = (Track*)malloc(8 * sizeof(Track));
  host_mid_intersec = (Pos*)malloc(4 * sizeof(Pos));
  host_in_intersec = (Pos*)malloc(4 * sizeof(Pos));
  host_out_intersec = (Pos*)malloc(4 * sizeof(Pos));
  host_turn_circle_intersec = (Pos*)malloc(4 * sizeof(Pos));

  cudaMalloc((void**)&dev_mid_fcn, 4 * sizeof(Track));
  cudaMalloc((void**)&dev_in_fcn, 4 * sizeof(Track));
  cudaMalloc((void**)&dev_out_fcn, 4 * sizeof(Track));
  cudaMalloc((void**)&dev_nominal_fcn, 8 * sizeof(Track));
  cudaMalloc((void**)&dev_mid_intersec, 4 * sizeof(Pos));
  cudaMalloc((void**)&dev_in_intersec, 4 * sizeof(Pos));
  cudaMalloc((void**)&dev_out_intersec, 4 * sizeof(Pos));
  cudaMalloc((void**)&dev_turn_circle_intersec, 4 * sizeof(Pos));

  // build_track(host_in_fcn, host_out_fcn, host_mid_intersec, host_in_intersec, host_out_intersec, host_mid_fcn);
  build_track(host_in_fcn, host_out_fcn, host_mid_intersec, host_in_intersec, host_out_intersec, host_mid_fcn, host_turn_circle_intersec, host_nominal_fcn);
  for(int i = 0; i < 8; i ++){
    std::cout<<host_nominal_fcn[i].b<<"y+"<<host_nominal_fcn[i].a<<"x+"<<host_nominal_fcn[i].c<<"=0"<<std::endl;
  }
  cudaMemcpy(dev_in_fcn, host_in_fcn, 4 * sizeof(Track), cudaMemcpyHostToDevice);
  cudaMemcpy(dev_out_fcn, host_out_fcn, 4 * sizeof(Track), cudaMemcpyHostToDevice);
  cudaMemcpy(dev_nominal_fcn, host_nominal_fcn, 8 * sizeof(Track), cudaMemcpyHostToDevice);
  cudaMemcpy(dev_turn_circle_intersec, host_turn_circle_intersec, 4 * sizeof(Pos), cudaMemcpyHostToDevice);
}

void kernelmain(Control* output_cmd, Control* host_U, State* output_state, int freq, float* cost, Obs* obs_pos, int obs_num){

  // Record runtime 
  cudaEvent_t start, stop;
  cudaEventCreate(&start);
  cudaEventCreate(&stop);
  auto startTim = chrono::steady_clock::now();

  // Setup parameters and Initialize variables
  Control* host_V, * dev_V;
  Control* dev_U;
  Control* dev_u_opt, * dev_u_opt_part, * host_u_opt;
  Control* dev_temp_u_opt;
  Control output_u0;  // might not be necessary
  State host_x0 = *output_state;
  State* dev_x0;
  State* dev_stateList, * host_stateList;
  
  float* dev_state_costList;
  float* dev_rollout_costList;
  float* host_rollout_costList;
  float* host_rho;
  float* dev_rho;
  float* dev_w_tilde;
  float* dev_eta;

  //Build obstacle
  Obs* dev_obstacle;

  bool no_obs = false;
  if(obs_num<=0){
    obs_num = 1;
    no_obs = true;
  }
  Obs host_obstacle[obs_num] = {0};
  for(int i = 0; i < obs_num; i ++){
    if(!no_obs){
      Obs temp = obs_pos[i];
      // temp.x += host_x0.x;
      // temp.y += host_x0.y;
      // temp.y += 3;
      // temp.r = OBS_SIZE;
      // temp.r = temp.r;
      host_obstacle[i] = temp;
    }else{
      Obs temp = {0,0,0};
      host_obstacle[i] = temp;
    }
    // std::cout<<"Obs: " <<host_obstacle[i].x<<" "<<host_obstacle[i].y<<" "<<host_obstacle[i].r<<" ("<<host_x0.x<<","<<host_x0.y<<")"<<std::endl;
  }

  // Obs host_obstacle[] = {
  // // {154 ,126.2 ,1},
  // // {153 ,124.5 ,1},
  // // {152 ,120.5 ,1},
  // {128 ,87.5 ,1.0},
  // // {152 ,119   ,1},
  // {134, 97, 1}
  // };

  // /* Check the influence of noise */
  // for(int i = 0; i < 5; i ++){
  //   host_obstacle[i].x += 3*rand()/RAND_MAX;
  //   host_obstacle[i].y += 3*rand()/RAND_MAX;
  // }

  // Obs host_obstacle[] = {
  // {128 ,87.5 ,1.0},
  // {152 ,120.5 ,1.0},
  // {152 ,119   ,1.0}
  // };

  // Obs host_obstacle[] = {0};

  // Obs host_obstacle[] = {0};
  int NUM_OBS = sizeof(host_obstacle) / sizeof(Obs);
  // std::cout<<"Num of obs: "<< NUM_OBS<<" = "<<obs_num<<std::endl;

  // Setup host memory
  host_V = (Control*)malloc(int(K * N_HRZ) * sizeof(Control));
  host_u_opt = (Control*)malloc(int(N_HRZ) * sizeof(Control));
  host_stateList = (State*)malloc(int(K * N_HRZ) * sizeof(State));
  host_rollout_costList = (float*)malloc(int(K) * sizeof(float));
  host_rho = (float*)malloc(sizeof(float));

  // Setup device memory
  cudaMalloc((void**)&dev_V, int(K * N_HRZ) * sizeof(Control));
  cudaMalloc((void**)&dev_x0, sizeof(State));
  cudaMalloc((void**)&dev_stateList, int(K * N_HRZ) * sizeof(State));
  cudaMalloc((void**)&dev_state_costList, int(K * N_HRZ) * sizeof(float));
  cudaMalloc((void**)&dev_rollout_costList, int(K) * sizeof(float));
  cudaMalloc((void**)&dev_rho, int(K) * sizeof(float));
  cudaMalloc((void**)&dev_w_tilde, int(K) * sizeof(float));
  cudaMalloc((void**)&dev_eta, int(K) * sizeof(float));
  cudaMalloc((void**)&dev_u_opt, int(N_HRZ) * sizeof(Control));
  cudaMalloc((void**)&dev_u_opt_part, int(N_HRZ) * sizeof(Control));
  cudaMalloc((void**)&dev_temp_u_opt, int(K * N_HRZ) * sizeof(Control));
  cudaMalloc((void**)&dev_U, int(N_HRZ) * sizeof(Control));
  cudaMalloc((void**)&dev_obstacle, NUM_OBS * sizeof(Obs));

  // Setup constant memory
  cudaMemcpy(dev_obstacle, host_obstacle, NUM_OBS * sizeof(Obs), cudaMemcpyHostToDevice);

  // Initialize nominal conrtrol
  // for (int n = 0; n < N_HRZ; n++) {
  // //host_U[n].vd = host_u0[0];
  // //host_U[n].wd = host_u0[1];
  // host_U[n].vd = 0;
  // host_U[n].wd = 0;
  // }

  // Start actual MPC iteration
  cudaMemcpy(dev_U, host_U, int(N_HRZ) * sizeof(Control), cudaMemcpyHostToDevice);
  cudaMemcpy(dev_x0, &host_x0, sizeof(State), cudaMemcpyHostToDevice);

  // Launch kernal functions
  genControlNew << <K, int(N_HRZ) >> > (time(NULL), dev_V, dev_U);  // Initialize random control perturbations to the nominal control U -> V

  genState << <K / 256, K / 4 >> > (dev_stateList, dev_x0, dev_V);  // Generate the states in each trajectory using the control sequences

  // costFcn << <int(N_HRZ) * 4, K / 4 >> > (dev_state_costList, dev_stateList, dev_out_fcn, dev_in_fcn, dev_obstacle); // Calculate the cost of all states individually
  costFcn << <int(N_HRZ) * 4, K / 4>> > (dev_state_costList, dev_stateList, dev_out_fcn, dev_in_fcn, dev_nominal_fcn, dev_obstacle, dev_turn_circle_intersec);
  
  PertCost << <K, int(N_HRZ) >> > (dev_state_costList, dev_V, dev_U);

  calcRolloutCost2 << <K, int(N_HRZ) >> > (dev_state_costList, dev_rollout_costList);  // Get trajectory cost of all rollouts by perfoming sum reduction once

  min_reduction << <K / 32 / 2, K / 32 >> > (dev_rollout_costList, dev_rho);  // Get minimum trajectory cost by performing min reduction twice
  min_reduction2 << <1, K / 32 >> > (dev_rho, dev_rho);

  calcWTilde << <K / 32, K / 32 >> > (dev_w_tilde, dev_rho, dev_rollout_costList);  // Calculate w_tilde

  sum_reduction << <K / 32 / 2, K / 32 >> > (dev_w_tilde, dev_eta); // Get eta by performing sum reduction twice
  sum_reduction << <1, K / 32 >> > (dev_eta, dev_eta);

  genWCorrect << <K, int(N_HRZ) >> > (dev_temp_u_opt, dev_eta, dev_w_tilde, dev_V, dev_U);  // Compute (control_pert)*w_i (NOMINAL CONTROL NOT ADDED, will be added later)

  wsum_reduction << <N_HRZ, K >> > (dev_temp_u_opt, dev_u_opt); // Sum up the weighted perturbations to form the optimal control sequence (NOMINAL CONTROL NOT ADDED)

  // If 1024 threads are too much for the hardware, the method below can be adapted @TODO
  //wsum_reduction_partial << <40, 512 >> > (dev_temp_u_opt, dev_u_opt, 0);
  //wsum_reduction_partial output_u0<< <40, 512 >> > (dev_temp_u_opt, dev_u_opt_part, 512);
  //sumControl << <20, 20 >> > (dev_u_opt_part, dev_u_opt);

  // Acts as a syncing buffer so no need for additional synhronization
  cudaMemcpy(host_rho, dev_rho, sizeof(float), cudaMemcpyDeviceToHost);
  // cudaMemcpy(host_V, dev_V, int(K * N_HRZ) * sizeof(Control), cudaMemcpyDeviceToHost);
  // cudaMemcpy(host_stateList, dev_stateList, int(K * N_HRZ) * sizeof(State), cudaMemcpyDeviceToHost);
  cudaMemcpy(host_u_opt, dev_u_opt, int(N_HRZ) * sizeof(Control), cudaMemcpyDeviceToHost);
  // cudaMemcpy(host_rollout_costList, dev_rollout_costList, int(K) * sizeof(float), cudaMemcpyDeviceToHost);

  // if use genWCorrect (NOMINAL CONTROL ADDED HERE)
  for (int i = 0; i < N_HRZ; i++) {
    host_u_opt[i].vd += host_U[i].vd;
    host_u_opt[i].wd += host_U[i].wd;
  }

  // Generate the predicted next state with u_opt
  // for(int i=0; i<freq/F_STP-1; i++){
  for(int i=0; i<freq/F_STP; i++){
    host_x0 = bicycleModel(host_x0, host_u_opt[0]);
  }

  // Shift the control by 1 
  output_u0 = host_u_opt[0];
  memmove(host_u_opt, &host_u_opt[1], int(N_HRZ - 1) * sizeof(Control));  // use memmove instead of memcpy because the destination overlaps the source
  // host_u_opt[int(N_HRZ)- 1] = host_u_opt[int(N_HRZ) - 1 - 1];             // use the former last control in the sequence as the shifted last control initial value
  host_u_opt[int(N_HRZ)- 1].vd = 2;
  host_u_opt[int(N_HRZ)- 1].wd = 0;
  memcpy(host_U, host_u_opt, int(N_HRZ) * sizeof(Control));

  // Output the commands back to main
  clampingFcn(&output_u0,1);
  output_cmd->vd = output_u0.vd;
  output_cmd->wd = output_u0.wd;

  // Update the output state to the new state
  output_state->x = host_x0.x;
  output_state->y = host_x0.y;
  output_state->phi = host_x0.phi;
  output_state->v0 = host_x0.v0;
  output_state->theta = host_x0.theta;
  output_state->theta_dot = host_x0.theta_dot;
  output_state->delta = host_x0.delta;

  *cost = *host_rho;
  

  cudaFree(dev_V);
  cudaFree(dev_x0);
  cudaFree(dev_stateList);
  cudaFree(dev_state_costList);
  cudaFree(dev_rollout_costList);
  cudaFree(dev_rho);
  cudaFree(dev_w_tilde);
  cudaFree(dev_eta);
  cudaFree(dev_u_opt);
  cudaFree(dev_u_opt_part);
  cudaFree(dev_temp_u_opt);
  cudaFree(dev_U);
  cudaFree(dev_obstacle);

  // for(int i=0; i<N_HRZ; i++){
  //    std::cout<<host_u_opt[i].wd<<" ";
  // }
  // std::cout<<std::endl;
  // for(int i=0; i<N_HRZ; i++){
  //    std::cout<<host_u_opt[i].vd<<" ";
  // }
  // std::cout<<std::endl;

  free(host_V);
  free(host_u_opt);
  free(host_stateList);
  free(host_rollout_costList);

  // std::cout << "Control: " << output_cmd->vd<<" "<< output_cmd->wd << " x: "<< output_state->x<< " y: "<< output_state->y<<std::endl;
  // Record the time of one oteration
  auto endTim = chrono::steady_clock::now();  
  auto diff = endTim - startTim;
  // cout <<  "Runtime: " << chrono::duration <double, milli>(diff).count() << " ms"<< endl;

  
  

  // static double simtim = 0;
  // // Output .csv files
  // std::fstream AAA;
  // // create a name for the file output
  // AAA.open("AAA.csv", std::ios::out | std::ios::app);
  // AAA << simtim << " " << output_state->x << " " << output_state->y << " " << output_state->phi << " " << output_state->v0 << " " << output_u0.vd << " " << output_u0.wd << " " << std::endl;
  // AAA.close();
  // simtim += T_STP;
}
