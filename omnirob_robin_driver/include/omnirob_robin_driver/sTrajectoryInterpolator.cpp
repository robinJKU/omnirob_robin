#define S_FUNCTION_NAME sTrajectoryInterpolator
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include <vector>
#include "cubic_trajectory_generator.h"
#include <stdio.h>
#include <cmath>


/* meta data -------------*/
#define NR_OF_TRAJECTORIES 7
#define CUT_OFF_FREQUENCY_LOW_PASS 6.0 // (rad/s) is used for smoothing start and stop manoeuvres

/* params ---------------*/
#define NUMSFCNPARAMS    4
#define PARAM_TS     0
#define PARAM_v_max  1
#define PARAM_a_max  2
#define PARAM_j_max  3

/* states ---------------*/
#define NUMCONTSTATES    0
#define NUMDISCSTATES    0

/* inputs ----------------*/
#define NUMINPUTPORTS 5
// 0
#define INPUT_qi         0
#define FEEDTHROUGHPORT0 1
#define INPUTPORT0WIDTH  NR_OF_TRAJECTORIES
// 1
#define INPUT_reset_trajectory  1
#define FEEDTHROUGHPORT1        1
#define INPUTPORT1WIDTH         1
// 2
#define INPUT_add_point_to_trajectory  2
#define FEEDTHROUGHPORT2 1
#define INPUTPORT2WIDTH  1
// 3
#define INPUT_start_trajectory  3
#define FEEDTHROUGHPORT3        1
#define INPUTPORT3WIDTH         1
// 4
#define INPUT_stop_trajectory   4
#define FEEDTHROUGHPORT4        1
#define INPUTPORT4WIDTH         1

/* work vector ------------------ */
#define WORK_NR_OF_PTR	2
#define WORK_PTR_S_NODE	0
#define WORK_PTR_TRAJECTORIES_OBJECT	1

#define WORK_NR_OF_INT 3
#define WORK_INT_TRAJECTROY_IS_ENABLED 0
#define WORK_INT_RESET_TRAJECTROY_MODE 1
#define WORK_INT_STOP_TRAJECTORY	   2

#define WORK_NR_OF_FLOAT 5
#define WORK_FLOAT_TRAJECTORY_TIME 0
#define WORK_FLOAT_TRAJECTORY_TIME_FILTERED 1
#define WORK_FLOAT_FILTER_COEFF_A1 2
#define WORK_FLOAT_FILTER_COEFF_B0 3
#define WORK_FLOAT_FILTER_COEFF_B1 4

/* outputs -----------------*/
#define NUMOUTPUTPORTS 3
// 0
#define OUTPUT_q          0
#define OUTPUTPORT0WIDTH  NR_OF_TRAJECTORIES
// 1
#define OUTPUT_t_E        1
#define OUTPUTPORT1WIDTH  1
// 2
#define OUTPUT_time_left  2
#define OUTPUTPORT2WIDTH  1

/* init size ---------------------------------------- */
static void mdlInitializeSizes(SimStruct *S)
{
  // params
  ssSetNumSFcnParams(S,NUMSFCNPARAMS);
  if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
    return; /* Parameter mismatch will be reported by Simulink */
  }
  
  // states
  ssSetNumContStates(S, NUMCONTSTATES);
  ssSetNumDiscStates(S, NUMDISCSTATES);

  // inputs
  if (!ssSetNumInputPorts(S,NUMINPUTPORTS)) return;
  ssSetInputPortWidth(S,0,INPUTPORT0WIDTH);
  ssSetInputPortDirectFeedThrough(S,0,FEEDTHROUGHPORT0);
  ssSetInputPortWidth(S,1,INPUTPORT1WIDTH);
  ssSetInputPortDirectFeedThrough(S,1,FEEDTHROUGHPORT1);
  ssSetInputPortWidth(S,2,INPUTPORT2WIDTH);
  ssSetInputPortDirectFeedThrough(S,2,FEEDTHROUGHPORT2);
  ssSetInputPortWidth(S,3,INPUTPORT3WIDTH);
  ssSetInputPortDirectFeedThrough(S,3,FEEDTHROUGHPORT3);
  ssSetInputPortWidth(S,4,INPUTPORT4WIDTH);
  ssSetInputPortDirectFeedThrough(S,4,FEEDTHROUGHPORT4);

  // outputs
  if (!ssSetNumOutputPorts(S,NUMOUTPUTPORTS)) return;
  ssSetOutputPortWidth(S,0,OUTPUTPORT0WIDTH);
  ssSetOutputPortWidth(S,1,OUTPUTPORT1WIDTH);
  ssSetOutputPortWidth(S,2,OUTPUTPORT2WIDTH);

  // sample time
  ssSetNumSampleTimes(S,1);
  ssSetNumModes(S,0);
  ssSetNumNonsampledZCs(S,0);
  ssSetOptions(S,0);
  
  // work vectors
  ssSetNumPWork(S, WORK_NR_OF_PTR);
  ssSetNumRWork(S, WORK_NR_OF_FLOAT);
  ssSetNumIWork(S, WORK_NR_OF_INT);
  
}// init size

/* init sample time ---------------------------------------- */
static void mdlInitializeSampleTimes(SimStruct *S)
{
  real_T *Ts =(real_T*) mxGetPr( ssGetSFcnParam(S, PARAM_TS));
  ssSetSampleTime(S,0,Ts[0]);
  ssSetOffsetTime(S,0,0.0);
}// init sample time

/* start ----------------------------------------------- */
#define MDL_START  /* Change to #undef to remove function */
#if defined(MDL_START)
static void mdlStart(SimStruct *S)
{
    // check if parameter exist and are valid
	const mxArray *v_max_ptr = ssGetSFcnParam(S, PARAM_v_max);
	const mxArray *a_max_ptr = ssGetSFcnParam(S, PARAM_a_max);
	const mxArray *j_max_ptr = ssGetSFcnParam(S, PARAM_j_max);
    if( mxGetNumberOfElements(v_max_ptr)!=NR_OF_TRAJECTORIES ){
		ssSetErrorStatus(S,"Nr of elements for parameter v_max doesn't match !!!");
		return;
	}
	if( mxGetNumberOfElements(a_max_ptr)!=NR_OF_TRAJECTORIES ){
		ssSetErrorStatus(S,"Nr of elements for parameter a_max doesn't match !!!");
		return;
	}
	if( mxGetNumberOfElements(j_max_ptr)!=NR_OF_TRAJECTORIES ){
		ssSetErrorStatus(S,"Nr of elements for parameter j_max doesn't match !!!");
		return;
	}
	
	// initialize pointer work variables
	ssGetPWork(S)[WORK_PTR_S_NODE] = (void *) new std::vector<std::vector<double> >(NR_OF_TRAJECTORIES);
	ssGetPWork(S)[WORK_PTR_TRAJECTORIES_OBJECT] = NULL;

	// initialize integer work variables
	ssGetIWork(S)[WORK_INT_TRAJECTROY_IS_ENABLED] = 0;
	ssGetIWork(S)[WORK_INT_RESET_TRAJECTROY_MODE] = 0;
	ssGetIWork(S)[WORK_INT_STOP_TRAJECTORY] = 0;

	// initialize float work variables
	ssGetRWork(S)[WORK_FLOAT_TRAJECTORY_TIME ] = 0.0;
	ssGetRWork(S)[WORK_FLOAT_TRAJECTORY_TIME_FILTERED ] = 0.0;
	
	real_T *Ts = (real_T*) mxGetPr(ssGetSFcnParam(S, PARAM_TS));
	ssGetRWork(S)[WORK_FLOAT_FILTER_COEFF_A1 ] = (2.0-Ts[0]*CUT_OFF_FREQUENCY_LOW_PASS)/(2.0+Ts[0]*CUT_OFF_FREQUENCY_LOW_PASS);
	ssGetRWork(S)[WORK_FLOAT_FILTER_COEFF_B0 ] = Ts[0]*CUT_OFF_FREQUENCY_LOW_PASS/(2.0+Ts[0]*CUT_OFF_FREQUENCY_LOW_PASS);
	ssGetRWork(S)[WORK_FLOAT_FILTER_COEFF_B1 ] = Ts[0]*CUT_OFF_FREQUENCY_LOW_PASS/(2.0+Ts[0]*CUT_OFF_FREQUENCY_LOW_PASS); 
    
	// initialize outputs
	real_T *t_E = ssGetOutputPortRealSignal(S, OUTPUT_t_E);
	t_E[0] = -1.0;

}// mdl start
#endif

std::vector<double> double_mxarray_to_vector( const mxArray *array){
	std::vector<double> vector( mxGetNumberOfElements(array));
	real_T *value = (real_T*) mxGetPr( array);
	
	for( unsigned int element_ii=0; element_ii<vector.size(); element_ii++){
		vector[element_ii]=value[element_ii];
	}
	return vector;
}// double mxarray to vector

/* calc outputs ---------------------------------------- */
static void mdlOutputs(SimStruct *S,int_T tid)
{
  // get outputs params and inputs 
  real_T *q = ssGetOutputPortRealSignal(S, OUTPUT_q);
  real_T *time_left = ssGetOutputPortRealSignal(S, OUTPUT_time_left);
  real_T *t_E = ssGetOutputPortRealSignal(S, OUTPUT_t_E);
  
  const mxArray *v_max = ssGetSFcnParam(S, PARAM_v_max);
  const mxArray *a_max = ssGetSFcnParam(S, PARAM_a_max);
  const mxArray *j_max = ssGetSFcnParam(S, PARAM_j_max);
  real_T *Ts = (real_T*) mxGetPr(ssGetSFcnParam(S, PARAM_TS));
          
  InputRealPtrsType qi_ptrptr = ssGetInputPortRealSignalPtrs(S, INPUT_qi);
  InputRealPtrsType reset_trajectory_ptrptr = ssGetInputPortRealSignalPtrs(S, INPUT_reset_trajectory);
  InputRealPtrsType add_point_to_trajectory_ptrptr = ssGetInputPortRealSignalPtrs(S, INPUT_add_point_to_trajectory);
  InputRealPtrsType start_trajectory_ptrptr = ssGetInputPortRealSignalPtrs(S, INPUT_start_trajectory);
  InputRealPtrsType stop_trajectory_ptrptr = ssGetInputPortRealSignalPtrs(S, INPUT_stop_trajectory);

  // get work vectors
  std::vector<std::vector<double> > *s_node = (std::vector<std::vector<double> > *) ssGetPWork(S)[WORK_PTR_S_NODE];
  
  // switch operation
  if( fabs(*reset_trajectory_ptrptr[0]-1.0)<1e-3 ){
	// operation: reset trajectory
	switch( ssGetIWork(S)[WORK_INT_RESET_TRAJECTROY_MODE] ){
	case 0:
		printf("reset trjectory - start\n");
		// initialize trajectoy reset
		ssGetIWork(S)[WORK_INT_RESET_TRAJECTROY_MODE]=1;
		ssGetRWork(S)[WORK_FLOAT_TRAJECTORY_TIME] = 0.0;
		ssGetRWork(S)[WORK_FLOAT_TRAJECTORY_TIME_FILTERED] = 0.0;
		ssGetIWork(S)[WORK_INT_TRAJECTROY_IS_ENABLED] = 0;
		ssGetIWork(S)[WORK_INT_STOP_TRAJECTORY] = 0;

		// remove old set points
		printf("remove old set points1\n");
		delete (std::vector<std::vector<double> > *) ssGetPWork(S)[WORK_PTR_S_NODE];
		printf("remove old set points2\n");
		s_node = new std::vector<std::vector<double> >(NR_OF_TRAJECTORIES);
		printf("remove old set points3\n");
		ssGetPWork(S)[WORK_PTR_S_NODE] = (void *)s_node;

		// remove old trajectory
		if( ssGetPWork(S)[WORK_PTR_TRAJECTORIES_OBJECT]!=NULL ){
			delete (cubic_trajectories *) ssGetPWork(S)[WORK_PTR_TRAJECTORIES_OBJECT];
			ssGetPWork(S)[WORK_PTR_TRAJECTORIES_OBJECT]=NULL;
		}

		// remove old t_E
		t_E[0] = -1.0;
		break;

	case 1:
		// finish trajectory reset
		ssGetIWork(S)[WORK_INT_RESET_TRAJECTROY_MODE]=0;

		if( s_node[0][0].size()==0 ){
			printf("Error - trajectory not initialized - no intermediate points specified\n");
		}else{
			printf("construct trajectory\n");
			cubic_trajectories *temp_trajectories = new cubic_trajectories( s_node[0], double_mxarray_to_vector(v_max), double_mxarray_to_vector(a_max), double_mxarray_to_vector(j_max) );
		
			printf("initialize trajectory ... ");
			temp_trajectories->initialize_trajectories();
		
			if( temp_trajectories->is_valid_initialized() ){
				printf("OK\n");
				ssGetPWork(S)[WORK_PTR_TRAJECTORIES_OBJECT] = (void*) temp_trajectories;
				t_E[0] = temp_trajectories->total_length();
			}else{
				delete temp_trajectories;
				printf("FAILED\n");
			}
		}
		printf("reset trjectory - end\n");
		break;
	}
  }
  
  if( fabs(*add_point_to_trajectory_ptrptr[0]-1.0)<1e-3 ){
	  // operation: add point to trajectory
	  if( ssGetIWork(S)[WORK_INT_RESET_TRAJECTROY_MODE]==1 ){
		printf("set point in trajectory\n");
		// if this is the first trajectory point, set output
		if( s_node[0][0].empty() ){
			for( unsigned int trajectory_ii=0; trajectory_ii<NR_OF_TRAJECTORIES; trajectory_ii++ ){
				q[trajectory_ii] = *qi_ptrptr[trajectory_ii];
			}
	    }
	  
		 // set point in trajectory
		std::vector<double> add_s_node( NR_OF_TRAJECTORIES);
		printf("add q = [");
		for( unsigned int trajectory_ii=0; trajectory_ii<NR_OF_TRAJECTORIES; trajectory_ii++ ){
			printf("%f, ",*qi_ptrptr[trajectory_ii]);
			s_node[0][trajectory_ii].push_back( *qi_ptrptr[trajectory_ii]);
		}// for all trajectories
		printf("]\n");
	 }else{
		printf("Error - could not set point in trajectory - reset mode not activated\n");
	 }
  }
  
  if( fabs(*start_trajectory_ptrptr[0]-1.0)<1e-3 ){
	  if( ssGetIWork(S)[WORK_INT_RESET_TRAJECTROY_MODE]==1 ){
		  printf("Error - could not start trajectory - reset mode is activate\n");
	  }else{
		  printf("start trajectory\n");
		  ssGetIWork(S)[WORK_INT_TRAJECTROY_IS_ENABLED] = 1;
	  }
  }

  if( fabs(*stop_trajectory_ptrptr[0]-1.0)<1e-3 ){
	  printf("stop trajectory\n");
	  ssGetIWork(S)[WORK_INT_TRAJECTROY_IS_ENABLED] = 0;
	  ssGetIWork(S)[WORK_INT_STOP_TRAJECTORY] = 1;

	  ssGetRWork(S)[WORK_FLOAT_TRAJECTORY_TIME] = ssGetRWork(S)[WORK_FLOAT_TRAJECTORY_TIME_FILTERED] + 1.0/CUT_OFF_FREQUENCY_LOW_PASS;
  }

  if( ssGetPWork(S)[WORK_PTR_TRAJECTORIES_OBJECT]!=NULL ){
	  // calc trajectory
	  cubic_trajectories *temp_trajectories = (cubic_trajectories *) ssGetPWork(S)[WORK_PTR_TRAJECTORIES_OBJECT];
	  std::vector<double> position(NR_OF_TRAJECTORIES);
	  temp_trajectories->evaluate_trajectories( ssGetRWork(S)[WORK_FLOAT_TRAJECTORY_TIME_FILTERED ], position);

	  for( unsigned int position_ii=0; position_ii<position.size(); position_ii++ ){
		  q[position_ii] = position[position_ii];
	  }

	  // update time
	  double u_km1 = ssGetRWork(S)[WORK_FLOAT_TRAJECTORY_TIME]; // u[k-1]
	  double u_k=u_km1;
	  if( ssGetIWork(S)[WORK_INT_TRAJECTROY_IS_ENABLED]==1 ){
		u_k += Ts[0];
	  }
		  
	  // filter for soft start / stop
	  double y_km1 = ssGetRWork(S)[WORK_FLOAT_TRAJECTORY_TIME_FILTERED];
	  double y_k=y_km1;

	  if( (ssGetIWork(S)[WORK_INT_STOP_TRAJECTORY]==0) && (ssGetIWork(S)[WORK_INT_TRAJECTROY_IS_ENABLED]==1) ){
		  y_k = y_km1+Ts[0];
	  }else if( ssGetIWork(S)[WORK_INT_STOP_TRAJECTORY]==1 ){
		  y_k = ssGetRWork(S)[WORK_FLOAT_FILTER_COEFF_A1 ]*y_km1 + ssGetRWork(S)[WORK_FLOAT_FILTER_COEFF_B0 ]*u_k + ssGetRWork(S)[WORK_FLOAT_FILTER_COEFF_B1 ]*u_km1;  
	  }
		
	  ssGetRWork(S)[WORK_FLOAT_TRAJECTORY_TIME] = u_k;
	  ssGetRWork(S)[WORK_FLOAT_TRAJECTORY_TIME_FILTERED] = y_k;

  }

  time_left[0] = t_E[0] - ssGetRWork(S)[WORK_FLOAT_TRAJECTORY_TIME_FILTERED];
  
}// calc outputs

/* terminate ---------------------------------------- */
static void mdlTerminate(SimStruct *S)
{
	// free space
	if( ssGetPWork(S)[WORK_PTR_TRAJECTORIES_OBJECT]!=NULL ){
		delete (cubic_trajectories *) ssGetPWork(S)[WORK_PTR_TRAJECTORIES_OBJECT];
	}
	delete (std::vector<std::vector<double> > *) ssGetPWork(S)[WORK_PTR_S_NODE];
	
}// terminate

#ifdef MATLAB_MEX_FILE   /* Compile as a MEX-file? */
  #include "simulink.c"  /* MEX-file interface mechanism */
#else
  #include "cg_sfun.h"   /* Code generation registration */
#endif
