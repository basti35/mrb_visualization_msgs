/********************************************************************
 * 
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012, Max-Planck-Gesellschaft
 * Copyright (c) 2012-2015, Inria
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ********************************************************************/


/*
 * You must specify the S_FUNCTION_NAME as the name of your S-function
 * (i.e. replace sfuntmpl_basic with the name of your S-function).
 */

#define S_FUNCTION_NAME  rmsg_pub_MarkerArray
#define S_FUNCTION_LEVEL 2

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)


/*
 * Need to include simstruc.h for the definition of the SimStruct and
 * its associated macro definitions.
 */
#include "simstruc.h"

#pragma push_macro("RT")
#undef RT

#include <ros/ros.h>

// Generic Publisher
#include <matlab_ros_bridge/GenericPublisher.hpp>

// Message
#include <visualization_msgs/MarkerArray.h>

#pragma pop_macro("RT")

#include <matlab_ros_bridge/RosMatlabBrigdeDefines.hpp>

#define MDL_CHECK_PARAMETERS
#if defined(MDL_CHECK_PARAMETERS) && defined(MATLAB_MEX_FILE)
  /* Function: mdlCheckParameters =============================================
   * Abstract:
   *    Validate our parameters to verify:
   *     o The numerator must be of a lower order than the denominator.
   *     o The sample time must be a real positive nonzero value.
   */
  static void mdlCheckParameters(SimStruct *S)
  {
	//  SFUNPRINTF("Calling mdlCheckParameters");
    
    // Tsim
	if (mxIsEmpty( ssGetSFcnParam(S,0)) ||
			mxIsSparse( ssGetSFcnParam(S,0)) ||
			mxIsComplex( ssGetSFcnParam(S,0)) ||
			mxIsLogical( ssGetSFcnParam(S,0)) ||
			!mxIsNumeric( ssGetSFcnParam(S,0)) ||
			!mxIsDouble( ssGetSFcnParam(S,0)) ||
			mxGetNumberOfElements(ssGetSFcnParam(S,0)) != 1) {
		ssSetErrorStatus(S,"Simulation time must be a single double Value");
		return;
	}

	// Prefix Topic
	if (!mxIsChar( ssGetSFcnParam(S,1)) ) {
		ssSetErrorStatus(S,"Prefix value must be char array (string)");
		return;
	}

    // Robot Array
    if (mxIsEmpty( ssGetSFcnParam(S,2)) ||
            mxIsSparse( ssGetSFcnParam(S,2)) ||
            mxIsComplex( ssGetSFcnParam(S,2)) ||
            mxIsLogical( ssGetSFcnParam(S,2)) ||
            !mxIsChar( ssGetSFcnParam(S,2)) ) {
        ssSetErrorStatus(S,"Robot Vector must be a char vector of robot ids");
        return;
    }
    
    // Postfix Topic
	if (!mxIsChar( ssGetSFcnParam(S,3)) ) {
		ssSetErrorStatus(S,"Postfix value must be char array (string)");
		return;
	}

    // N. of Marker
    if (mxIsEmpty( ssGetSFcnParam(S,4)) ||
          mxIsSparse( ssGetSFcnParam(S,4)) ||
          mxIsComplex( ssGetSFcnParam(S,4)) ||
          mxIsLogical( ssGetSFcnParam(S,4)) ||
          mxIsInt32( ssGetSFcnParam(S,4)) )
    {
        ssSetErrorStatus(S,"N. of markers must be a Uint32");
        return;
    }

    // Frame_id
  	if (!mxIsChar( ssGetSFcnParam(S,5)) )
    {
		ssSetErrorStatus(S,"Frame_id value must be char array (string)");
		return;
  	}
  }
#endif /* MDL_CHECK_PARAMETERS */



/* Error handling
 * --------------
 *
 * You should use the following technique to report errors encountered within
 * an S-function:
 *
 *       ssSetErrorStatus(S,"Error encountered due to ...");
 *       return;
 *
 * Note that the 2nd argument to ssSetErrorStatus must be persistent memory.
 * It cannot be a local variable. For example the following will cause
 * unpredictable errors:
 *
 *      mdlOutputs()
 *      {
 *         char msg[256];         {ILLEGAL: to fix use "static char msg[256];"}
 *         sprintf(msg,"Error due to %s", string);
 *         ssSetErrorStatus(S,msg);
 *         return;
 *      }
 *
 * See matlabroot/simulink/src/sfuntmpl_doc.c for more details.
 */

/*====================*
 * S-function methods *
 *====================*/


//double Tsim;

/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *    The sizes information is used by Simulink to determine the S-function
 *    block's characteristics (number of inputs, outputs, states, etc.).
 */

static void mdlInitializeSizes(SimStruct *S)
{
    /* See sfuntmpl_doc.c for more details on the macros below */

    ssSetNumSFcnParams(S, 6);  /* Number of expected parameters */
#if defined(MATLAB_MEX_FILE)
    if (ssGetNumSFcnParams(S) == ssGetSFcnParamsCount(S)) {
        mdlCheckParameters(S);
        if (ssGetErrorStatus(S) != NULL) {
            return;
        }
    } else {
        return; /* Parameter mismatch will be reported by Simulink. */
    }
#endif

    const int_T nRobots = mxGetNumberOfElements(ssGetSFcnParam(S,2));

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    if (!ssSetNumInputPorts(S, 9)) return;

    const int_T nMarkers = mxGetScalar(ssGetSFcnParam(S,4));

    ssSetInputPortMatrixDimensions(S, 0, nRobots,  nMarkers); // id
    ssSetInputPortMatrixDimensions(S, 1, nRobots,  nMarkers); // type
    ssSetInputPortMatrixDimensions(S, 2, nRobots,  nMarkers); // action
    ssSetInputPortMatrixDimensions(S, 3, nRobots,  3*nMarkers); // position
    ssSetInputPortMatrixDimensions(S, 4, nRobots,  4*nMarkers); // orientation
    ssSetInputPortMatrixDimensions(S, 5, nRobots,  3*nMarkers); // scale
    ssSetInputPortMatrixDimensions(S, 6, nRobots,  4*nMarkers); // color
    ssSetInputPortMatrixDimensions(S, 7, nRobots,  nMarkers); // lifetime
    ssSetInputPortMatrixDimensions(S, 8, nRobots,  nMarkers); // locked



	for (int_T i = 0; i < ssGetNumInputPorts(S); ++i) {
		/*direct input signal access*/
    	ssSetInputPortRequiredContiguous(S, i, true); 
		/*
		 * Set direct feedthrough flag (1=yes, 0=no).
		 * A port has direct feedthrough if the input is used in either
		 * the mdlOutputs or mdlGetTimeOfNextVarHit functions.
		 * See matlabroot/simulink/src/sfuntmpl_directfeed.txt.
		 */
		ssSetInputPortDirectFeedThrough(S, i, 1);
	}

    if (!ssSetNumOutputPorts(S, 0)) return;

    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 2); // nRobots, n_markers
    ssSetNumPWork(S, nRobots+1); // nRobots x GenericPub + frame_id
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    /* Specify the sim state compliance to be same as a built-in block */
    ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);

    ssSetOptions(S, 0);
}



/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    This function is used to specify the sample time(s) for your
 *    S-function. You must register the same number of sample times as
 *    specified in ssSetNumSampleTimes.
 */

static void mdlInitializeSampleTimes(SimStruct *S)
{
    real_T Tsim = mxGetScalar(ssGetSFcnParam(S, 0));
    ssSetSampleTime(S, 0, Tsim);                      //DISCRETE_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);

}



#define MDL_INITIALIZE_CONDITIONS   /* Change to #undef to remove function */
#if defined(MDL_INITIALIZE_CONDITIONS)
  /* Function: mdlInitializeConditions ========================================
   * Abstract:
   *    In this function, you should initialize the continuous and discrete
   *    states for your S-function block.  The initial states are placed
   *    in the state vector, ssGetContStates(S) or ssGetRealDiscStates(S).
   *    You can also perform any other initialization activities that your
   *    S-function may require. Note, this routine will be called at the
   *    start of simulation and if it is present in an enabled subsystem
   *    configured to reset states, it will be call when the enabled subsystem
   *    restarts execution to reset the states.
   */
  static void mdlInitializeConditions(SimStruct *S)
  {
  }
#endif /* MDL_INITIALIZE_CONDITIONS */

#define MDL_START  /* Change to #undef to remove function */
#if defined(MDL_START) 
  /* Function: mdlStart =======================================================
   * Abstract:
   *    This function is called once at start of model execution. If you
   *    have states that should be initialized once, this is the place
   *    to do it.
   */

static void mdlStart(SimStruct *S)
{   
    SFUNPRINTF("Creating Instance of %s.\n", TOSTRING(S_FUNCTION_NAME));
    // init ROS if not yet done.
    initROS(S);

    void** vecPWork = ssGetPWork(S);
    // save nRobots in IWorkVector
    int_T nRobots = mxGetNumberOfElements(ssGetSFcnParam(S,2));
    *ssGetIWork(S) = nRobots;
    ssGetIWork(S)[1] = mxGetScalar(ssGetSFcnParam(S,4));

    ros::NodeHandle nodeHandle(ros::this_node::getName());

    // get Topic Strings
    size_t prefix_buflen = mxGetN((ssGetSFcnParam(S, 1)))*sizeof(mxChar)+1;
    size_t postfix_buflen = mxGetN((ssGetSFcnParam(S, 3)))*sizeof(mxChar)+1;
    char* prefix_topic = (char*)mxMalloc(prefix_buflen);
    char* postfix_topic = (char*)mxMalloc(postfix_buflen);
    mxGetString((ssGetSFcnParam(S, 1)), prefix_topic, prefix_buflen);
    mxGetString((ssGetSFcnParam(S, 3)), postfix_topic, postfix_buflen);

    //SFUNPRINTF("The string being passed as a Paramater is - %s\n ", topic);
    std::stringstream sstream;
    mxChar* robotIDs = (mxChar*)mxGetData(ssGetSFcnParam(S, 2));
    for (unsigned int i = 0; i < nRobots; ++i) {
      sstream.str(std::string());
      
      // build topicstring
      sstream << prefix_topic;
      sstream << (uint)robotIDs[i];
      sstream << postfix_topic;

      GenericPublisher<visualization_msgs::MarkerArray>* pub
        = new GenericPublisher<visualization_msgs::MarkerArray>(nodeHandle, sstream.str(), 10);
      vecPWork[i] = pub;
    }

  //get frame_id
  size_t buflen = mxGetN((ssGetSFcnParam(S, 5)))*sizeof(mxChar)+1;
  char* frame_id = (char*)mxMalloc(buflen);
  mxGetString((ssGetSFcnParam(S, 5)), frame_id, buflen);
  vecPWork[nRobots] = new std::string(frame_id);


    // free char array
    mxFree(prefix_topic);
    mxFree(postfix_topic);
  }
#endif /*  MDL_START */



/* Function: mdlOutputs =======================================================
 * Abstract:
 *    In this function, you compute the outputs of your S-function
 *    block.
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    // get Objects
    void** vecPWork = ssGetPWork(S);
    int_T nRobots = *ssGetIWork(S);
    std::string *frame_id = (std::string *) vecPWork[nRobots]; //Being 0-indexed, this is the last element of vecPWork: frame_id
    const int_T nMarkers = ssGetIWork(S)[1];

    // get Pointers
    // accessing inputs
    const real_T *id          = (const real_T*) ssGetInputPortSignal(S,0);
    const real_T *type        = (const real_T*) ssGetInputPortSignal(S,1);
    const real_T *action      = (const real_T*) ssGetInputPortSignal(S,2);
    const real_T *position    = (const real_T*) ssGetInputPortSignal(S,3);
    const real_T *orientation = (const real_T*) ssGetInputPortSignal(S,4);
    const real_T *scale       = (const real_T*) ssGetInputPortSignal(S,5);
    const real_T *color       = (const real_T*) ssGetInputPortSignal(S,6);
    const real_T *lifetime    = (const real_T*) ssGetInputPortSignal(S,7);
    const real_T *locked      = (const real_T*) ssGetInputPortSignal(S,8);

    ros::Time now(ros::Time::now());

    for (unsigned int i = 0; i < nRobots; ++i)
    {
      visualization_msgs::MarkerArray msg;
      GenericPublisher<visualization_msgs::MarkerArray>* pub 
        = (GenericPublisher<visualization_msgs::MarkerArray>*)vecPWork[i];
      
      // define send Time.
      msg.markers.resize(nMarkers);
      for (unsigned int curr_mark = 0; curr_mark < nMarkers; ++curr_mark)
      {
        msg.markers[curr_mark].header.stamp = now;
        msg.markers[curr_mark].header.frame_id = *frame_id;
        msg.markers[curr_mark].id = id[curr_mark+ i * nMarkers];
        msg.markers[curr_mark].type = type[curr_mark+ i * nMarkers];
        msg.markers[curr_mark].action = action[curr_mark+ i * nMarkers];
        msg.markers[curr_mark].pose.position.x = position[3*curr_mark+ i * nMarkers + 0];
        msg.markers[curr_mark].pose.position.y = position[3*curr_mark+ i * nMarkers + 1];
        msg.markers[curr_mark].pose.position.z = position[3*curr_mark+ i * nMarkers + 2];
        msg.markers[curr_mark].pose.orientation.x = orientation[4*curr_mark+ i * nMarkers + 0];
        msg.markers[curr_mark].pose.orientation.y = orientation[4*curr_mark+ i * nMarkers + 1];
        msg.markers[curr_mark].pose.orientation.z = orientation[4*curr_mark+ i * nMarkers + 2];
        msg.markers[curr_mark].pose.orientation.w = orientation[4*curr_mark+ i * nMarkers + 3];
        msg.markers[curr_mark].scale.x = scale[3*curr_mark+ i * nMarkers + 0];
        msg.markers[curr_mark].scale.y = scale[3*curr_mark+ i * nMarkers + 1];
        msg.markers[curr_mark].scale.z = scale[3*curr_mark+ i * nMarkers + 2];
        msg.markers[curr_mark].color.r = color[4*curr_mark+ i * nMarkers + 0];
        msg.markers[curr_mark].color.g = color[4*curr_mark+ i * nMarkers + 1];
        msg.markers[curr_mark].color.b = color[4*curr_mark+ i * nMarkers + 2];
        msg.markers[curr_mark].color.a = color[4*curr_mark+ i * nMarkers + 3];
       
/*
        Waiting for https://github.com/ros/ros_comm/issues/148
        msg.markers[curr_mark].pose = geometry_msgs::Pose ( geometry_msgs::Point(position[3*curr_mark+0], position[3*curr_mark+1], position[3*curr_mark+2]),
                                                            geometry_msgs::Quaternion(orientation[4*curr_mark+0], orientation[4*curr_mark+1], orientation[4*curr_mark+2], orientation[4*curr_mark+3]));
        msg.markers[curr_mark].scale = geometry_msgs::Vector3 ( scale[3*curr_mark + 0], scale[3*curr_mark + 1], scale[3*curr_mark + 2]);
        msg.markers[curr_mark].color = std_msgs::ColorRGBA ( color[4*curr_mark + 0], color[4*curr_mark + 1], color[4*curr_mark + 2], color[4*curr_mark + 3]);
*/
        msg.markers[curr_mark].lifetime = ros::Duration(lifetime[curr_mark + i * nMarkers]);
        msg.markers[curr_mark].frame_locked = (locked[curr_mark + i * nMarkers] != false);
      }
      pub->publish(msg);
    }
}



#define MDL_UPDATE  /* Change to #undef to remove function */
#if defined(MDL_UPDATE)
  /* Function: mdlUpdate ======================================================
   * Abstract:
   *    This function is called once for every major integration time step.
   *    Discrete states are typically updated here, but this function is useful
   *    for performing any tasks that should only take place once per
   *    integration step.
   */
  static void mdlUpdate(SimStruct *S, int_T tid)
  {
  }
#endif /* MDL_UPDATE */



#define MDL_DERIVATIVES  /* Change to #undef to remove function */
#if defined(MDL_DERIVATIVES)
  /* Function: mdlDerivatives =================================================
   * Abstract:
   *    In this function, you compute the S-function block's derivatives.
   *    The derivatives are placed in the derivative vector, ssGetdX(S).
   */
  static void mdlDerivatives(SimStruct *S)
  {
  }
#endif /* MDL_DERIVATIVES */



/* Function: mdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was
 *    allocated in mdlStart, this is the place to free it.
 */
static void mdlTerminate(SimStruct *S)
{
    // get Objects
    void** vecPWork = ssGetPWork(S);

    int_T nRobots = *ssGetIWork(S);
    for (unsigned int i = 0; i < nRobots; ++i) {
        GenericPublisher<visualization_msgs::MarkerArray>* pub = (GenericPublisher<visualization_msgs::MarkerArray>*)vecPWork[i];
        // cleanup
        delete pub;
	}


    SFUNPRINTF("Terminating Instance of %s.\n", TOSTRING(S_FUNCTION_NAME));
}



/*=============================*
 * Required S-function trailer *
 *=============================*/

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
