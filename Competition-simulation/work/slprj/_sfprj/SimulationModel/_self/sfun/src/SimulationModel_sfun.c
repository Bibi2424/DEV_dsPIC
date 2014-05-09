/* Include files */

#include "SimulationModel_sfun.h"
#include "SimulationModel_sfun_debug_macros.h"
#include "c2_SimulationModel.h"
#include "c5_SimulationModel.h"
#include "c6_SimulationModel.h"
#include "c7_SimulationModel.h"
#include "c8_SimulationModel.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */
uint32_T _SimulationModelMachineNumber_;

/* Function Declarations */

/* Function Definitions */
void SimulationModel_initializer(void)
{
}

void SimulationModel_terminator(void)
{
}

/* SFunction Glue Code */
unsigned int sf_SimulationModel_method_dispatcher(SimStruct *simstructPtr,
  unsigned int chartFileNumber, const char* specsCksum, int_T method, void *data)
{
  if (chartFileNumber==2) {
    c2_SimulationModel_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==5) {
    c5_SimulationModel_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==6) {
    c6_SimulationModel_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==7) {
    c7_SimulationModel_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==8) {
    c8_SimulationModel_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  return 0;
}

unsigned int sf_SimulationModel_process_check_sum_call( int nlhs, mxArray *
  plhs[], int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[20];
  if (nrhs<1 || !mxIsChar(prhs[0]) )
    return 0;

  /* Possible call to get the checksum */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"sf_get_check_sum"))
    return 0;
  plhs[0] = mxCreateDoubleMatrix( 1,4,mxREAL);
  if (nrhs>1 && mxIsChar(prhs[1])) {
    mxGetString(prhs[1], commandName,sizeof(commandName)/sizeof(char));
    commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
    if (!strcmp(commandName,"machine")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2726871752U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2692177518U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(884891079U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(998455194U);
    } else if (!strcmp(commandName,"exportedFcn")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0U);
    } else if (!strcmp(commandName,"makefile")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2795326625U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3614828100U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2449601383U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(830715839U);
    } else if (nrhs==3 && !strcmp(commandName,"chart")) {
      unsigned int chartFileNumber;
      chartFileNumber = (unsigned int)mxGetScalar(prhs[2]);
      switch (chartFileNumber) {
       case 2:
        {
          extern void sf_c2_SimulationModel_get_check_sum(mxArray *plhs[]);
          sf_c2_SimulationModel_get_check_sum(plhs);
          break;
        }

       case 5:
        {
          extern void sf_c5_SimulationModel_get_check_sum(mxArray *plhs[]);
          sf_c5_SimulationModel_get_check_sum(plhs);
          break;
        }

       case 6:
        {
          extern void sf_c6_SimulationModel_get_check_sum(mxArray *plhs[]);
          sf_c6_SimulationModel_get_check_sum(plhs);
          break;
        }

       case 7:
        {
          extern void sf_c7_SimulationModel_get_check_sum(mxArray *plhs[]);
          sf_c7_SimulationModel_get_check_sum(plhs);
          break;
        }

       case 8:
        {
          extern void sf_c8_SimulationModel_get_check_sum(mxArray *plhs[]);
          sf_c8_SimulationModel_get_check_sum(plhs);
          break;
        }

       default:
        ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0.0);
      }
    } else if (!strcmp(commandName,"target")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3031367619U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(4001028638U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3978939492U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(838979348U);
    } else {
      return 0;
    }
  } else {
    ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1265044430U);
    ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(870819430U);
    ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3035902789U);
    ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2506955358U);
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_SimulationModel_autoinheritance_info( int nlhs, mxArray * plhs[],
  int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[32];
  char aiChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]) )
    return 0;

  /* Possible call to get the autoinheritance_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_autoinheritance_info"))
    return 0;
  mxGetString(prhs[2], aiChksum,sizeof(aiChksum)/sizeof(char));
  aiChksum[(sizeof(aiChksum)/sizeof(char)-1)] = '\0';

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 2:
      {
        if (strcmp(aiChksum, "71uuUKZ6WmSGXBH0fUl7AD") == 0) {
          extern mxArray *sf_c2_SimulationModel_get_autoinheritance_info(void);
          plhs[0] = sf_c2_SimulationModel_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 5:
      {
        if (strcmp(aiChksum, "cOM7uErdwUZ6euP9O7hMVH") == 0) {
          extern mxArray *sf_c5_SimulationModel_get_autoinheritance_info(void);
          plhs[0] = sf_c5_SimulationModel_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 6:
      {
        if (strcmp(aiChksum, "e9eNiyjX7FdpIfaAp4Uz5E") == 0) {
          extern mxArray *sf_c6_SimulationModel_get_autoinheritance_info(void);
          plhs[0] = sf_c6_SimulationModel_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 7:
      {
        if (strcmp(aiChksum, "4mVVsT5d2mOLKYXUfuIU7F") == 0) {
          extern mxArray *sf_c7_SimulationModel_get_autoinheritance_info(void);
          plhs[0] = sf_c7_SimulationModel_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 8:
      {
        if (strcmp(aiChksum, "iLYCs8bm7r2EOS18m9NOeE") == 0) {
          extern mxArray *sf_c8_SimulationModel_get_autoinheritance_info(void);
          plhs[0] = sf_c8_SimulationModel_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_SimulationModel_get_eml_resolved_functions_info( int nlhs,
  mxArray * plhs[], int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[64];
  if (nrhs<2 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the get_eml_resolved_functions_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_eml_resolved_functions_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 2:
      {
        extern const mxArray
          *sf_c2_SimulationModel_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c2_SimulationModel_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 5:
      {
        extern const mxArray
          *sf_c5_SimulationModel_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c5_SimulationModel_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 6:
      {
        extern const mxArray
          *sf_c6_SimulationModel_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c6_SimulationModel_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 7:
      {
        extern const mxArray
          *sf_c7_SimulationModel_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c7_SimulationModel_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 8:
      {
        extern const mxArray
          *sf_c8_SimulationModel_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c8_SimulationModel_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_SimulationModel_third_party_uses_info( int nlhs, mxArray * plhs[],
  int nrhs, const mxArray * prhs[] )
{
  char commandName[64];
  char tpChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the third_party_uses_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  mxGetString(prhs[2], tpChksum,sizeof(tpChksum)/sizeof(char));
  tpChksum[(sizeof(tpChksum)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_third_party_uses_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 2:
      {
        if (strcmp(tpChksum, "VxQPghVCdUExcSIgqE9kyC") == 0) {
          extern mxArray *sf_c2_SimulationModel_third_party_uses_info(void);
          plhs[0] = sf_c2_SimulationModel_third_party_uses_info();
          break;
        }
      }

     case 5:
      {
        if (strcmp(tpChksum, "eRakhMnlKT9xJviVBhRoNH") == 0) {
          extern mxArray *sf_c5_SimulationModel_third_party_uses_info(void);
          plhs[0] = sf_c5_SimulationModel_third_party_uses_info();
          break;
        }
      }

     case 6:
      {
        if (strcmp(tpChksum, "uXor2wIQrt8ncPgIOdiUrB") == 0) {
          extern mxArray *sf_c6_SimulationModel_third_party_uses_info(void);
          plhs[0] = sf_c6_SimulationModel_third_party_uses_info();
          break;
        }
      }

     case 7:
      {
        if (strcmp(tpChksum, "1ZtBlEZkBziQt1QzED0QVG") == 0) {
          extern mxArray *sf_c7_SimulationModel_third_party_uses_info(void);
          plhs[0] = sf_c7_SimulationModel_third_party_uses_info();
          break;
        }
      }

     case 8:
      {
        if (strcmp(tpChksum, "Zs3TLFDt54K99IhInIYMoE") == 0) {
          extern mxArray *sf_c8_SimulationModel_third_party_uses_info(void);
          plhs[0] = sf_c8_SimulationModel_third_party_uses_info();
          break;
        }
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;
}

unsigned int sf_SimulationModel_updateBuildInfo_args_info( int nlhs, mxArray *
  plhs[], int nrhs, const mxArray * prhs[] )
{
  char commandName[64];
  char tpChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the updateBuildInfo_args_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  mxGetString(prhs[2], tpChksum,sizeof(tpChksum)/sizeof(char));
  tpChksum[(sizeof(tpChksum)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_updateBuildInfo_args_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 2:
      {
        if (strcmp(tpChksum, "VxQPghVCdUExcSIgqE9kyC") == 0) {
          extern mxArray *sf_c2_SimulationModel_updateBuildInfo_args_info(void);
          plhs[0] = sf_c2_SimulationModel_updateBuildInfo_args_info();
          break;
        }
      }

     case 5:
      {
        if (strcmp(tpChksum, "eRakhMnlKT9xJviVBhRoNH") == 0) {
          extern mxArray *sf_c5_SimulationModel_updateBuildInfo_args_info(void);
          plhs[0] = sf_c5_SimulationModel_updateBuildInfo_args_info();
          break;
        }
      }

     case 6:
      {
        if (strcmp(tpChksum, "uXor2wIQrt8ncPgIOdiUrB") == 0) {
          extern mxArray *sf_c6_SimulationModel_updateBuildInfo_args_info(void);
          plhs[0] = sf_c6_SimulationModel_updateBuildInfo_args_info();
          break;
        }
      }

     case 7:
      {
        if (strcmp(tpChksum, "1ZtBlEZkBziQt1QzED0QVG") == 0) {
          extern mxArray *sf_c7_SimulationModel_updateBuildInfo_args_info(void);
          plhs[0] = sf_c7_SimulationModel_updateBuildInfo_args_info();
          break;
        }
      }

     case 8:
      {
        if (strcmp(tpChksum, "Zs3TLFDt54K99IhInIYMoE") == 0) {
          extern mxArray *sf_c8_SimulationModel_updateBuildInfo_args_info(void);
          plhs[0] = sf_c8_SimulationModel_updateBuildInfo_args_info();
          break;
        }
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;
}

void SimulationModel_debug_initialize(struct SfDebugInstanceStruct*
  debugInstance)
{
  _SimulationModelMachineNumber_ = sf_debug_initialize_machine(debugInstance,
    "SimulationModel","sfun",0,5,0,0,0);
  sf_debug_set_machine_event_thresholds(debugInstance,
    _SimulationModelMachineNumber_,0,0);
  sf_debug_set_machine_data_thresholds(debugInstance,
    _SimulationModelMachineNumber_,0);
}

void SimulationModel_register_exported_symbols(SimStruct* S)
{
}

static mxArray* sRtwOptimizationInfoStruct= NULL;
mxArray* load_SimulationModel_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct==NULL) {
    sRtwOptimizationInfoStruct = sf_load_rtw_optimization_info("SimulationModel",
      "SimulationModel");
    mexMakeArrayPersistent(sRtwOptimizationInfoStruct);
  }

  return(sRtwOptimizationInfoStruct);
}

void unload_SimulationModel_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct!=NULL) {
    mxDestroyArray(sRtwOptimizationInfoStruct);
    sRtwOptimizationInfoStruct = NULL;
  }
}
