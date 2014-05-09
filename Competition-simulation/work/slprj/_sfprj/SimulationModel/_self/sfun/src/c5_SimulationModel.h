#ifndef __c5_SimulationModel_h__
#define __c5_SimulationModel_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc5_SimulationModelInstanceStruct
#define typedef_SFc5_SimulationModelInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c5_sfEvent;
  uint8_T c5_tp_Standby;
  uint8_T c5_tp_Running;
  uint8_T c5_tp_NearLocation;
  uint8_T c5_tp_Roaming;
  uint8_T c5_tp_Finished;
  boolean_T c5_isStable;
  uint8_T c5_is_active_c5_SimulationModel;
  uint8_T c5_is_c5_SimulationModel;
  uint8_T c5_is_Running;
  real_T c5_startPos[2];
  real_T c5_allowedRunTime;
  real_T c5_statTimeReq;
  real_T c5_startingRadius;
  real_T c5_targetBoundaryRadius;
  boolean_T c5_nearLocation;
  real_T c5_numTargetsFound;
  real_T c5_numTargets;
  boolean_T c5_currentTarget[8];
  real_T c5_snapshotTime;
  real_T local_t;
  uint8_T c5_doSetSimStateSideEffects;
  const mxArray *c5_setSimStateSideEffectsInfo;
} SFc5_SimulationModelInstanceStruct;

#endif                                 /*typedef_SFc5_SimulationModelInstanceStruct*/

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c5_SimulationModel_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c5_SimulationModel_get_check_sum(mxArray *plhs[]);
extern void c5_SimulationModel_method_dispatcher(SimStruct *S, int_T method,
  void *data);

#endif
