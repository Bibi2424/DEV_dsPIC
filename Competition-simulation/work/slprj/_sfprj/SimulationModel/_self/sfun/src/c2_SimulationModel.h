#ifndef __c2_SimulationModel_h__
#define __c2_SimulationModel_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc2_SimulationModelInstanceStruct
#define typedef_SFc2_SimulationModelInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c2_sfEvent;
  uint8_T c2_tp_Standby;
  uint8_T c2_tp_Running;
  uint8_T c2_tp_Next_Target;
  uint8_T c2_tp_Apply_camera_correction;
  uint8_T c2_tp_Wait_for_scoring;
  uint8_T c2_tp_Calc_next_robot_move;
  uint8_T c2_tp_All_targets_done;
  uint8_T c2_tp_Finished;
  boolean_T c2_isStable;
  uint8_T c2_is_active_c2_SimulationModel;
  uint8_T c2_is_c2_SimulationModel;
  uint8_T c2_is_Running;
  uint8_T c2_TargetIndex;
  uint8_T c2_TargetFound;
  real32_T c2_AxleLength;
  real32_T c2_WheelRadius;
  real32_T c2_EncRes;
  int32_T c2_encoder_right_prev_target;
  uint8_T c2_ind;
  real32_T c2_dR;
  real32_T c2_dL;
  real32_T c2_CurrentDistance;
  real32_T c2_CurrentBearing;
  int16_T c2_dist_remaining;
  int32_T c2_encoder_left_prev_target;
  real_T c2_startPos[2];
  real32_T c2_prev_target[2];
  real32_T c2_prev_bearing;
  real32_T c2_BearingToTarget;
  real32_T c2_DistanceToTarget;
  real_T c2_pcam[2];
  real_T c2_lcam[2];
  uint8_T c2_temporalCounter_i1;
  uint8_T c2_doSetSimStateSideEffects;
  const mxArray *c2_setSimStateSideEffectsInfo;
} SFc2_SimulationModelInstanceStruct;

#endif                                 /*typedef_SFc2_SimulationModelInstanceStruct*/

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c2_SimulationModel_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c2_SimulationModel_get_check_sum(mxArray *plhs[]);
extern void c2_SimulationModel_method_dispatcher(SimStruct *S, int_T method,
  void *data);

#endif
