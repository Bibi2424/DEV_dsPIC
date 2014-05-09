/* Include files */

#include <stddef.h>
#include "blas.h"
#include "SimulationModel_sfun.h"
#include "c2_SimulationModel.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "SimulationModel_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(sfGlobalDebugInstanceStruct,S);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)
#define c2_IN_NO_ACTIVE_CHILD          ((uint8_T)0U)
#define c2_IN_All_targets_done         ((uint8_T)1U)
#define c2_IN_Finished                 ((uint8_T)2U)
#define c2_IN_Running                  ((uint8_T)3U)
#define c2_IN_Standby                  ((uint8_T)4U)
#define c2_IN_Apply_camera_correction  ((uint8_T)1U)
#define c2_IN_Calc_next_robot_move     ((uint8_T)2U)
#define c2_IN_Next_Target              ((uint8_T)3U)
#define c2_IN_Wait_for_scoring         ((uint8_T)4U)

/* Variable Declarations */

/* Variable Definitions */
static real_T _sfTime_;
static const char * c2_debug_family_names[2] = { "nargin", "nargout" };

static const char * c2_b_debug_family_names[2] = { "nargin", "nargout" };

static const char * c2_c_debug_family_names[2] = { "nargin", "nargout" };

static const char * c2_d_debug_family_names[2] = { "nargin", "nargout" };

static const char * c2_e_debug_family_names[2] = { "nargin", "nargout" };

static const char * c2_f_debug_family_names[2] = { "nargin", "nargout" };

static const char * c2_g_debug_family_names[2] = { "nargin", "nargout" };

static const char * c2_h_debug_family_names[2] = { "nargin", "nargout" };

static const char * c2_i_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c2_j_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c2_k_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c2_l_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c2_m_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c2_n_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c2_o_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c2_p_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c2_q_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c2_r_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c2_s_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c2_t_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static boolean_T c2_dataWrittenToVector[19];

/* Function Declarations */
static void initialize_c2_SimulationModel(SFc2_SimulationModelInstanceStruct
  *chartInstance);
static void initialize_params_c2_SimulationModel
  (SFc2_SimulationModelInstanceStruct *chartInstance);
static void enable_c2_SimulationModel(SFc2_SimulationModelInstanceStruct
  *chartInstance);
static void disable_c2_SimulationModel(SFc2_SimulationModelInstanceStruct
  *chartInstance);
static void c2_update_debugger_state_c2_SimulationModel
  (SFc2_SimulationModelInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c2_SimulationModel
  (SFc2_SimulationModelInstanceStruct *chartInstance);
static void set_sim_state_c2_SimulationModel(SFc2_SimulationModelInstanceStruct *
  chartInstance, const mxArray *c2_st);
static void c2_set_sim_state_side_effects_c2_SimulationModel
  (SFc2_SimulationModelInstanceStruct *chartInstance);
static void finalize_c2_SimulationModel(SFc2_SimulationModelInstanceStruct
  *chartInstance);
static void sf_gateway_c2_SimulationModel(SFc2_SimulationModelInstanceStruct
  *chartInstance);
static void c2_chartstep_c2_SimulationModel(SFc2_SimulationModelInstanceStruct
  *chartInstance);
static void initSimStructsc2_SimulationModel(SFc2_SimulationModelInstanceStruct *
  chartInstance);
static void c2_exit_internal_Running(SFc2_SimulationModelInstanceStruct
  *chartInstance);
static void c2_enter_atomic_Next_Target(SFc2_SimulationModelInstanceStruct
  *chartInstance);
static void c2_enter_atomic_Apply_camera_correction
  (SFc2_SimulationModelInstanceStruct *chartInstance);
static void c2_enter_atomic_Calc_next_robot_move
  (SFc2_SimulationModelInstanceStruct *chartInstance);
static void c2_Calc_next_robot_move(SFc2_SimulationModelInstanceStruct
  *chartInstance);
static real32_T c2_eml_xnrm2(SFc2_SimulationModelInstanceStruct *chartInstance,
  real32_T c2_x[2]);
static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber, uint32_T c2_instanceNumber);
static const mxArray *c2_sf_marshallOut(void *chartInstanceVoid, void *c2_inData);
static real_T c2_emlrt_marshallIn(SFc2_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static boolean_T c2_b_emlrt_marshallIn(SFc2_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c2_sf_internal_predicateOutput, const char_T
  *c2_identifier);
static boolean_T c2_c_emlrt_marshallIn(SFc2_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static void c2_info_helper(const mxArray **c2_info);
static const mxArray *c2_emlrt_marshallOut(const char * c2_u);
static const mxArray *c2_b_emlrt_marshallOut(const uint32_T c2_u);
static const mxArray *c2_c_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static int32_T c2_d_emlrt_marshallIn(SFc2_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c2_b_sfEvent, const char_T *c2_identifier);
static int32_T c2_e_emlrt_marshallIn(SFc2_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_d_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static uint8_T c2_f_emlrt_marshallIn(SFc2_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c2_b_tp_Standby, const char_T *c2_identifier);
static uint8_T c2_g_emlrt_marshallIn(SFc2_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_e_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static real32_T c2_h_emlrt_marshallIn(SFc2_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c2_BearingRemainToTarget, const char_T
  *c2_identifier);
static real32_T c2_i_emlrt_marshallIn(SFc2_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_f_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static const mxArray *c2_g_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static const mxArray *c2_h_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static int16_T c2_j_emlrt_marshallIn(SFc2_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c2_b_dist_remaining, const char_T
  *c2_identifier);
static int16_T c2_k_emlrt_marshallIn(SFc2_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_i_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_l_emlrt_marshallIn(SFc2_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[2]);
static void c2_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_j_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static const mxArray *c2_k_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_m_emlrt_marshallIn(SFc2_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c2_b_prev_target, const char_T *c2_identifier,
  real32_T c2_y[2]);
static void c2_n_emlrt_marshallIn(SFc2_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real32_T c2_y[2]);
static void c2_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_o_emlrt_marshallIn(SFc2_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c2_b_setSimStateSideEffectsInfo, const char_T
  *c2_identifier);
static const mxArray *c2_p_emlrt_marshallIn(SFc2_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_updateDataWrittenToVector(SFc2_SimulationModelInstanceStruct
  *chartInstance, uint32_T c2_vectorIndex);
static void c2_errorIfDataNotWrittenToFcn(SFc2_SimulationModelInstanceStruct
  *chartInstance, uint32_T c2_vectorIndex, uint32_T c2_dataNumber);
static void init_dsm_address_info(SFc2_SimulationModelInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c2_SimulationModel(SFc2_SimulationModelInstanceStruct
  *chartInstance)
{
  chartInstance->c2_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c2_doSetSimStateSideEffects = 0U;
  chartInstance->c2_setSimStateSideEffectsInfo = NULL;
  chartInstance->c2_tp_All_targets_done = 0U;
  chartInstance->c2_temporalCounter_i1 = 0U;
  chartInstance->c2_tp_Finished = 0U;
  chartInstance->c2_is_Running = c2_IN_NO_ACTIVE_CHILD;
  chartInstance->c2_tp_Running = 0U;
  chartInstance->c2_tp_Apply_camera_correction = 0U;
  chartInstance->c2_tp_Calc_next_robot_move = 0U;
  chartInstance->c2_tp_Next_Target = 0U;
  chartInstance->c2_tp_Wait_for_scoring = 0U;
  chartInstance->c2_temporalCounter_i1 = 0U;
  chartInstance->c2_tp_Standby = 0U;
  chartInstance->c2_is_active_c2_SimulationModel = 0U;
  chartInstance->c2_is_c2_SimulationModel = c2_IN_NO_ACTIVE_CHILD;
}

static void initialize_params_c2_SimulationModel
  (SFc2_SimulationModelInstanceStruct *chartInstance)
{
  real_T c2_d0;
  real_T c2_d1;
  real_T c2_d2;
  real_T c2_dv0[2];
  int32_T c2_i0;
  real_T c2_dv1[2];
  int32_T c2_i1;
  real_T c2_dv2[2];
  int32_T c2_i2;
  sf_mex_import_named("AxleLength", sf_mex_get_sfun_param(chartInstance->S, 0, 0),
                      &c2_d0, 0, 0, 0U, 0, 0U, 0);
  chartInstance->c2_AxleLength = (real32_T)c2_d0;
  sf_mex_import_named("WheelRadius", sf_mex_get_sfun_param(chartInstance->S, 2,
    0), &c2_d1, 0, 0, 0U, 0, 0U, 0);
  chartInstance->c2_WheelRadius = (real32_T)c2_d1;
  sf_mex_import_named("EncRes", sf_mex_get_sfun_param(chartInstance->S, 1, 0),
                      &c2_d2, 0, 0, 0U, 0, 0U, 0);
  chartInstance->c2_EncRes = (real32_T)c2_d2;
  sf_mex_import_named("startPos", sf_mex_get_sfun_param(chartInstance->S, 5, 0),
                      c2_dv0, 0, 0, 0U, 1, 0U, 2, 1, 2);
  for (c2_i0 = 0; c2_i0 < 2; c2_i0++) {
    chartInstance->c2_startPos[c2_i0] = c2_dv0[c2_i0];
  }

  sf_mex_import_named("pcam", sf_mex_get_sfun_param(chartInstance->S, 4, 0),
                      c2_dv1, 0, 0, 0U, 1, 0U, 2, 1, 2);
  for (c2_i1 = 0; c2_i1 < 2; c2_i1++) {
    chartInstance->c2_pcam[c2_i1] = c2_dv1[c2_i1];
  }

  sf_mex_import_named("lcam", sf_mex_get_sfun_param(chartInstance->S, 3, 0),
                      c2_dv2, 0, 0, 0U, 1, 0U, 2, 1, 2);
  for (c2_i2 = 0; c2_i2 < 2; c2_i2++) {
    chartInstance->c2_lcam[c2_i2] = c2_dv2[c2_i2];
  }
}

static void enable_c2_SimulationModel(SFc2_SimulationModelInstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c2_SimulationModel(SFc2_SimulationModelInstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c2_update_debugger_state_c2_SimulationModel
  (SFc2_SimulationModelInstanceStruct *chartInstance)
{
  uint32_T c2_prevAniVal;
  c2_prevAniVal = _SFD_GET_ANIMATION();
  _SFD_SET_ANIMATION(0U);
  _SFD_SET_HONOR_BREAKPOINTS(0U);
  if (chartInstance->c2_is_active_c2_SimulationModel == 1U) {
    _SFD_CC_CALL(CHART_ACTIVE_TAG, 0U, chartInstance->c2_sfEvent);
  }

  if (chartInstance->c2_is_c2_SimulationModel == c2_IN_Standby) {
    _SFD_CS_CALL(STATE_ACTIVE_TAG, 7U, chartInstance->c2_sfEvent);
  } else {
    _SFD_CS_CALL(STATE_INACTIVE_TAG, 7U, chartInstance->c2_sfEvent);
  }

  if (chartInstance->c2_is_c2_SimulationModel == c2_IN_Running) {
    _SFD_CS_CALL(STATE_ACTIVE_TAG, 2U, chartInstance->c2_sfEvent);
  } else {
    _SFD_CS_CALL(STATE_INACTIVE_TAG, 2U, chartInstance->c2_sfEvent);
  }

  if (chartInstance->c2_is_Running == c2_IN_Next_Target) {
    _SFD_CS_CALL(STATE_ACTIVE_TAG, 5U, chartInstance->c2_sfEvent);
  } else {
    _SFD_CS_CALL(STATE_INACTIVE_TAG, 5U, chartInstance->c2_sfEvent);
  }

  if (chartInstance->c2_is_Running == c2_IN_Apply_camera_correction) {
    _SFD_CS_CALL(STATE_ACTIVE_TAG, 3U, chartInstance->c2_sfEvent);
  } else {
    _SFD_CS_CALL(STATE_INACTIVE_TAG, 3U, chartInstance->c2_sfEvent);
  }

  if (chartInstance->c2_is_Running == c2_IN_Wait_for_scoring) {
    _SFD_CS_CALL(STATE_ACTIVE_TAG, 6U, chartInstance->c2_sfEvent);
  } else {
    _SFD_CS_CALL(STATE_INACTIVE_TAG, 6U, chartInstance->c2_sfEvent);
  }

  if (chartInstance->c2_is_Running == c2_IN_Calc_next_robot_move) {
    _SFD_CS_CALL(STATE_ACTIVE_TAG, 4U, chartInstance->c2_sfEvent);
  } else {
    _SFD_CS_CALL(STATE_INACTIVE_TAG, 4U, chartInstance->c2_sfEvent);
  }

  if (chartInstance->c2_is_c2_SimulationModel == c2_IN_All_targets_done) {
    _SFD_CS_CALL(STATE_ACTIVE_TAG, 0U, chartInstance->c2_sfEvent);
  } else {
    _SFD_CS_CALL(STATE_INACTIVE_TAG, 0U, chartInstance->c2_sfEvent);
  }

  if (chartInstance->c2_is_c2_SimulationModel == c2_IN_Finished) {
    _SFD_CS_CALL(STATE_ACTIVE_TAG, 1U, chartInstance->c2_sfEvent);
  } else {
    _SFD_CS_CALL(STATE_INACTIVE_TAG, 1U, chartInstance->c2_sfEvent);
  }

  _SFD_SET_ANIMATION(c2_prevAniVal);
  _SFD_SET_HONOR_BREAKPOINTS(1U);
  _SFD_ANIMATE();
}

static const mxArray *get_sim_state_c2_SimulationModel
  (SFc2_SimulationModelInstanceStruct *chartInstance)
{
  const mxArray *c2_st;
  const mxArray *c2_y = NULL;
  real32_T c2_hoistedGlobal;
  real32_T c2_u;
  const mxArray *c2_b_y = NULL;
  real32_T c2_b_hoistedGlobal;
  real32_T c2_b_u;
  const mxArray *c2_c_y = NULL;
  boolean_T c2_c_hoistedGlobal;
  boolean_T c2_c_u;
  const mxArray *c2_d_y = NULL;
  boolean_T c2_d_hoistedGlobal;
  boolean_T c2_d_u;
  const mxArray *c2_e_y = NULL;
  boolean_T c2_e_hoistedGlobal;
  boolean_T c2_e_u;
  const mxArray *c2_f_y = NULL;
  real32_T c2_f_hoistedGlobal;
  real32_T c2_f_u;
  const mxArray *c2_g_y = NULL;
  real32_T c2_g_hoistedGlobal;
  real32_T c2_g_u;
  const mxArray *c2_h_y = NULL;
  real32_T c2_h_hoistedGlobal;
  real32_T c2_h_u;
  const mxArray *c2_i_y = NULL;
  real32_T c2_i_hoistedGlobal;
  real32_T c2_i_u;
  const mxArray *c2_j_y = NULL;
  uint8_T c2_j_hoistedGlobal;
  uint8_T c2_j_u;
  const mxArray *c2_k_y = NULL;
  uint8_T c2_k_hoistedGlobal;
  uint8_T c2_k_u;
  const mxArray *c2_l_y = NULL;
  real32_T c2_l_hoistedGlobal;
  real32_T c2_l_u;
  const mxArray *c2_m_y = NULL;
  real32_T c2_m_hoistedGlobal;
  real32_T c2_m_u;
  const mxArray *c2_n_y = NULL;
  int16_T c2_n_hoistedGlobal;
  int16_T c2_n_u;
  const mxArray *c2_o_y = NULL;
  int32_T c2_o_hoistedGlobal;
  int32_T c2_o_u;
  const mxArray *c2_p_y = NULL;
  int32_T c2_p_hoistedGlobal;
  int32_T c2_p_u;
  const mxArray *c2_q_y = NULL;
  uint8_T c2_q_hoistedGlobal;
  uint8_T c2_q_u;
  const mxArray *c2_r_y = NULL;
  real32_T c2_r_hoistedGlobal;
  real32_T c2_r_u;
  const mxArray *c2_s_y = NULL;
  int32_T c2_i3;
  real32_T c2_s_u[2];
  const mxArray *c2_t_y = NULL;
  uint8_T c2_s_hoistedGlobal;
  uint8_T c2_t_u;
  const mxArray *c2_u_y = NULL;
  uint8_T c2_t_hoistedGlobal;
  uint8_T c2_u_u;
  const mxArray *c2_v_y = NULL;
  uint8_T c2_u_hoistedGlobal;
  uint8_T c2_v_u;
  const mxArray *c2_w_y = NULL;
  uint8_T c2_v_hoistedGlobal;
  uint8_T c2_w_u;
  const mxArray *c2_x_y = NULL;
  real32_T *c2_BearingRemainToTarget;
  real32_T *c2_DistanceRemainToTarget;
  boolean_T *c2_allTargetsDone;
  boolean_T *c2_doBeep;
  boolean_T *c2_doScan;
  c2_doScan = (boolean_T *)ssGetOutputPortSignal(chartInstance->S, 5);
  c2_doBeep = (boolean_T *)ssGetOutputPortSignal(chartInstance->S, 4);
  c2_allTargetsDone = (boolean_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c2_DistanceRemainToTarget = (real32_T *)ssGetOutputPortSignal(chartInstance->S,
    2);
  c2_BearingRemainToTarget = (real32_T *)ssGetOutputPortSignal(chartInstance->S,
    1);
  c2_st = NULL;
  c2_st = NULL;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_createcellmatrix(23, 1), false);
  c2_hoistedGlobal = *c2_BearingRemainToTarget;
  c2_u = c2_hoistedGlobal;
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_u, 1, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c2_y, 0, c2_b_y);
  c2_b_hoistedGlobal = *c2_DistanceRemainToTarget;
  c2_b_u = c2_b_hoistedGlobal;
  c2_c_y = NULL;
  sf_mex_assign(&c2_c_y, sf_mex_create("y", &c2_b_u, 1, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c2_y, 1, c2_c_y);
  c2_c_hoistedGlobal = *c2_allTargetsDone;
  c2_c_u = c2_c_hoistedGlobal;
  c2_d_y = NULL;
  sf_mex_assign(&c2_d_y, sf_mex_create("y", &c2_c_u, 11, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c2_y, 2, c2_d_y);
  c2_d_hoistedGlobal = *c2_doBeep;
  c2_d_u = c2_d_hoistedGlobal;
  c2_e_y = NULL;
  sf_mex_assign(&c2_e_y, sf_mex_create("y", &c2_d_u, 11, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c2_y, 3, c2_e_y);
  c2_e_hoistedGlobal = *c2_doScan;
  c2_e_u = c2_e_hoistedGlobal;
  c2_f_y = NULL;
  sf_mex_assign(&c2_f_y, sf_mex_create("y", &c2_e_u, 11, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c2_y, 4, c2_f_y);
  c2_f_hoistedGlobal = chartInstance->c2_BearingToTarget;
  c2_f_u = c2_f_hoistedGlobal;
  c2_g_y = NULL;
  sf_mex_assign(&c2_g_y, sf_mex_create("y", &c2_f_u, 1, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c2_y, 5, c2_g_y);
  c2_g_hoistedGlobal = chartInstance->c2_CurrentBearing;
  c2_g_u = c2_g_hoistedGlobal;
  c2_h_y = NULL;
  sf_mex_assign(&c2_h_y, sf_mex_create("y", &c2_g_u, 1, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c2_y, 6, c2_h_y);
  c2_h_hoistedGlobal = chartInstance->c2_CurrentDistance;
  c2_h_u = c2_h_hoistedGlobal;
  c2_i_y = NULL;
  sf_mex_assign(&c2_i_y, sf_mex_create("y", &c2_h_u, 1, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c2_y, 7, c2_i_y);
  c2_i_hoistedGlobal = chartInstance->c2_DistanceToTarget;
  c2_i_u = c2_i_hoistedGlobal;
  c2_j_y = NULL;
  sf_mex_assign(&c2_j_y, sf_mex_create("y", &c2_i_u, 1, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c2_y, 8, c2_j_y);
  c2_j_hoistedGlobal = chartInstance->c2_TargetFound;
  c2_j_u = c2_j_hoistedGlobal;
  c2_k_y = NULL;
  sf_mex_assign(&c2_k_y, sf_mex_create("y", &c2_j_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c2_y, 9, c2_k_y);
  c2_k_hoistedGlobal = chartInstance->c2_TargetIndex;
  c2_k_u = c2_k_hoistedGlobal;
  c2_l_y = NULL;
  sf_mex_assign(&c2_l_y, sf_mex_create("y", &c2_k_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c2_y, 10, c2_l_y);
  c2_l_hoistedGlobal = chartInstance->c2_dL;
  c2_l_u = c2_l_hoistedGlobal;
  c2_m_y = NULL;
  sf_mex_assign(&c2_m_y, sf_mex_create("y", &c2_l_u, 1, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c2_y, 11, c2_m_y);
  c2_m_hoistedGlobal = chartInstance->c2_dR;
  c2_m_u = c2_m_hoistedGlobal;
  c2_n_y = NULL;
  sf_mex_assign(&c2_n_y, sf_mex_create("y", &c2_m_u, 1, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c2_y, 12, c2_n_y);
  c2_n_hoistedGlobal = chartInstance->c2_dist_remaining;
  c2_n_u = c2_n_hoistedGlobal;
  c2_o_y = NULL;
  sf_mex_assign(&c2_o_y, sf_mex_create("y", &c2_n_u, 4, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c2_y, 13, c2_o_y);
  c2_o_hoistedGlobal = chartInstance->c2_encoder_left_prev_target;
  c2_o_u = c2_o_hoistedGlobal;
  c2_p_y = NULL;
  sf_mex_assign(&c2_p_y, sf_mex_create("y", &c2_o_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c2_y, 14, c2_p_y);
  c2_p_hoistedGlobal = chartInstance->c2_encoder_right_prev_target;
  c2_p_u = c2_p_hoistedGlobal;
  c2_q_y = NULL;
  sf_mex_assign(&c2_q_y, sf_mex_create("y", &c2_p_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c2_y, 15, c2_q_y);
  c2_q_hoistedGlobal = chartInstance->c2_ind;
  c2_q_u = c2_q_hoistedGlobal;
  c2_r_y = NULL;
  sf_mex_assign(&c2_r_y, sf_mex_create("y", &c2_q_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c2_y, 16, c2_r_y);
  c2_r_hoistedGlobal = chartInstance->c2_prev_bearing;
  c2_r_u = c2_r_hoistedGlobal;
  c2_s_y = NULL;
  sf_mex_assign(&c2_s_y, sf_mex_create("y", &c2_r_u, 1, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c2_y, 17, c2_s_y);
  for (c2_i3 = 0; c2_i3 < 2; c2_i3++) {
    c2_s_u[c2_i3] = chartInstance->c2_prev_target[c2_i3];
  }

  c2_t_y = NULL;
  sf_mex_assign(&c2_t_y, sf_mex_create("y", c2_s_u, 1, 0U, 1U, 0U, 2, 1, 2),
                false);
  sf_mex_setcell(c2_y, 18, c2_t_y);
  c2_s_hoistedGlobal = chartInstance->c2_is_active_c2_SimulationModel;
  c2_t_u = c2_s_hoistedGlobal;
  c2_u_y = NULL;
  sf_mex_assign(&c2_u_y, sf_mex_create("y", &c2_t_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c2_y, 19, c2_u_y);
  c2_t_hoistedGlobal = chartInstance->c2_is_c2_SimulationModel;
  c2_u_u = c2_t_hoistedGlobal;
  c2_v_y = NULL;
  sf_mex_assign(&c2_v_y, sf_mex_create("y", &c2_u_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c2_y, 20, c2_v_y);
  c2_u_hoistedGlobal = chartInstance->c2_is_Running;
  c2_v_u = c2_u_hoistedGlobal;
  c2_w_y = NULL;
  sf_mex_assign(&c2_w_y, sf_mex_create("y", &c2_v_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c2_y, 21, c2_w_y);
  c2_v_hoistedGlobal = chartInstance->c2_temporalCounter_i1;
  c2_w_u = c2_v_hoistedGlobal;
  c2_x_y = NULL;
  sf_mex_assign(&c2_x_y, sf_mex_create("y", &c2_w_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c2_y, 22, c2_x_y);
  sf_mex_assign(&c2_st, c2_y, false);
  return c2_st;
}

static void set_sim_state_c2_SimulationModel(SFc2_SimulationModelInstanceStruct *
  chartInstance, const mxArray *c2_st)
{
  const mxArray *c2_u;
  real32_T c2_fv0[2];
  int32_T c2_i4;
  real32_T *c2_BearingRemainToTarget;
  real32_T *c2_DistanceRemainToTarget;
  boolean_T *c2_allTargetsDone;
  boolean_T *c2_doBeep;
  boolean_T *c2_doScan;
  c2_doScan = (boolean_T *)ssGetOutputPortSignal(chartInstance->S, 5);
  c2_doBeep = (boolean_T *)ssGetOutputPortSignal(chartInstance->S, 4);
  c2_allTargetsDone = (boolean_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c2_DistanceRemainToTarget = (real32_T *)ssGetOutputPortSignal(chartInstance->S,
    2);
  c2_BearingRemainToTarget = (real32_T *)ssGetOutputPortSignal(chartInstance->S,
    1);
  c2_u = sf_mex_dup(c2_st);
  *c2_BearingRemainToTarget = c2_h_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c2_u, 0)), "BearingRemainToTarget");
  *c2_DistanceRemainToTarget = c2_h_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c2_u, 1)), "DistanceRemainToTarget");
  *c2_allTargetsDone = c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c2_u, 2)), "allTargetsDone");
  *c2_doBeep = c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell
    (c2_u, 3)), "doBeep");
  *c2_doScan = c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell
    (c2_u, 4)), "doScan");
  chartInstance->c2_BearingToTarget = c2_h_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell(c2_u, 5)), "BearingToTarget");
  chartInstance->c2_CurrentBearing = c2_h_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell(c2_u, 6)), "CurrentBearing");
  chartInstance->c2_CurrentDistance = c2_h_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell(c2_u, 7)), "CurrentDistance");
  chartInstance->c2_DistanceToTarget = c2_h_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell(c2_u, 8)), "DistanceToTarget");
  chartInstance->c2_TargetFound = c2_f_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell(c2_u, 9)), "TargetFound");
  chartInstance->c2_TargetIndex = c2_f_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell(c2_u, 10)), "TargetIndex");
  chartInstance->c2_dL = c2_h_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c2_u, 11)), "dL");
  chartInstance->c2_dR = c2_h_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c2_u, 12)), "dR");
  chartInstance->c2_dist_remaining = c2_j_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell(c2_u, 13)), "dist_remaining");
  chartInstance->c2_encoder_left_prev_target = c2_d_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 14)),
     "encoder_left_prev_target");
  chartInstance->c2_encoder_right_prev_target = c2_d_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 15)),
     "encoder_right_prev_target");
  chartInstance->c2_ind = c2_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c2_u, 16)), "ind");
  chartInstance->c2_prev_bearing = c2_h_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell(c2_u, 17)), "prev_bearing");
  c2_m_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 18)),
                        "prev_target", c2_fv0);
  for (c2_i4 = 0; c2_i4 < 2; c2_i4++) {
    chartInstance->c2_prev_target[c2_i4] = c2_fv0[c2_i4];
  }

  chartInstance->c2_is_active_c2_SimulationModel = c2_f_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 19)),
     "is_active_c2_SimulationModel");
  chartInstance->c2_is_c2_SimulationModel = c2_f_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell(c2_u, 20)), "is_c2_SimulationModel");
  chartInstance->c2_is_Running = c2_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c2_u, 21)), "is_Running");
  chartInstance->c2_temporalCounter_i1 = c2_f_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell(c2_u, 22)), "temporalCounter_i1");
  sf_mex_assign(&chartInstance->c2_setSimStateSideEffectsInfo,
                c2_o_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell
    (c2_u, 23)), "setSimStateSideEffectsInfo"), true);
  sf_mex_destroy(&c2_u);
  chartInstance->c2_doSetSimStateSideEffects = 1U;
  c2_update_debugger_state_c2_SimulationModel(chartInstance);
  sf_mex_destroy(&c2_st);
}

static void c2_set_sim_state_side_effects_c2_SimulationModel
  (SFc2_SimulationModelInstanceStruct *chartInstance)
{
  if (chartInstance->c2_doSetSimStateSideEffects != 0) {
    if (chartInstance->c2_is_c2_SimulationModel == c2_IN_All_targets_done) {
      chartInstance->c2_tp_All_targets_done = 1U;
      if (sf_mex_sub(chartInstance->c2_setSimStateSideEffectsInfo,
                     "setSimStateSideEffectsInfo", 1, 2) == 0.0) {
        chartInstance->c2_temporalCounter_i1 = 0U;
      }
    } else {
      chartInstance->c2_tp_All_targets_done = 0U;
    }

    if (chartInstance->c2_is_c2_SimulationModel == c2_IN_Finished) {
      chartInstance->c2_tp_Finished = 1U;
    } else {
      chartInstance->c2_tp_Finished = 0U;
    }

    if (chartInstance->c2_is_c2_SimulationModel == c2_IN_Running) {
      chartInstance->c2_tp_Running = 1U;
    } else {
      chartInstance->c2_tp_Running = 0U;
    }

    if (chartInstance->c2_is_Running == c2_IN_Apply_camera_correction) {
      chartInstance->c2_tp_Apply_camera_correction = 1U;
    } else {
      chartInstance->c2_tp_Apply_camera_correction = 0U;
    }

    if (chartInstance->c2_is_Running == c2_IN_Calc_next_robot_move) {
      chartInstance->c2_tp_Calc_next_robot_move = 1U;
    } else {
      chartInstance->c2_tp_Calc_next_robot_move = 0U;
    }

    if (chartInstance->c2_is_Running == c2_IN_Next_Target) {
      chartInstance->c2_tp_Next_Target = 1U;
    } else {
      chartInstance->c2_tp_Next_Target = 0U;
    }

    if (chartInstance->c2_is_Running == c2_IN_Wait_for_scoring) {
      chartInstance->c2_tp_Wait_for_scoring = 1U;
      if (sf_mex_sub(chartInstance->c2_setSimStateSideEffectsInfo,
                     "setSimStateSideEffectsInfo", 1, 8) == 0.0) {
        chartInstance->c2_temporalCounter_i1 = 0U;
      }
    } else {
      chartInstance->c2_tp_Wait_for_scoring = 0U;
    }

    if (chartInstance->c2_is_c2_SimulationModel == c2_IN_Standby) {
      chartInstance->c2_tp_Standby = 1U;
    } else {
      chartInstance->c2_tp_Standby = 0U;
    }

    chartInstance->c2_doSetSimStateSideEffects = 0U;
  }
}

static void finalize_c2_SimulationModel(SFc2_SimulationModelInstanceStruct
  *chartInstance)
{
  sf_mex_destroy(&chartInstance->c2_setSimStateSideEffectsInfo);
}

static void sf_gateway_c2_SimulationModel(SFc2_SimulationModelInstanceStruct
  *chartInstance)
{
  int32_T c2_i5;
  int32_T c2_i6;
  int32_T c2_i7;
  int32_T c2_i8;
  int32_T c2_i9;
  int32_T c2_i10;
  int32_T c2_i11;
  real32_T *c2_BearingRemainToTarget;
  int32_T *c2_EncoderLeft;
  int32_T *c2_EncoderRight;
  real32_T *c2_DistanceRemainToTarget;
  boolean_T *c2_Go;
  boolean_T *c2_allTargetsDone;
  boolean_T *c2_doBeep;
  boolean_T *c2_doScan;
  real32_T (*c2_Targets)[16];
  int8_T (*c2_BearingCameraTargets)[6];
  int16_T (*c2_DistanceCameraTargets)[6];
  c2_doScan = (boolean_T *)ssGetOutputPortSignal(chartInstance->S, 5);
  c2_doBeep = (boolean_T *)ssGetOutputPortSignal(chartInstance->S, 4);
  c2_allTargetsDone = (boolean_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c2_Targets = (real32_T (*)[16])ssGetInputPortSignal(chartInstance->S, 5);
  c2_Go = (boolean_T *)ssGetInputPortSignal(chartInstance->S, 4);
  c2_BearingCameraTargets = (int8_T (*)[6])ssGetInputPortSignal(chartInstance->S,
    3);
  c2_DistanceCameraTargets = (int16_T (*)[6])ssGetInputPortSignal
    (chartInstance->S, 2);
  c2_DistanceRemainToTarget = (real32_T *)ssGetOutputPortSignal(chartInstance->S,
    2);
  c2_EncoderRight = (int32_T *)ssGetInputPortSignal(chartInstance->S, 1);
  c2_EncoderLeft = (int32_T *)ssGetInputPortSignal(chartInstance->S, 0);
  c2_BearingRemainToTarget = (real32_T *)ssGetOutputPortSignal(chartInstance->S,
    1);
  c2_set_sim_state_side_effects_c2_SimulationModel(chartInstance);
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  if (chartInstance->c2_temporalCounter_i1 < 31U) {
    chartInstance->c2_temporalCounter_i1++;
  }

  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 0U, chartInstance->c2_sfEvent);
  _SFD_DATA_RANGE_CHECK((real_T)*c2_BearingRemainToTarget, 0U);
  _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c2_TargetIndex, 1U);
  _SFD_DATA_RANGE_CHECK((real_T)*c2_EncoderLeft, 2U);
  _SFD_DATA_RANGE_CHECK((real_T)*c2_EncoderRight, 3U);
  _SFD_DATA_RANGE_CHECK((real_T)*c2_DistanceRemainToTarget, 4U);
  for (c2_i5 = 0; c2_i5 < 6; c2_i5++) {
    _SFD_DATA_RANGE_CHECK((real_T)(*c2_DistanceCameraTargets)[c2_i5], 5U);
  }

  _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c2_TargetFound, 6U);
  for (c2_i6 = 0; c2_i6 < 6; c2_i6++) {
    _SFD_DATA_RANGE_CHECK((real_T)(*c2_BearingCameraTargets)[c2_i6], 7U);
  }

  _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c2_AxleLength, 8U);
  _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c2_WheelRadius, 9U);
  _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c2_EncRes, 10U);
  _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c2_encoder_right_prev_target, 11U);
  _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c2_ind, 12U);
  _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c2_dR, 13U);
  _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c2_dL, 14U);
  _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c2_CurrentDistance, 15U);
  _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c2_CurrentBearing, 16U);
  _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c2_dist_remaining, 17U);
  _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c2_encoder_left_prev_target, 18U);
  _SFD_DATA_RANGE_CHECK((real_T)*c2_Go, 19U);
  for (c2_i7 = 0; c2_i7 < 2; c2_i7++) {
    _SFD_DATA_RANGE_CHECK(chartInstance->c2_startPos[c2_i7], 20U);
  }

  for (c2_i8 = 0; c2_i8 < 16; c2_i8++) {
    _SFD_DATA_RANGE_CHECK((real_T)(*c2_Targets)[c2_i8], 21U);
  }

  for (c2_i9 = 0; c2_i9 < 2; c2_i9++) {
    _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c2_prev_target[c2_i9], 22U);
  }

  _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c2_prev_bearing, 23U);
  _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c2_BearingToTarget, 24U);
  _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c2_DistanceToTarget, 25U);
  _SFD_DATA_RANGE_CHECK((real_T)*c2_allTargetsDone, 26U);
  _SFD_DATA_RANGE_CHECK((real_T)*c2_doBeep, 27U);
  _SFD_DATA_RANGE_CHECK((real_T)*c2_doScan, 28U);
  for (c2_i10 = 0; c2_i10 < 2; c2_i10++) {
    _SFD_DATA_RANGE_CHECK(chartInstance->c2_pcam[c2_i10], 29U);
  }

  for (c2_i11 = 0; c2_i11 < 2; c2_i11++) {
    _SFD_DATA_RANGE_CHECK(chartInstance->c2_lcam[c2_i11], 30U);
  }

  chartInstance->c2_sfEvent = CALL_EVENT;
  c2_chartstep_c2_SimulationModel(chartInstance);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_SimulationModelMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void c2_chartstep_c2_SimulationModel(SFc2_SimulationModelInstanceStruct
  *chartInstance)
{
  uint32_T c2_debug_family_var_map[2];
  real_T c2_nargin = 0.0;
  real_T c2_nargout = 0.0;
  int32_T c2_i12;
  int32_T c2_i13;
  uint32_T c2_b_debug_family_var_map[3];
  real_T c2_b_nargin = 0.0;
  real_T c2_b_nargout = 1.0;
  boolean_T c2_out;
  real_T c2_c_nargin = 0.0;
  real_T c2_c_nargout = 0.0;
  real_T c2_d_nargin = 0.0;
  real_T c2_d_nargout = 1.0;
  boolean_T c2_b_out;
  real_T c2_e_nargin = 0.0;
  real_T c2_e_nargout = 0.0;
  real_T c2_f_nargin = 0.0;
  real_T c2_f_nargout = 1.0;
  boolean_T c2_c_out;
  real_T c2_g_nargin = 0.0;
  real_T c2_g_nargout = 1.0;
  boolean_T c2_d_out;
  boolean_T *c2_allTargetsDone;
  boolean_T *c2_doScan;
  real32_T *c2_DistanceRemainToTarget;
  boolean_T *c2_Go;
  boolean_T *c2_doBeep;
  boolean_T guard1 = false;
  c2_doScan = (boolean_T *)ssGetOutputPortSignal(chartInstance->S, 5);
  c2_doBeep = (boolean_T *)ssGetOutputPortSignal(chartInstance->S, 4);
  c2_allTargetsDone = (boolean_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c2_Go = (boolean_T *)ssGetInputPortSignal(chartInstance->S, 4);
  c2_DistanceRemainToTarget = (real32_T *)ssGetOutputPortSignal(chartInstance->S,
    2);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 0U, chartInstance->c2_sfEvent);
  if (chartInstance->c2_is_active_c2_SimulationModel == 0U) {
    _SFD_CC_CALL(CHART_ENTER_ENTRY_FUNCTION_TAG, 0U, chartInstance->c2_sfEvent);
    chartInstance->c2_is_active_c2_SimulationModel = 1U;
    _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 0U, chartInstance->c2_sfEvent);
    _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 1U, chartInstance->c2_sfEvent);
    chartInstance->c2_is_c2_SimulationModel = c2_IN_Standby;
    _SFD_CS_CALL(STATE_ACTIVE_TAG, 7U, chartInstance->c2_sfEvent);
    chartInstance->c2_tp_Standby = 1U;
    _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c2_debug_family_names,
      c2_debug_family_var_map);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 0U, c2_sf_marshallOut,
      c2_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 1U, c2_sf_marshallOut,
      c2_sf_marshallIn);
    *c2_doBeep = false;
    c2_updateDataWrittenToVector(chartInstance, 17U);
    _SFD_DATA_RANGE_CHECK((real_T)*c2_doBeep, 27U);
    *c2_doScan = false;
    c2_updateDataWrittenToVector(chartInstance, 18U);
    _SFD_DATA_RANGE_CHECK((real_T)*c2_doScan, 28U);
    chartInstance->c2_encoder_left_prev_target = 0;
    c2_updateDataWrittenToVector(chartInstance, 11U);
    _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c2_encoder_left_prev_target,
                          18U);
    chartInstance->c2_encoder_right_prev_target = 0;
    c2_updateDataWrittenToVector(chartInstance, 4U);
    _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c2_encoder_right_prev_target,
                          11U);
    chartInstance->c2_ind = 0U;
    c2_updateDataWrittenToVector(chartInstance, 5U);
    _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c2_ind, 12U);
    chartInstance->c2_CurrentDistance = 0.0F;
    c2_updateDataWrittenToVector(chartInstance, 8U);
    _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c2_CurrentDistance, 15U);
    for (c2_i12 = 0; c2_i12 < 2; c2_i12++) {
      chartInstance->c2_prev_target[c2_i12] = (real32_T)
        chartInstance->c2_startPos[c2_i12];
    }

    c2_updateDataWrittenToVector(chartInstance, 12U);
    for (c2_i13 = 0; c2_i13 < 2; c2_i13++) {
      _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c2_prev_target[c2_i13], 22U);
    }

    chartInstance->c2_prev_bearing = 0.0F;
    c2_updateDataWrittenToVector(chartInstance, 13U);
    _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c2_prev_bearing, 23U);
    chartInstance->c2_TargetIndex = 0U;
    c2_updateDataWrittenToVector(chartInstance, 1U);
    _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c2_TargetIndex, 1U);
    *c2_allTargetsDone = false;
    c2_updateDataWrittenToVector(chartInstance, 16U);
    _SFD_DATA_RANGE_CHECK((real_T)*c2_allTargetsDone, 26U);
    _SFD_SYMBOL_SCOPE_POP();
  } else {
    switch (chartInstance->c2_is_c2_SimulationModel) {
     case c2_IN_All_targets_done:
      CV_CHART_EVAL(0, 0, 1);
      _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG, 9U,
                   chartInstance->c2_sfEvent);
      _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 3U, 3U, c2_n_debug_family_names,
        c2_b_debug_family_var_map);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_b_nargin, 0U, c2_sf_marshallOut,
        c2_sf_marshallIn);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_b_nargout, 1U, c2_sf_marshallOut,
        c2_sf_marshallIn);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_out, 2U, c2_b_sf_marshallOut,
        c2_b_sf_marshallIn);
      c2_out = CV_EML_IF(9, 0, 0, chartInstance->c2_temporalCounter_i1 >= 30);
      _SFD_SYMBOL_SCOPE_POP();
      if (c2_out) {
        _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 9U, chartInstance->c2_sfEvent);
        chartInstance->c2_tp_All_targets_done = 0U;
        _SFD_CS_CALL(STATE_INACTIVE_TAG, 0U, chartInstance->c2_sfEvent);
        chartInstance->c2_is_c2_SimulationModel = c2_IN_Finished;
        _SFD_CS_CALL(STATE_ACTIVE_TAG, 1U, chartInstance->c2_sfEvent);
        chartInstance->c2_tp_Finished = 1U;
        _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c2_h_debug_family_names,
          c2_debug_family_var_map);
        _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_c_nargin, 0U, c2_sf_marshallOut,
          c2_sf_marshallIn);
        _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_c_nargout, 1U,
          c2_sf_marshallOut, c2_sf_marshallIn);
        *c2_allTargetsDone = true;
        c2_updateDataWrittenToVector(chartInstance, 16U);
        _SFD_DATA_RANGE_CHECK((real_T)*c2_allTargetsDone, 26U);
        *c2_doScan = false;
        c2_updateDataWrittenToVector(chartInstance, 18U);
        _SFD_DATA_RANGE_CHECK((real_T)*c2_doScan, 28U);
        _SFD_SYMBOL_SCOPE_POP();
      } else {
        _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG, 0U,
                     chartInstance->c2_sfEvent);
      }

      _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG, 0U, chartInstance->c2_sfEvent);
      break;

     case c2_IN_Finished:
      CV_CHART_EVAL(0, 0, 2);
      _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG, 1U,
                   chartInstance->c2_sfEvent);
      _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG, 1U, chartInstance->c2_sfEvent);
      break;

     case c2_IN_Running:
      CV_CHART_EVAL(0, 0, 3);
      _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG, 8U,
                   chartInstance->c2_sfEvent);
      _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 3U, 3U, c2_m_debug_family_names,
        c2_b_debug_family_var_map);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_d_nargin, 0U, c2_sf_marshallOut,
        c2_sf_marshallIn);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_d_nargout, 1U, c2_sf_marshallOut,
        c2_sf_marshallIn);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_b_out, 2U, c2_b_sf_marshallOut,
        c2_b_sf_marshallIn);
      c2_errorIfDataNotWrittenToFcn(chartInstance, 2U, 4U);
      guard1 = false;
      if (CV_EML_COND(8, 0, 0, *c2_DistanceRemainToTarget < 7.0F)) {
        c2_errorIfDataNotWrittenToFcn(chartInstance, 1U, 1U);
        if (CV_EML_COND(8, 0, 1, (real_T)chartInstance->c2_TargetIndex >= 8.0))
        {
          CV_EML_MCDC(8, 0, 0, true);
          CV_EML_IF(8, 0, 0, true);
          c2_b_out = true;
        } else {
          guard1 = true;
        }
      } else {
        guard1 = true;
      }

      if (guard1 == true) {
        CV_EML_MCDC(8, 0, 0, false);
        CV_EML_IF(8, 0, 0, false);
        c2_b_out = false;
      }

      _SFD_SYMBOL_SCOPE_POP();
      if (c2_b_out) {
        _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 8U, chartInstance->c2_sfEvent);
        c2_exit_internal_Running(chartInstance);
        chartInstance->c2_tp_Running = 0U;
        _SFD_CS_CALL(STATE_INACTIVE_TAG, 2U, chartInstance->c2_sfEvent);
        chartInstance->c2_is_c2_SimulationModel = c2_IN_All_targets_done;
        _SFD_CS_CALL(STATE_ACTIVE_TAG, 0U, chartInstance->c2_sfEvent);
        chartInstance->c2_temporalCounter_i1 = 0U;
        chartInstance->c2_tp_All_targets_done = 1U;
        _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c2_g_debug_family_names,
          c2_debug_family_var_map);
        _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_e_nargin, 0U, c2_sf_marshallOut,
          c2_sf_marshallIn);
        _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_e_nargout, 1U,
          c2_sf_marshallOut, c2_sf_marshallIn);
        *c2_DistanceRemainToTarget = 0.0F;
        c2_updateDataWrittenToVector(chartInstance, 2U);
        _SFD_DATA_RANGE_CHECK((real_T)*c2_DistanceRemainToTarget, 4U);
        *c2_doScan = true;
        c2_updateDataWrittenToVector(chartInstance, 18U);
        _SFD_DATA_RANGE_CHECK((real_T)*c2_doScan, 28U);
        _SFD_SYMBOL_SCOPE_POP();
      } else {
        _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG, 2U,
                     chartInstance->c2_sfEvent);
        switch (chartInstance->c2_is_Running) {
         case c2_IN_Apply_camera_correction:
          CV_STATE_EVAL(2, 0, 1);
          _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 7U, chartInstance->c2_sfEvent);
          chartInstance->c2_tp_Apply_camera_correction = 0U;
          _SFD_CS_CALL(STATE_INACTIVE_TAG, 3U, chartInstance->c2_sfEvent);
          chartInstance->c2_is_Running = c2_IN_Calc_next_robot_move;
          _SFD_CS_CALL(STATE_ACTIVE_TAG, 4U, chartInstance->c2_sfEvent);
          chartInstance->c2_tp_Calc_next_robot_move = 1U;
          c2_enter_atomic_Calc_next_robot_move(chartInstance);
          _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG, 3U, chartInstance->c2_sfEvent);
          break;

         case c2_IN_Calc_next_robot_move:
          CV_STATE_EVAL(2, 0, 2);
          c2_Calc_next_robot_move(chartInstance);
          break;

         case c2_IN_Next_Target:
          CV_STATE_EVAL(2, 0, 3);
          _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 5U, chartInstance->c2_sfEvent);
          chartInstance->c2_tp_Next_Target = 0U;
          _SFD_CS_CALL(STATE_INACTIVE_TAG, 5U, chartInstance->c2_sfEvent);
          chartInstance->c2_is_Running = c2_IN_Calc_next_robot_move;
          _SFD_CS_CALL(STATE_ACTIVE_TAG, 4U, chartInstance->c2_sfEvent);
          chartInstance->c2_tp_Calc_next_robot_move = 1U;
          c2_enter_atomic_Calc_next_robot_move(chartInstance);
          _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG, 5U, chartInstance->c2_sfEvent);
          break;

         case c2_IN_Wait_for_scoring:
          CV_STATE_EVAL(2, 0, 4);
          _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG, 4U,
                       chartInstance->c2_sfEvent);
          _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 3U, 3U, c2_i_debug_family_names,
            c2_b_debug_family_var_map);
          _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_f_nargin, 0U,
            c2_sf_marshallOut, c2_sf_marshallIn);
          _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_f_nargout, 1U,
            c2_sf_marshallOut, c2_sf_marshallIn);
          _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_c_out, 2U,
            c2_b_sf_marshallOut, c2_b_sf_marshallIn);
          c2_c_out = CV_EML_IF(4, 0, 0, chartInstance->c2_temporalCounter_i1 >=
                               30);
          _SFD_SYMBOL_SCOPE_POP();
          if (c2_c_out) {
            _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 4U, chartInstance->c2_sfEvent);
            chartInstance->c2_tp_Wait_for_scoring = 0U;
            _SFD_CS_CALL(STATE_INACTIVE_TAG, 6U, chartInstance->c2_sfEvent);
            chartInstance->c2_is_Running = c2_IN_Next_Target;
            _SFD_CS_CALL(STATE_ACTIVE_TAG, 5U, chartInstance->c2_sfEvent);
            chartInstance->c2_tp_Next_Target = 1U;
            c2_enter_atomic_Next_Target(chartInstance);
          } else {
            _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG, 6U,
                         chartInstance->c2_sfEvent);
          }

          _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG, 6U, chartInstance->c2_sfEvent);
          break;

         default:
          CV_STATE_EVAL(2, 0, 0);
          chartInstance->c2_is_Running = c2_IN_NO_ACTIVE_CHILD;
          _SFD_CS_CALL(STATE_INACTIVE_TAG, 3U, chartInstance->c2_sfEvent);
          break;
        }
      }

      _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG, 2U, chartInstance->c2_sfEvent);
      break;

     case c2_IN_Standby:
      CV_CHART_EVAL(0, 0, 4);
      _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG, 0U,
                   chartInstance->c2_sfEvent);
      _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 3U, 3U, c2_l_debug_family_names,
        c2_b_debug_family_var_map);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_g_nargin, 0U, c2_sf_marshallOut,
        c2_sf_marshallIn);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_g_nargout, 1U, c2_sf_marshallOut,
        c2_sf_marshallIn);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_d_out, 2U, c2_b_sf_marshallOut,
        c2_b_sf_marshallIn);
      c2_d_out = CV_EML_IF(0, 0, 0, *c2_Go);
      _SFD_SYMBOL_SCOPE_POP();
      if (c2_d_out) {
        _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 0U, chartInstance->c2_sfEvent);
        chartInstance->c2_tp_Standby = 0U;
        _SFD_CS_CALL(STATE_INACTIVE_TAG, 7U, chartInstance->c2_sfEvent);
        chartInstance->c2_is_c2_SimulationModel = c2_IN_Running;
        _SFD_CS_CALL(STATE_ACTIVE_TAG, 2U, chartInstance->c2_sfEvent);
        chartInstance->c2_tp_Running = 1U;
        _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 2U, chartInstance->c2_sfEvent);
        chartInstance->c2_is_Running = c2_IN_Next_Target;
        _SFD_CS_CALL(STATE_ACTIVE_TAG, 5U, chartInstance->c2_sfEvent);
        chartInstance->c2_tp_Next_Target = 1U;
        c2_enter_atomic_Next_Target(chartInstance);
      } else {
        _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG, 7U,
                     chartInstance->c2_sfEvent);
      }

      _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG, 7U, chartInstance->c2_sfEvent);
      break;

     default:
      CV_CHART_EVAL(0, 0, 0);
      chartInstance->c2_is_c2_SimulationModel = c2_IN_NO_ACTIVE_CHILD;
      _SFD_CS_CALL(STATE_INACTIVE_TAG, 0U, chartInstance->c2_sfEvent);
      break;
    }
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 0U, chartInstance->c2_sfEvent);
}

static void initSimStructsc2_SimulationModel(SFc2_SimulationModelInstanceStruct *
  chartInstance)
{
  (void)chartInstance;
}

static void c2_exit_internal_Running(SFc2_SimulationModelInstanceStruct
  *chartInstance)
{
  switch (chartInstance->c2_is_Running) {
   case c2_IN_Apply_camera_correction:
    CV_STATE_EVAL(2, 1, 1);
    chartInstance->c2_tp_Apply_camera_correction = 0U;
    chartInstance->c2_is_Running = c2_IN_NO_ACTIVE_CHILD;
    _SFD_CS_CALL(STATE_INACTIVE_TAG, 3U, chartInstance->c2_sfEvent);
    break;

   case c2_IN_Calc_next_robot_move:
    CV_STATE_EVAL(2, 1, 2);
    chartInstance->c2_tp_Calc_next_robot_move = 0U;
    chartInstance->c2_is_Running = c2_IN_NO_ACTIVE_CHILD;
    _SFD_CS_CALL(STATE_INACTIVE_TAG, 4U, chartInstance->c2_sfEvent);
    break;

   case c2_IN_Next_Target:
    CV_STATE_EVAL(2, 1, 3);
    chartInstance->c2_tp_Next_Target = 0U;
    chartInstance->c2_is_Running = c2_IN_NO_ACTIVE_CHILD;
    _SFD_CS_CALL(STATE_INACTIVE_TAG, 5U, chartInstance->c2_sfEvent);
    break;

   case c2_IN_Wait_for_scoring:
    CV_STATE_EVAL(2, 1, 4);
    chartInstance->c2_tp_Wait_for_scoring = 0U;
    chartInstance->c2_is_Running = c2_IN_NO_ACTIVE_CHILD;
    _SFD_CS_CALL(STATE_INACTIVE_TAG, 6U, chartInstance->c2_sfEvent);
    break;

   default:
    CV_STATE_EVAL(2, 1, 0);
    chartInstance->c2_is_Running = c2_IN_NO_ACTIVE_CHILD;
    _SFD_CS_CALL(STATE_INACTIVE_TAG, 3U, chartInstance->c2_sfEvent);
    break;
  }
}

static void c2_enter_atomic_Next_Target(SFc2_SimulationModelInstanceStruct
  *chartInstance)
{
  uint32_T c2_debug_family_var_map[2];
  real_T c2_nargin = 0.0;
  real_T c2_nargout = 0.0;
  uint32_T c2_u0;
  uint8_T c2_hoistedGlobal;
  int32_T c2_b_hoistedGlobal;
  int32_T c2_i14;
  real32_T c2_x[2];
  int32_T c2_i15;
  real32_T c2_b_x[2];
  real32_T c2_y;
  uint8_T c2_c_hoistedGlobal;
  uint8_T c2_d_hoistedGlobal;
  real32_T c2_b_y;
  real32_T c2_c_x;
  real32_T c2_c_y;
  real32_T c2_d_x;
  real32_T c2_r;
  real32_T c2_b;
  real32_T c2_b_r;
  int32_T c2_i16;
  int32_T c2_i17;
  int32_T c2_i18;
  boolean_T *c2_doScan;
  int32_T *c2_EncoderLeft;
  int32_T *c2_EncoderRight;
  real32_T *c2_DistanceRemainToTarget;
  real32_T *c2_BearingRemainToTarget;
  real32_T (*c2_Targets)[16];
  c2_doScan = (boolean_T *)ssGetOutputPortSignal(chartInstance->S, 5);
  c2_Targets = (real32_T (*)[16])ssGetInputPortSignal(chartInstance->S, 5);
  c2_DistanceRemainToTarget = (real32_T *)ssGetOutputPortSignal(chartInstance->S,
    2);
  c2_EncoderRight = (int32_T *)ssGetInputPortSignal(chartInstance->S, 1);
  c2_EncoderLeft = (int32_T *)ssGetInputPortSignal(chartInstance->S, 0);
  c2_BearingRemainToTarget = (real32_T *)ssGetOutputPortSignal(chartInstance->S,
    1);
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c2_b_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 0U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 1U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  *c2_doScan = false;
  c2_updateDataWrittenToVector(chartInstance, 18U);
  _SFD_DATA_RANGE_CHECK((real_T)*c2_doScan, 28U);
  c2_errorIfDataNotWrittenToFcn(chartInstance, 1U, 1U);
  c2_u0 = (uint32_T)chartInstance->c2_TargetIndex + 1U;
  if (CV_SATURATION_EVAL(4, 5, 0, 0, c2_u0 > 255U)) {
    c2_u0 = 255U;
  }

  chartInstance->c2_TargetIndex = (uint8_T)c2_u0;
  c2_updateDataWrittenToVector(chartInstance, 1U);
  _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c2_TargetIndex, 1U);
  c2_errorIfDataNotWrittenToFcn(chartInstance, 1U, 1U);
  c2_errorIfDataNotWrittenToFcn(chartInstance, 12U, 22U);
  c2_hoistedGlobal = chartInstance->c2_TargetIndex;
  c2_b_hoistedGlobal = _SFD_ARRAY_BOUNDS_CHECK(21U, (uint8_T)
    _SFD_EML_ARRAY_BOUNDS_CHECK(
    "State Next_Target #27 in Chart \'InputProcessing/Estimator\' (#24):   DistanceToTarget = norm(Targets(TargetIndex,1:2)-prev_target"
    ");\n                                                                                            ^^^^^^^",
    (int32_T)(uint8_T)_SFD_INTEGER_CHECK(
    "State Next_Target #27 in Chart \'InputProcessing/Estimator\' (#24):   DistanceToTarget = norm(Targets(TargetIndex,1:2)-prev_target"
    ");\n                                                                                                    ^^^^^^^^^^^",
    (real_T)c2_hoistedGlobal), 1, 8, 1, 0) - 1, 0, 7, 1, 0);
  for (c2_i14 = 0; c2_i14 < 2; c2_i14++) {
    c2_x[c2_i14] = (*c2_Targets)[c2_b_hoistedGlobal + (c2_i14 << 3)] -
      chartInstance->c2_prev_target[c2_i14];
  }

  for (c2_i15 = 0; c2_i15 < 2; c2_i15++) {
    c2_b_x[c2_i15] = c2_x[c2_i15];
  }

  c2_y = c2_eml_xnrm2(chartInstance, c2_b_x);
  chartInstance->c2_DistanceToTarget = c2_y;
  c2_updateDataWrittenToVector(chartInstance, 15U);
  _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c2_DistanceToTarget, 25U);
  c2_errorIfDataNotWrittenToFcn(chartInstance, 1U, 1U);
  c2_errorIfDataNotWrittenToFcn(chartInstance, 12U, 22U);
  c2_errorIfDataNotWrittenToFcn(chartInstance, 13U, 23U);
  c2_c_hoistedGlobal = chartInstance->c2_TargetIndex;
  c2_d_hoistedGlobal = chartInstance->c2_TargetIndex;
  c2_b_y = (*c2_Targets)[8 + _SFD_ARRAY_BOUNDS_CHECK(21U, (uint8_T)
    _SFD_EML_ARRAY_BOUNDS_CHECK(
    "State Next_Target #27 in Chart \'InputProcessing/Estimator\' (#24):   BearingToTarget = atan2d(Targets(TargetIndex,2)-prev_target("
    "2),...\n                                                                                             ^^^^^^^",
    (int32_T)(uint8_T)_SFD_INTEGER_CHECK(
    "State Next_Target #27 in Chart \'InputProcessing/Estimator\' (#24):   BearingToTarget = atan2d(Targets(TargetIndex,2)-prev_target("
    "2),...\n                                                                                                     ^^^^^^^^^^^",
    (real_T)c2_c_hoistedGlobal), 1, 8, 1, 0) - 1, 0, 7, 1, 0)] -
    chartInstance->c2_prev_target[1];
  c2_c_x = (*c2_Targets)[_SFD_ARRAY_BOUNDS_CHECK(21U, (uint8_T)
    _SFD_EML_ARRAY_BOUNDS_CHECK(
    "State Next_Target #27 in Chart \'InputProcessing/Estimator\' (#24):       Targets(TargetIndex,1)-prev_target(1)) - prev_bearing;\n "
    "                                                                       ^^^^^^^",
    (int32_T)(uint8_T)_SFD_INTEGER_CHECK(
    "State Next_Target #27 in Chart \'InputProcessing/Estimator\' (#24):       Targets(TargetIndex,1)-prev_target(1)) - prev_bearing;\n "
    "                                                                               ^^^^^^^^^^^",
    (real_T)c2_d_hoistedGlobal), 1, 8, 1, 0) - 1, 0, 7, 1, 0)] -
    chartInstance->c2_prev_target[0];
  c2_c_y = c2_b_y;
  c2_d_x = c2_c_x;
  c2_r = muSingleScalarAtan2(c2_c_y, c2_d_x);
  c2_b = c2_r;
  c2_b_r = 57.2957802F * c2_b;
  chartInstance->c2_BearingToTarget = c2_b_r - chartInstance->c2_prev_bearing;
  c2_updateDataWrittenToVector(chartInstance, 12U);
  c2_updateDataWrittenToVector(chartInstance, 1U);
  c2_updateDataWrittenToVector(chartInstance, 14U);
  _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c2_BearingToTarget, 24U);
  c2_errorIfDataNotWrittenToFcn(chartInstance, 1U, 1U);
  c2_i16 = _SFD_ARRAY_BOUNDS_CHECK(21U, (uint8_T)_SFD_EML_ARRAY_BOUNDS_CHECK(
    "State Next_Target #27 in Chart \'InputProcessing/Estimator\' (#24):   prev_target = Targets(TargetIndex,1:2);\n                    "
    "                                                              ^^^^^^^",
    (int32_T)(uint8_T)_SFD_INTEGER_CHECK(
    "State Next_Target #27 in Chart \'InputProcessing/Estimator\' (#24):   prev_target = Targets(TargetIndex,1:2);\n                    "
    "                                                                      ^^^^^^^^^^^",
                                     (real_T)chartInstance->c2_TargetIndex), 1,
    8, 1, 0) - 1, 0, 7, 1, 0);
  for (c2_i17 = 0; c2_i17 < 2; c2_i17++) {
    chartInstance->c2_prev_target[c2_i17] = (*c2_Targets)[c2_i16 + (c2_i17 << 3)];
  }

  c2_updateDataWrittenToVector(chartInstance, 12U);
  for (c2_i18 = 0; c2_i18 < 2; c2_i18++) {
    _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c2_prev_target[c2_i18], 22U);
  }

  c2_errorIfDataNotWrittenToFcn(chartInstance, 13U, 23U);
  c2_errorIfDataNotWrittenToFcn(chartInstance, 14U, 24U);
  chartInstance->c2_prev_bearing += chartInstance->c2_BearingToTarget;
  c2_updateDataWrittenToVector(chartInstance, 13U);
  _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c2_prev_bearing, 23U);
  chartInstance->c2_encoder_left_prev_target = *c2_EncoderLeft;
  c2_updateDataWrittenToVector(chartInstance, 11U);
  _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c2_encoder_left_prev_target, 18U);
  chartInstance->c2_encoder_right_prev_target = *c2_EncoderRight;
  c2_updateDataWrittenToVector(chartInstance, 4U);
  _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c2_encoder_right_prev_target, 11U);
  c2_errorIfDataNotWrittenToFcn(chartInstance, 15U, 25U);
  *c2_DistanceRemainToTarget = chartInstance->c2_DistanceToTarget;
  c2_updateDataWrittenToVector(chartInstance, 2U);
  _SFD_DATA_RANGE_CHECK((real_T)*c2_DistanceRemainToTarget, 4U);
  c2_errorIfDataNotWrittenToFcn(chartInstance, 14U, 24U);
  *c2_BearingRemainToTarget = chartInstance->c2_BearingToTarget;
  c2_updateDataWrittenToVector(chartInstance, 0U);
  _SFD_DATA_RANGE_CHECK((real_T)*c2_BearingRemainToTarget, 0U);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c2_enter_atomic_Apply_camera_correction
  (SFc2_SimulationModelInstanceStruct *chartInstance)
{
  uint32_T c2_debug_family_var_map[2];
  real_T c2_nargin = 0.0;
  real_T c2_nargout = 0.0;
  real32_T c2_f0;
  int16_T c2_i19;
  uint8_T c2_b_ind;
  uint8_T c2_hoistedGlobal;
  int16_T c2_b_hoistedGlobal;
  int32_T c2_i20;
  int16_T c2_x;
  int16_T c2_b_x;
  int32_T c2_i21;
  int16_T c2_y;
  uint8_T c2_c_hoistedGlobal;
  int16_T c2_d_hoistedGlobal;
  int32_T c2_i22;
  int16_T c2_c_x;
  int16_T c2_d_x;
  int32_T c2_i23;
  int16_T c2_b_y;
  uint8_T c2_e_hoistedGlobal;
  int16_T c2_f_hoistedGlobal;
  int32_T c2_i24;
  int16_T c2_e_x;
  int16_T c2_f_x;
  int32_T c2_i25;
  int16_T c2_c_y;
  uint8_T c2_g_hoistedGlobal;
  real32_T c2_A;
  real32_T c2_g_x;
  real32_T c2_h_x;
  real32_T c2_i_x;
  boolean_T *c2_doBeep;
  int32_T *c2_EncoderRight;
  int32_T *c2_EncoderLeft;
  real32_T *c2_DistanceRemainToTarget;
  int16_T (*c2_DistanceCameraTargets)[6];
  int8_T (*c2_BearingCameraTargets)[6];
  boolean_T guard1 = false;
  c2_doBeep = (boolean_T *)ssGetOutputPortSignal(chartInstance->S, 4);
  c2_BearingCameraTargets = (int8_T (*)[6])ssGetInputPortSignal(chartInstance->S,
    3);
  c2_DistanceCameraTargets = (int16_T (*)[6])ssGetInputPortSignal
    (chartInstance->S, 2);
  c2_DistanceRemainToTarget = (real32_T *)ssGetOutputPortSignal(chartInstance->S,
    2);
  c2_EncoderRight = (int32_T *)ssGetInputPortSignal(chartInstance->S, 1);
  c2_EncoderLeft = (int32_T *)ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c2_c_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 0U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 1U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  chartInstance->c2_TargetFound = 0U;
  c2_updateDataWrittenToVector(chartInstance, 3U);
  _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c2_TargetFound, 6U);
  c2_errorIfDataNotWrittenToFcn(chartInstance, 2U, 4U);
  c2_f0 = muSingleScalarRound(*c2_DistanceRemainToTarget);
  if (c2_f0 < 32768.0F) {
    if (CV_SATURATION_EVAL(4, 3, 0, 1, c2_f0 >= -32768.0F)) {
      c2_i19 = (int16_T)c2_f0;
    } else {
      c2_i19 = MIN_int16_T;
    }
  } else if (CV_SATURATION_EVAL(4, 3, 0, 0, c2_f0 >= 32768.0F)) {
    c2_i19 = MAX_int16_T;
  } else {
    c2_i19 = 0;
  }

  chartInstance->c2_dist_remaining = c2_i19;
  c2_updateDataWrittenToVector(chartInstance, 10U);
  _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c2_dist_remaining, 17U);
  chartInstance->c2_ind = 1U;
  c2_updateDataWrittenToVector(chartInstance, 5U);
  _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c2_ind, 12U);
  c2_b_ind = 1U;
  while (c2_b_ind < 7) {
    chartInstance->c2_ind = c2_b_ind;
    c2_updateDataWrittenToVector(chartInstance, 5U);
    _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c2_ind, 12U);
    CV_EML_FOR(3, 1, 0, 1);
    c2_errorIfDataNotWrittenToFcn(chartInstance, 5U, 12U);
    if (CV_EML_IF(3, 1, 0, (real_T)(*c2_DistanceCameraTargets)
                  [_SFD_ARRAY_BOUNDS_CHECK(5U, (uint8_T)
          _SFD_EML_ARRAY_BOUNDS_CHECK(
           "State Apply_camera_correction #28 in Chart \'InputProcessing/Estimator\' (#24):          if(DistanceCameraTargets(ind) ~= 0 )\n    "
           "                                                                                      ^^^^^^^^^^^^^^^^^^^^^",
           (int32_T)(uint8_T)_SFD_INTEGER_CHECK(
            "State Apply_camera_correction #28 in Chart \'InputProcessing/Estimator\' (#24):          if(DistanceCameraTargets(ind) ~= 0 )\n    "
            "                                                                                                            ^^^",
            (real_T)chartInstance->c2_ind), 1, 6, 1, 0) - 1, 0, 5, 1, 0)] != 0.0))
    {
      c2_hoistedGlobal = chartInstance->c2_ind;
      c2_b_hoistedGlobal = chartInstance->c2_dist_remaining;
      c2_i20 = (*c2_DistanceCameraTargets)[_SFD_ARRAY_BOUNDS_CHECK(5U, (uint8_T)
        _SFD_EML_ARRAY_BOUNDS_CHECK(
        "State Apply_camera_correction #28 in Chart \'InputProcessing/Estimator\' (#24):              if(abs(DistanceCameraTargets(ind) - d"
        "ist_remaining) < 15)\n                                                                                                  ^^^^^^^^^"
        "^^^^^^^^^^^^", (int32_T)(uint8_T)_SFD_INTEGER_CHECK(
        "State Apply_camera_correction #28 in Chart \'InputProcessing/Estimator\' (#24):              if(abs(DistanceCameraTargets(ind) - d"
        "ist_remaining) < 15)\n                                                                                                           "
        "             ^^^", (real_T)c2_hoistedGlobal), 1, 6, 1, 0) - 1, 0, 5, 1,
        0)] - c2_b_hoistedGlobal;
      if (c2_i20 > 32767) {
        CV_SATURATION_EVAL(4, 3, 2, 0, 1);
        c2_i20 = 32767;
      } else {
        if (CV_SATURATION_EVAL(4, 3, 2, 0, c2_i20 < -32768)) {
          c2_i20 = -32768;
        }
      }

      c2_x = (int16_T)c2_i20;
      c2_b_x = c2_x;
      c2_i21 = -c2_b_x;
      if (c2_i21 > 32767) {
        CV_SATURATION_EVAL(4, 3, 1, 0, 1);
        c2_i21 = 32767;
      } else {
        if (CV_SATURATION_EVAL(4, 3, 1, 0, c2_i21 < -32768)) {
          c2_i21 = -32768;
        }
      }

      if ((real_T)c2_b_x < 0.0) {
        c2_y = (int16_T)c2_i21;
      } else {
        c2_y = c2_b_x;
      }

      if (CV_EML_IF(3, 1, 1, (real_T)c2_y < 15.0)) {
        guard1 = false;
        if (CV_EML_COND(3, 1, 0, (real_T)chartInstance->c2_TargetFound == 0.0))
        {
          guard1 = true;
        } else {
          c2_errorIfDataNotWrittenToFcn(chartInstance, 3U, 6U);
          c2_errorIfDataNotWrittenToFcn(chartInstance, 10U, 17U);
          c2_errorIfDataNotWrittenToFcn(chartInstance, 5U, 12U);
          c2_c_hoistedGlobal = chartInstance->c2_TargetFound;
          c2_d_hoistedGlobal = chartInstance->c2_dist_remaining;
          c2_i22 = (*c2_DistanceCameraTargets)[_SFD_ARRAY_BOUNDS_CHECK(5U,
            (uint8_T)_SFD_EML_ARRAY_BOUNDS_CHECK(
            "State Apply_camera_correction #28 in Chart \'InputProcessing/Estimator\' (#24):                  if (TargetFound == 0 || abs(Dista"
            "nceCameraTargets(TargetFound) -  ...\n                                                                                           "
            "                                ^^^^^^^^^^^^^^^^^^^^^", (int32_T)
            (uint8_T)_SFD_INTEGER_CHECK(
            "State Apply_camera_correction #28 in Chart \'InputProcessing/Estimator\' (#24):                  if (TargetFound == 0 || abs(Dista"
            "nceCameraTargets(TargetFound) -  ...\n                                                                                           "
            "                                                      ^^^^^^^^^^^",
            (real_T)c2_c_hoistedGlobal), 1, 6, 1, 0) - 1, 0, 5, 1, 0)] -
            c2_d_hoistedGlobal;
          if (c2_i22 > 32767) {
            CV_SATURATION_EVAL(4, 3, 4, 0, 1);
            c2_i22 = 32767;
          } else {
            if (CV_SATURATION_EVAL(4, 3, 4, 0, c2_i22 < -32768)) {
              c2_i22 = -32768;
            }
          }

          c2_c_x = (int16_T)c2_i22;
          c2_d_x = c2_c_x;
          c2_i23 = -c2_d_x;
          if (c2_i23 > 32767) {
            CV_SATURATION_EVAL(4, 3, 3, 0, 1);
            c2_i23 = 32767;
          } else {
            if (CV_SATURATION_EVAL(4, 3, 3, 0, c2_i23 < -32768)) {
              c2_i23 = -32768;
            }
          }

          if ((real_T)c2_d_x < 0.0) {
            c2_b_y = (int16_T)c2_i23;
          } else {
            c2_b_y = c2_d_x;
          }

          c2_e_hoistedGlobal = chartInstance->c2_ind;
          c2_f_hoistedGlobal = chartInstance->c2_dist_remaining;
          c2_i24 = (*c2_DistanceCameraTargets)[_SFD_ARRAY_BOUNDS_CHECK(5U,
            (uint8_T)_SFD_EML_ARRAY_BOUNDS_CHECK(
            "State Apply_camera_correction #28 in Chart \'InputProcessing/Estimator\' (#24):                          dist_remaining) > abs(Dis"
            "tanceCameraTargets(ind) - dist_remaining))\n                                                                                     "
            "                                        ^^^^^^^^^^^^^^^^^^^^^",
            (int32_T)(uint8_T)_SFD_INTEGER_CHECK(
            "State Apply_camera_correction #28 in Chart \'InputProcessing/Estimator\' (#24):                          dist_remaining) > abs(Dis"
            "tanceCameraTargets(ind) - dist_remaining))\n                                                                                     "
            "                                                              ^^^",
            (real_T)c2_e_hoistedGlobal), 1, 6, 1, 0) - 1, 0, 5, 1, 0)] -
            c2_f_hoistedGlobal;
          if (c2_i24 > 32767) {
            CV_SATURATION_EVAL(4, 3, 6, 0, 1);
            c2_i24 = 32767;
          } else {
            if (CV_SATURATION_EVAL(4, 3, 6, 0, c2_i24 < -32768)) {
              c2_i24 = -32768;
            }
          }

          c2_e_x = (int16_T)c2_i24;
          c2_f_x = c2_e_x;
          c2_i25 = -c2_f_x;
          if (c2_i25 > 32767) {
            CV_SATURATION_EVAL(4, 3, 5, 0, 1);
            c2_i25 = 32767;
          } else {
            if (CV_SATURATION_EVAL(4, 3, 5, 0, c2_i25 < -32768)) {
              c2_i25 = -32768;
            }
          }

          if ((real_T)c2_f_x < 0.0) {
            c2_c_y = (int16_T)c2_i25;
          } else {
            c2_c_y = c2_f_x;
          }

          if (CV_EML_COND(3, 1, 1, c2_b_y > c2_c_y)) {
            guard1 = true;
          } else {
            c2_updateDataWrittenToVector(chartInstance, 10U);
            CV_EML_MCDC(3, 1, 0, false);
            CV_EML_IF(3, 1, 2, false);
          }
        }

        if (guard1 == true) {
          CV_EML_MCDC(3, 1, 0, true);
          CV_EML_IF(3, 1, 2, true);
          c2_errorIfDataNotWrittenToFcn(chartInstance, 5U, 12U);
          chartInstance->c2_TargetFound = chartInstance->c2_ind;
          c2_updateDataWrittenToVector(chartInstance, 3U);
          _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c2_TargetFound, 6U);
        }
      } else {
        c2_errorIfDataNotWrittenToFcn(chartInstance, 3U, 6U);
      }
    } else {
      c2_errorIfDataNotWrittenToFcn(chartInstance, 10U, 17U);
      c2_errorIfDataNotWrittenToFcn(chartInstance, 5U, 12U);
    }

    c2_b_ind = (uint8_T)((uint32_T)c2_b_ind + 1U);
    _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
  }

  CV_EML_FOR(3, 1, 0, 0);
  c2_errorIfDataNotWrittenToFcn(chartInstance, 3U, 6U);
  if (CV_EML_IF(3, 1, 3, (real_T)chartInstance->c2_TargetFound != 0.0)) {
    *c2_doBeep = true;
    c2_updateDataWrittenToVector(chartInstance, 17U);
    _SFD_DATA_RANGE_CHECK((real_T)*c2_doBeep, 27U);
    c2_errorIfDataNotWrittenToFcn(chartInstance, 3U, 6U);
    chartInstance->c2_DistanceToTarget = (real32_T)(*c2_DistanceCameraTargets)
      [_SFD_ARRAY_BOUNDS_CHECK(5U, (uint8_T)_SFD_EML_ARRAY_BOUNDS_CHECK(
      "State Apply_camera_correction #28 in Chart \'InputProcessing/Estimator\' (#24):          DistanceToTarget = single(DistanceCameraT"
      "argets(TargetFound));\n                                                                                                          "
      "       ^^^^^^^^^^^^^^^^^^^^^", (int32_T)(uint8_T)_SFD_INTEGER_CHECK(
      "State Apply_camera_correction #28 in Chart \'InputProcessing/Estimator\' (#24):          DistanceToTarget = single(DistanceCameraT"
      "argets(TargetFound));\n                                                                                                          "
      "                             ^^^^^^^^^^^", (real_T)
      chartInstance->c2_TargetFound), 1, 6, 1, 0) - 1, 0, 5, 1, 0)];
    c2_updateDataWrittenToVector(chartInstance, 15U);
    _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c2_DistanceToTarget, 25U);
    c2_errorIfDataNotWrittenToFcn(chartInstance, 3U, 6U);
    c2_g_hoistedGlobal = chartInstance->c2_TargetFound;
    c2_A = (real32_T)(*c2_BearingCameraTargets)[_SFD_ARRAY_BOUNDS_CHECK(7U,
      (uint8_T)_SFD_EML_ARRAY_BOUNDS_CHECK(
      "State Apply_camera_correction #28 in Chart \'InputProcessing/Estimator\' (#24):          BearingToTarget = single(BearingCameraTar"
      "gets(TargetFound))/2;\n                                                                                                          "
      "      ^^^^^^^^^^^^^^^^^^^^", (int32_T)(uint8_T)_SFD_INTEGER_CHECK(
      "State Apply_camera_correction #28 in Chart \'InputProcessing/Estimator\' (#24):          BearingToTarget = single(BearingCameraTar"
      "gets(TargetFound))/2;\n                                                                                                          "
      "                           ^^^^^^^^^^^", (real_T)c2_g_hoistedGlobal), 1,
      6, 1, 0) - 1, 0, 5, 1, 0)];
    c2_g_x = c2_A;
    c2_h_x = c2_g_x;
    c2_i_x = c2_h_x;
    chartInstance->c2_BearingToTarget = c2_i_x / 2.0F;
    c2_updateDataWrittenToVector(chartInstance, 14U);
    _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c2_BearingToTarget, 24U);
    chartInstance->c2_encoder_left_prev_target = *c2_EncoderLeft;
    c2_updateDataWrittenToVector(chartInstance, 11U);
    _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c2_encoder_left_prev_target,
                          18U);
    chartInstance->c2_encoder_right_prev_target = *c2_EncoderRight;
    c2_updateDataWrittenToVector(chartInstance, 4U);
    _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c2_encoder_right_prev_target,
                          11U);
  }

  _SFD_SYMBOL_SCOPE_POP();
}

static void c2_enter_atomic_Calc_next_robot_move
  (SFc2_SimulationModelInstanceStruct *chartInstance)
{
  uint32_T c2_debug_family_var_map[2];
  real_T c2_nargin = 0.0;
  real_T c2_nargout = 0.0;
  int32_T c2_hoistedGlobal;
  int32_T c2_b_hoistedGlobal;
  int64_T c2_i26;
  real32_T c2_a;
  real32_T c2_y;
  real32_T c2_c_hoistedGlobal;
  real32_T c2_b_a;
  real32_T c2_b_y;
  real32_T c2_d_hoistedGlobal;
  real32_T c2_A;
  real32_T c2_B;
  real32_T c2_x;
  real32_T c2_c_y;
  real32_T c2_b_x;
  real32_T c2_d_y;
  real32_T c2_c_x;
  real32_T c2_e_y;
  int32_T c2_e_hoistedGlobal;
  int32_T c2_f_hoistedGlobal;
  int64_T c2_i27;
  real32_T c2_c_a;
  real32_T c2_f_y;
  real32_T c2_g_hoistedGlobal;
  real32_T c2_d_a;
  real32_T c2_g_y;
  real32_T c2_h_hoistedGlobal;
  real32_T c2_b_A;
  real32_T c2_b_B;
  real32_T c2_d_x;
  real32_T c2_h_y;
  real32_T c2_e_x;
  real32_T c2_i_y;
  real32_T c2_f_x;
  real32_T c2_j_y;
  real32_T c2_i_hoistedGlobal;
  real32_T c2_j_hoistedGlobal;
  real32_T c2_c_A;
  real32_T c2_g_x;
  real32_T c2_h_x;
  real32_T c2_i_x;
  real32_T c2_k_hoistedGlobal;
  real32_T c2_l_hoistedGlobal;
  real32_T c2_e_a;
  real32_T c2_k_y;
  real32_T c2_m_hoistedGlobal;
  real32_T c2_f_a;
  real32_T c2_l_y;
  real32_T c2_g_a;
  real32_T c2_m_y;
  real32_T c2_d_A;
  real32_T c2_c_B;
  real32_T c2_j_x;
  real32_T c2_n_y;
  real32_T c2_k_x;
  real32_T c2_o_y;
  real32_T c2_l_x;
  real32_T c2_p_y;
  int32_T *c2_EncoderRight;
  int32_T *c2_EncoderLeft;
  real32_T *c2_BearingRemainToTarget;
  real32_T *c2_DistanceRemainToTarget;
  boolean_T *c2_doBeep;
  c2_doBeep = (boolean_T *)ssGetOutputPortSignal(chartInstance->S, 4);
  c2_DistanceRemainToTarget = (real32_T *)ssGetOutputPortSignal(chartInstance->S,
    2);
  c2_EncoderRight = (int32_T *)ssGetInputPortSignal(chartInstance->S, 1);
  c2_EncoderLeft = (int32_T *)ssGetInputPortSignal(chartInstance->S, 0);
  c2_BearingRemainToTarget = (real32_T *)ssGetOutputPortSignal(chartInstance->S,
    1);
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c2_e_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 0U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 1U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  *c2_doBeep = false;
  c2_updateDataWrittenToVector(chartInstance, 17U);
  _SFD_DATA_RANGE_CHECK((real_T)*c2_doBeep, 27U);
  c2_errorIfDataNotWrittenToFcn(chartInstance, 11U, 18U);
  c2_hoistedGlobal = *c2_EncoderLeft;
  c2_b_hoistedGlobal = chartInstance->c2_encoder_left_prev_target;
  c2_i26 = (int64_T)c2_hoistedGlobal - (int64_T)c2_b_hoistedGlobal;
  if (c2_i26 > 2147483647LL) {
    CV_SATURATION_EVAL(4, 4, 0, 0, 1);
    c2_i26 = 2147483647LL;
  } else {
    if (CV_SATURATION_EVAL(4, 4, 0, 0, c2_i26 < -2147483648LL)) {
      c2_i26 = -2147483648LL;
    }
  }

  c2_a = (real32_T)(int32_T)c2_i26;
  c2_y = c2_a * 2.0F;
  c2_c_hoistedGlobal = chartInstance->c2_WheelRadius;
  c2_b_a = c2_y * c2_c_hoistedGlobal;
  c2_b_y = c2_b_a * 3.14159274F;
  c2_d_hoistedGlobal = chartInstance->c2_EncRes;
  c2_A = c2_b_y;
  c2_B = c2_d_hoistedGlobal;
  c2_x = c2_A;
  c2_c_y = c2_B;
  c2_b_x = c2_x;
  c2_d_y = c2_c_y;
  c2_c_x = c2_b_x;
  c2_e_y = c2_d_y;
  chartInstance->c2_dL = c2_c_x / c2_e_y;
  c2_updateDataWrittenToVector(chartInstance, 7U);
  _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c2_dL, 14U);
  c2_errorIfDataNotWrittenToFcn(chartInstance, 4U, 11U);
  c2_e_hoistedGlobal = *c2_EncoderRight;
  c2_f_hoistedGlobal = chartInstance->c2_encoder_right_prev_target;
  c2_i27 = (int64_T)c2_e_hoistedGlobal - (int64_T)c2_f_hoistedGlobal;
  if (c2_i27 > 2147483647LL) {
    CV_SATURATION_EVAL(4, 4, 1, 0, 1);
    c2_i27 = 2147483647LL;
  } else {
    if (CV_SATURATION_EVAL(4, 4, 1, 0, c2_i27 < -2147483648LL)) {
      c2_i27 = -2147483648LL;
    }
  }

  c2_c_a = (real32_T)(int32_T)c2_i27;
  c2_f_y = c2_c_a * 2.0F;
  c2_g_hoistedGlobal = chartInstance->c2_WheelRadius;
  c2_d_a = c2_f_y * c2_g_hoistedGlobal;
  c2_g_y = c2_d_a * 3.14159274F;
  c2_h_hoistedGlobal = chartInstance->c2_EncRes;
  c2_b_A = c2_g_y;
  c2_b_B = c2_h_hoistedGlobal;
  c2_d_x = c2_b_A;
  c2_h_y = c2_b_B;
  c2_e_x = c2_d_x;
  c2_i_y = c2_h_y;
  c2_f_x = c2_e_x;
  c2_j_y = c2_i_y;
  chartInstance->c2_dR = c2_f_x / c2_j_y;
  c2_updateDataWrittenToVector(chartInstance, 6U);
  _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c2_dR, 13U);
  c2_errorIfDataNotWrittenToFcn(chartInstance, 7U, 14U);
  c2_errorIfDataNotWrittenToFcn(chartInstance, 6U, 13U);
  c2_i_hoistedGlobal = chartInstance->c2_dL;
  c2_j_hoistedGlobal = chartInstance->c2_dR;
  c2_c_A = c2_i_hoistedGlobal + c2_j_hoistedGlobal;
  c2_g_x = c2_c_A;
  c2_h_x = c2_g_x;
  c2_i_x = c2_h_x;
  chartInstance->c2_CurrentDistance = c2_i_x / 2.0F;
  c2_updateDataWrittenToVector(chartInstance, 8U);
  _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c2_CurrentDistance, 15U);
  c2_errorIfDataNotWrittenToFcn(chartInstance, 6U, 13U);
  c2_errorIfDataNotWrittenToFcn(chartInstance, 7U, 14U);
  c2_k_hoistedGlobal = chartInstance->c2_dR;
  c2_l_hoistedGlobal = chartInstance->c2_dL;
  c2_e_a = c2_k_hoistedGlobal - c2_l_hoistedGlobal;
  c2_k_y = c2_e_a * 180.0F;
  c2_m_hoistedGlobal = chartInstance->c2_AxleLength;
  c2_f_a = c2_m_hoistedGlobal;
  c2_l_y = c2_f_a * 1.01F;
  c2_g_a = c2_l_y;
  c2_m_y = c2_g_a * 3.14159274F;
  c2_d_A = c2_k_y;
  c2_c_B = c2_m_y;
  c2_j_x = c2_d_A;
  c2_n_y = c2_c_B;
  c2_k_x = c2_j_x;
  c2_o_y = c2_n_y;
  c2_l_x = c2_k_x;
  c2_p_y = c2_o_y;
  chartInstance->c2_CurrentBearing = c2_l_x / c2_p_y;
  c2_updateDataWrittenToVector(chartInstance, 9U);
  _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c2_CurrentBearing, 16U);
  c2_errorIfDataNotWrittenToFcn(chartInstance, 15U, 25U);
  c2_errorIfDataNotWrittenToFcn(chartInstance, 8U, 15U);
  *c2_DistanceRemainToTarget = chartInstance->c2_DistanceToTarget -
    chartInstance->c2_CurrentDistance;
  c2_updateDataWrittenToVector(chartInstance, 2U);
  _SFD_DATA_RANGE_CHECK((real_T)*c2_DistanceRemainToTarget, 4U);
  c2_errorIfDataNotWrittenToFcn(chartInstance, 14U, 24U);
  c2_errorIfDataNotWrittenToFcn(chartInstance, 9U, 16U);
  *c2_BearingRemainToTarget = chartInstance->c2_BearingToTarget -
    chartInstance->c2_CurrentBearing;
  c2_updateDataWrittenToVector(chartInstance, 0U);
  _SFD_DATA_RANGE_CHECK((real_T)*c2_BearingRemainToTarget, 0U);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c2_Calc_next_robot_move(SFc2_SimulationModelInstanceStruct
  *chartInstance)
{
  uint32_T c2_debug_family_var_map[3];
  real_T c2_nargin = 0.0;
  real_T c2_nargout = 1.0;
  boolean_T c2_out;
  uint32_T c2_b_debug_family_var_map[2];
  real_T c2_b_nargin = 0.0;
  real_T c2_b_nargout = 0.0;
  real_T c2_c_nargin = 0.0;
  real_T c2_c_nargout = 1.0;
  boolean_T c2_b_out;
  real_T c2_d_nargin = 0.0;
  real_T c2_d_nargout = 0.0;
  int32_T c2_hoistedGlobal;
  int32_T c2_b_hoistedGlobal;
  int64_T c2_i28;
  real32_T c2_a;
  real32_T c2_y;
  real32_T c2_c_hoistedGlobal;
  real32_T c2_b_a;
  real32_T c2_b_y;
  real32_T c2_d_hoistedGlobal;
  real32_T c2_A;
  real32_T c2_B;
  real32_T c2_x;
  real32_T c2_c_y;
  real32_T c2_b_x;
  real32_T c2_d_y;
  real32_T c2_c_x;
  real32_T c2_e_y;
  int32_T c2_e_hoistedGlobal;
  int32_T c2_f_hoistedGlobal;
  int64_T c2_i29;
  real32_T c2_c_a;
  real32_T c2_f_y;
  real32_T c2_g_hoistedGlobal;
  real32_T c2_d_a;
  real32_T c2_g_y;
  real32_T c2_h_hoistedGlobal;
  real32_T c2_b_A;
  real32_T c2_b_B;
  real32_T c2_d_x;
  real32_T c2_h_y;
  real32_T c2_e_x;
  real32_T c2_i_y;
  real32_T c2_f_x;
  real32_T c2_j_y;
  real32_T c2_i_hoistedGlobal;
  real32_T c2_j_hoistedGlobal;
  real32_T c2_c_A;
  real32_T c2_g_x;
  real32_T c2_h_x;
  real32_T c2_i_x;
  real32_T c2_k_hoistedGlobal;
  real32_T c2_l_hoistedGlobal;
  real32_T c2_e_a;
  real32_T c2_k_y;
  real32_T c2_m_hoistedGlobal;
  real32_T c2_f_a;
  real32_T c2_l_y;
  real32_T c2_g_a;
  real32_T c2_m_y;
  real32_T c2_d_A;
  real32_T c2_c_B;
  real32_T c2_j_x;
  real32_T c2_n_y;
  real32_T c2_k_x;
  real32_T c2_o_y;
  real32_T c2_l_x;
  real32_T c2_p_y;
  boolean_T *c2_doScan;
  real32_T *c2_DistanceRemainToTarget;
  int32_T *c2_EncoderRight;
  int32_T *c2_EncoderLeft;
  real32_T *c2_BearingRemainToTarget;
  boolean_T *c2_doBeep;
  boolean_T guard1 = false;
  c2_doScan = (boolean_T *)ssGetOutputPortSignal(chartInstance->S, 5);
  c2_doBeep = (boolean_T *)ssGetOutputPortSignal(chartInstance->S, 4);
  c2_DistanceRemainToTarget = (real32_T *)ssGetOutputPortSignal(chartInstance->S,
    2);
  c2_EncoderRight = (int32_T *)ssGetInputPortSignal(chartInstance->S, 1);
  c2_EncoderLeft = (int32_T *)ssGetInputPortSignal(chartInstance->S, 0);
  c2_BearingRemainToTarget = (real32_T *)ssGetOutputPortSignal(chartInstance->S,
    1);
  _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG, 3U, chartInstance->c2_sfEvent);
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 3U, 3U, c2_j_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 0U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 1U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_out, 2U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  c2_errorIfDataNotWrittenToFcn(chartInstance, 2U, 4U);
  c2_out = CV_EML_IF(3, 0, 0, *c2_DistanceRemainToTarget < 7.0F);
  _SFD_SYMBOL_SCOPE_POP();
  if (c2_out) {
    _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 3U, chartInstance->c2_sfEvent);
    chartInstance->c2_tp_Calc_next_robot_move = 0U;
    _SFD_CS_CALL(STATE_INACTIVE_TAG, 4U, chartInstance->c2_sfEvent);
    chartInstance->c2_is_Running = c2_IN_Wait_for_scoring;
    _SFD_CS_CALL(STATE_ACTIVE_TAG, 6U, chartInstance->c2_sfEvent);
    chartInstance->c2_temporalCounter_i1 = 0U;
    chartInstance->c2_tp_Wait_for_scoring = 1U;
    _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c2_d_debug_family_names,
      c2_b_debug_family_var_map);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_b_nargin, 0U, c2_sf_marshallOut,
      c2_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_b_nargout, 1U, c2_sf_marshallOut,
      c2_sf_marshallIn);
    *c2_DistanceRemainToTarget = 0.0F;
    c2_updateDataWrittenToVector(chartInstance, 2U);
    _SFD_DATA_RANGE_CHECK((real_T)*c2_DistanceRemainToTarget, 4U);
    *c2_doScan = true;
    c2_updateDataWrittenToVector(chartInstance, 18U);
    _SFD_DATA_RANGE_CHECK((real_T)*c2_doScan, 28U);
    _SFD_SYMBOL_SCOPE_POP();
  } else {
    _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG, 6U, chartInstance->c2_sfEvent);
    _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 3U, 3U, c2_k_debug_family_names,
      c2_debug_family_var_map);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_c_nargin, 0U, c2_sf_marshallOut,
      c2_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_c_nargout, 1U, c2_sf_marshallOut,
      c2_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_b_out, 2U, c2_b_sf_marshallOut,
      c2_b_sf_marshallIn);
    c2_errorIfDataNotWrittenToFcn(chartInstance, 8U, 15U);
    guard1 = false;
    if (CV_EML_COND(6, 0, 0, chartInstance->c2_CurrentDistance > 5.0F)) {
      if (CV_EML_COND(6, 0, 1, (real_T)*c2_DistanceRemainToTarget >
                      chartInstance->c2_pcam[0] - 5.0)) {
        CV_EML_MCDC(6, 0, 0, true);
        CV_EML_IF(6, 0, 0, true);
        c2_b_out = true;
      } else {
        guard1 = true;
      }
    } else {
      c2_errorIfDataNotWrittenToFcn(chartInstance, 2U, 4U);
      guard1 = true;
    }

    if (guard1 == true) {
      CV_EML_MCDC(6, 0, 0, false);
      CV_EML_IF(6, 0, 0, false);
      c2_b_out = false;
    }

    _SFD_SYMBOL_SCOPE_POP();
    if (c2_b_out) {
      _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 6U, chartInstance->c2_sfEvent);
      chartInstance->c2_tp_Calc_next_robot_move = 0U;
      _SFD_CS_CALL(STATE_INACTIVE_TAG, 4U, chartInstance->c2_sfEvent);
      chartInstance->c2_is_Running = c2_IN_Apply_camera_correction;
      _SFD_CS_CALL(STATE_ACTIVE_TAG, 3U, chartInstance->c2_sfEvent);
      chartInstance->c2_tp_Apply_camera_correction = 1U;
      c2_enter_atomic_Apply_camera_correction(chartInstance);
    } else {
      _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG, 4U,
                   chartInstance->c2_sfEvent);
      _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c2_f_debug_family_names,
        c2_b_debug_family_var_map);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_d_nargin, 0U, c2_sf_marshallOut,
        c2_sf_marshallIn);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_d_nargout, 1U, c2_sf_marshallOut,
        c2_sf_marshallIn);
      *c2_doBeep = false;
      c2_updateDataWrittenToVector(chartInstance, 17U);
      _SFD_DATA_RANGE_CHECK((real_T)*c2_doBeep, 27U);
      c2_errorIfDataNotWrittenToFcn(chartInstance, 11U, 18U);
      c2_hoistedGlobal = *c2_EncoderLeft;
      c2_b_hoistedGlobal = chartInstance->c2_encoder_left_prev_target;
      c2_i28 = (int64_T)c2_hoistedGlobal - (int64_T)c2_b_hoistedGlobal;
      if (c2_i28 > 2147483647LL) {
        CV_SATURATION_EVAL(4, 4, 0, 0, 1);
        c2_i28 = 2147483647LL;
      } else {
        if (CV_SATURATION_EVAL(4, 4, 0, 0, c2_i28 < -2147483648LL)) {
          c2_i28 = -2147483648LL;
        }
      }

      c2_a = (real32_T)(int32_T)c2_i28;
      c2_y = c2_a * 2.0F;
      c2_c_hoistedGlobal = chartInstance->c2_WheelRadius;
      c2_b_a = c2_y * c2_c_hoistedGlobal;
      c2_b_y = c2_b_a * 3.14159274F;
      c2_d_hoistedGlobal = chartInstance->c2_EncRes;
      c2_A = c2_b_y;
      c2_B = c2_d_hoistedGlobal;
      c2_x = c2_A;
      c2_c_y = c2_B;
      c2_b_x = c2_x;
      c2_d_y = c2_c_y;
      c2_c_x = c2_b_x;
      c2_e_y = c2_d_y;
      chartInstance->c2_dL = c2_c_x / c2_e_y;
      c2_updateDataWrittenToVector(chartInstance, 7U);
      _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c2_dL, 14U);
      c2_errorIfDataNotWrittenToFcn(chartInstance, 4U, 11U);
      c2_e_hoistedGlobal = *c2_EncoderRight;
      c2_f_hoistedGlobal = chartInstance->c2_encoder_right_prev_target;
      c2_i29 = (int64_T)c2_e_hoistedGlobal - (int64_T)c2_f_hoistedGlobal;
      if (c2_i29 > 2147483647LL) {
        CV_SATURATION_EVAL(4, 4, 1, 0, 1);
        c2_i29 = 2147483647LL;
      } else {
        if (CV_SATURATION_EVAL(4, 4, 1, 0, c2_i29 < -2147483648LL)) {
          c2_i29 = -2147483648LL;
        }
      }

      c2_c_a = (real32_T)(int32_T)c2_i29;
      c2_f_y = c2_c_a * 2.0F;
      c2_g_hoistedGlobal = chartInstance->c2_WheelRadius;
      c2_d_a = c2_f_y * c2_g_hoistedGlobal;
      c2_g_y = c2_d_a * 3.14159274F;
      c2_h_hoistedGlobal = chartInstance->c2_EncRes;
      c2_b_A = c2_g_y;
      c2_b_B = c2_h_hoistedGlobal;
      c2_d_x = c2_b_A;
      c2_h_y = c2_b_B;
      c2_e_x = c2_d_x;
      c2_i_y = c2_h_y;
      c2_f_x = c2_e_x;
      c2_j_y = c2_i_y;
      chartInstance->c2_dR = c2_f_x / c2_j_y;
      c2_updateDataWrittenToVector(chartInstance, 6U);
      _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c2_dR, 13U);
      c2_errorIfDataNotWrittenToFcn(chartInstance, 7U, 14U);
      c2_errorIfDataNotWrittenToFcn(chartInstance, 6U, 13U);
      c2_i_hoistedGlobal = chartInstance->c2_dL;
      c2_j_hoistedGlobal = chartInstance->c2_dR;
      c2_c_A = c2_i_hoistedGlobal + c2_j_hoistedGlobal;
      c2_g_x = c2_c_A;
      c2_h_x = c2_g_x;
      c2_i_x = c2_h_x;
      chartInstance->c2_CurrentDistance = c2_i_x / 2.0F;
      c2_updateDataWrittenToVector(chartInstance, 8U);
      _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c2_CurrentDistance, 15U);
      c2_errorIfDataNotWrittenToFcn(chartInstance, 6U, 13U);
      c2_errorIfDataNotWrittenToFcn(chartInstance, 7U, 14U);
      c2_k_hoistedGlobal = chartInstance->c2_dR;
      c2_l_hoistedGlobal = chartInstance->c2_dL;
      c2_e_a = c2_k_hoistedGlobal - c2_l_hoistedGlobal;
      c2_k_y = c2_e_a * 180.0F;
      c2_m_hoistedGlobal = chartInstance->c2_AxleLength;
      c2_f_a = c2_m_hoistedGlobal;
      c2_l_y = c2_f_a * 1.01F;
      c2_g_a = c2_l_y;
      c2_m_y = c2_g_a * 3.14159274F;
      c2_d_A = c2_k_y;
      c2_c_B = c2_m_y;
      c2_j_x = c2_d_A;
      c2_n_y = c2_c_B;
      c2_k_x = c2_j_x;
      c2_o_y = c2_n_y;
      c2_l_x = c2_k_x;
      c2_p_y = c2_o_y;
      chartInstance->c2_CurrentBearing = c2_l_x / c2_p_y;
      c2_updateDataWrittenToVector(chartInstance, 9U);
      _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c2_CurrentBearing, 16U);
      c2_errorIfDataNotWrittenToFcn(chartInstance, 15U, 25U);
      c2_errorIfDataNotWrittenToFcn(chartInstance, 8U, 15U);
      *c2_DistanceRemainToTarget = chartInstance->c2_DistanceToTarget -
        chartInstance->c2_CurrentDistance;
      c2_updateDataWrittenToVector(chartInstance, 2U);
      _SFD_DATA_RANGE_CHECK((real_T)*c2_DistanceRemainToTarget, 4U);
      c2_errorIfDataNotWrittenToFcn(chartInstance, 14U, 24U);
      c2_errorIfDataNotWrittenToFcn(chartInstance, 9U, 16U);
      *c2_BearingRemainToTarget = chartInstance->c2_BearingToTarget -
        chartInstance->c2_CurrentBearing;
      c2_updateDataWrittenToVector(chartInstance, 0U);
      _SFD_DATA_RANGE_CHECK((real_T)*c2_BearingRemainToTarget, 0U);
      _SFD_SYMBOL_SCOPE_POP();
    }
  }

  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG, 4U, chartInstance->c2_sfEvent);
}

static real32_T c2_eml_xnrm2(SFc2_SimulationModelInstanceStruct *chartInstance,
  real32_T c2_x[2])
{
  real32_T c2_y;
  real32_T c2_scale;
  int32_T c2_k;
  int32_T c2_b_k;
  real32_T c2_b_x;
  real32_T c2_c_x;
  real32_T c2_absxk;
  real32_T c2_t;
  (void)chartInstance;
  c2_y = 0.0F;
  c2_scale = 1.17549435E-38F;
  for (c2_k = 1; c2_k < 3; c2_k++) {
    c2_b_k = c2_k;
    c2_b_x = c2_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c2_b_k), 1, 2, 1, 0) - 1];
    c2_c_x = c2_b_x;
    c2_absxk = muSingleScalarAbs(c2_c_x);
    if (c2_absxk > c2_scale) {
      c2_t = c2_scale / c2_absxk;
      c2_y = 1.0F + c2_y * c2_t * c2_t;
      c2_scale = c2_absxk;
    } else {
      c2_t = c2_absxk / c2_scale;
      c2_y += c2_t * c2_t;
    }
  }

  return c2_scale * muSingleScalarSqrt(c2_y);
}

static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber, uint32_T c2_instanceNumber)
{
  (void)c2_machineNumber;
  (void)c2_chartNumber;
  (void)c2_instanceNumber;
}

static const mxArray *c2_sf_marshallOut(void *chartInstanceVoid, void *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  real_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc2_SimulationModelInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(real_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static real_T c2_emlrt_marshallIn(SFc2_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  real_T c2_y;
  real_T c2_d3;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_d3, 1, 0, 0U, 0, 0U, 0);
  c2_y = c2_d3;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_nargout;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y;
  SFc2_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc2_SimulationModelInstanceStruct *)chartInstanceVoid;
  c2_nargout = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_nargout), &c2_thisId);
  sf_mex_destroy(&c2_nargout);
  *(real_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  boolean_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc2_SimulationModelInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(boolean_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 11, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static boolean_T c2_b_emlrt_marshallIn(SFc2_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c2_sf_internal_predicateOutput, const char_T
  *c2_identifier)
{
  boolean_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_c_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c2_sf_internal_predicateOutput), &c2_thisId);
  sf_mex_destroy(&c2_sf_internal_predicateOutput);
  return c2_y;
}

static boolean_T c2_c_emlrt_marshallIn(SFc2_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  boolean_T c2_y;
  boolean_T c2_b0;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_b0, 1, 11, 0U, 0, 0U, 0);
  c2_y = c2_b0;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_sf_internal_predicateOutput;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  boolean_T c2_y;
  SFc2_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc2_SimulationModelInstanceStruct *)chartInstanceVoid;
  c2_sf_internal_predicateOutput = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_c_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c2_sf_internal_predicateOutput), &c2_thisId);
  sf_mex_destroy(&c2_sf_internal_predicateOutput);
  *(boolean_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

const mxArray *sf_c2_SimulationModel_get_eml_resolved_functions_info(void)
{
  const mxArray *c2_nameCaptureInfo = NULL;
  c2_nameCaptureInfo = NULL;
  sf_mex_assign(&c2_nameCaptureInfo, sf_mex_createstruct("structure", 2, 47, 1),
                false);
  c2_info_helper(&c2_nameCaptureInfo);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c2_nameCaptureInfo);
  return c2_nameCaptureInfo;
}

static void c2_info_helper(const mxArray **c2_info)
{
  const mxArray *c2_rhs0 = NULL;
  const mxArray *c2_lhs0 = NULL;
  const mxArray *c2_rhs1 = NULL;
  const mxArray *c2_lhs1 = NULL;
  const mxArray *c2_rhs2 = NULL;
  const mxArray *c2_lhs2 = NULL;
  const mxArray *c2_rhs3 = NULL;
  const mxArray *c2_lhs3 = NULL;
  const mxArray *c2_rhs4 = NULL;
  const mxArray *c2_lhs4 = NULL;
  const mxArray *c2_rhs5 = NULL;
  const mxArray *c2_lhs5 = NULL;
  const mxArray *c2_rhs6 = NULL;
  const mxArray *c2_lhs6 = NULL;
  const mxArray *c2_rhs7 = NULL;
  const mxArray *c2_lhs7 = NULL;
  const mxArray *c2_rhs8 = NULL;
  const mxArray *c2_lhs8 = NULL;
  const mxArray *c2_rhs9 = NULL;
  const mxArray *c2_lhs9 = NULL;
  const mxArray *c2_rhs10 = NULL;
  const mxArray *c2_lhs10 = NULL;
  const mxArray *c2_rhs11 = NULL;
  const mxArray *c2_lhs11 = NULL;
  const mxArray *c2_rhs12 = NULL;
  const mxArray *c2_lhs12 = NULL;
  const mxArray *c2_rhs13 = NULL;
  const mxArray *c2_lhs13 = NULL;
  const mxArray *c2_rhs14 = NULL;
  const mxArray *c2_lhs14 = NULL;
  const mxArray *c2_rhs15 = NULL;
  const mxArray *c2_lhs15 = NULL;
  const mxArray *c2_rhs16 = NULL;
  const mxArray *c2_lhs16 = NULL;
  const mxArray *c2_rhs17 = NULL;
  const mxArray *c2_lhs17 = NULL;
  const mxArray *c2_rhs18 = NULL;
  const mxArray *c2_lhs18 = NULL;
  const mxArray *c2_rhs19 = NULL;
  const mxArray *c2_lhs19 = NULL;
  const mxArray *c2_rhs20 = NULL;
  const mxArray *c2_lhs20 = NULL;
  const mxArray *c2_rhs21 = NULL;
  const mxArray *c2_lhs21 = NULL;
  const mxArray *c2_rhs22 = NULL;
  const mxArray *c2_lhs22 = NULL;
  const mxArray *c2_rhs23 = NULL;
  const mxArray *c2_lhs23 = NULL;
  const mxArray *c2_rhs24 = NULL;
  const mxArray *c2_lhs24 = NULL;
  const mxArray *c2_rhs25 = NULL;
  const mxArray *c2_lhs25 = NULL;
  const mxArray *c2_rhs26 = NULL;
  const mxArray *c2_lhs26 = NULL;
  const mxArray *c2_rhs27 = NULL;
  const mxArray *c2_lhs27 = NULL;
  const mxArray *c2_rhs28 = NULL;
  const mxArray *c2_lhs28 = NULL;
  const mxArray *c2_rhs29 = NULL;
  const mxArray *c2_lhs29 = NULL;
  const mxArray *c2_rhs30 = NULL;
  const mxArray *c2_lhs30 = NULL;
  const mxArray *c2_rhs31 = NULL;
  const mxArray *c2_lhs31 = NULL;
  const mxArray *c2_rhs32 = NULL;
  const mxArray *c2_lhs32 = NULL;
  const mxArray *c2_rhs33 = NULL;
  const mxArray *c2_lhs33 = NULL;
  const mxArray *c2_rhs34 = NULL;
  const mxArray *c2_lhs34 = NULL;
  const mxArray *c2_rhs35 = NULL;
  const mxArray *c2_lhs35 = NULL;
  const mxArray *c2_rhs36 = NULL;
  const mxArray *c2_lhs36 = NULL;
  const mxArray *c2_rhs37 = NULL;
  const mxArray *c2_lhs37 = NULL;
  const mxArray *c2_rhs38 = NULL;
  const mxArray *c2_lhs38 = NULL;
  const mxArray *c2_rhs39 = NULL;
  const mxArray *c2_lhs39 = NULL;
  const mxArray *c2_rhs40 = NULL;
  const mxArray *c2_lhs40 = NULL;
  const mxArray *c2_rhs41 = NULL;
  const mxArray *c2_lhs41 = NULL;
  const mxArray *c2_rhs42 = NULL;
  const mxArray *c2_lhs42 = NULL;
  const mxArray *c2_rhs43 = NULL;
  const mxArray *c2_lhs43 = NULL;
  const mxArray *c2_rhs44 = NULL;
  const mxArray *c2_lhs44 = NULL;
  const mxArray *c2_rhs45 = NULL;
  const mxArray *c2_lhs45 = NULL;
  const mxArray *c2_rhs46 = NULL;
  const mxArray *c2_lhs46 = NULL;
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 0);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("norm"), "name", "name", 0);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 0);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m"), "resolved",
                  "resolved", 0);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363713868U), "fileTimeLo",
                  "fileTimeLo", 0);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 0);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 0);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 0);
  sf_mex_assign(&c2_rhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs0), "rhs", "rhs", 0);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs0), "lhs", "lhs", 0);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m!genpnorm"),
                  "context", "context", 1);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 1);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 1);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 1);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1323170578U), "fileTimeLo",
                  "fileTimeLo", 1);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 1);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 1);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 1);
  sf_mex_assign(&c2_rhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs1), "rhs", "rhs", 1);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs1), "lhs", "lhs", 1);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m!genpnorm"),
                  "context", "context", 2);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 2);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 2);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 2);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 2);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 2);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 2);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 2);
  sf_mex_assign(&c2_rhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs2), "rhs", "rhs", 2);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs2), "lhs", "lhs", 2);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/matfun/norm.m!genpnorm"),
                  "context", "context", 3);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_xnrm2"), "name", "name", 3);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 3);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xnrm2.m"),
                  "resolved", "resolved", 3);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1375980692U), "fileTimeLo",
                  "fileTimeLo", 3);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 3);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 3);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 3);
  sf_mex_assign(&c2_rhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs3), "rhs", "rhs", 3);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs3), "lhs", "lhs", 3);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xnrm2.m"), "context",
                  "context", 4);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 4);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 4);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 4);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 4);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 4);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 4);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 4);
  sf_mex_assign(&c2_rhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs4), "rhs", "rhs", 4);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs4), "lhs", "lhs", 4);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xnrm2.m"), "context",
                  "context", 5);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.blas.xnrm2"),
                  "name", "name", 5);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 5);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xnrm2.p"),
                  "resolved", "resolved", 5);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 5);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 5);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 5);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 5);
  sf_mex_assign(&c2_rhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs5), "rhs", "rhs", 5);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs5), "lhs", "lhs", 5);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xnrm2.p"),
                  "context", "context", 6);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 6);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 6);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 6);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 6);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 6);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 6);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 6);
  sf_mex_assign(&c2_rhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs6), "rhs", "rhs", 6);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs6), "lhs", "lhs", 6);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xnrm2.p!below_threshold"),
                  "context", "context", 7);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.blas.threshold"),
                  "name", "name", 7);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 7);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 7);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 7);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 7);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 7);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 7);
  sf_mex_assign(&c2_rhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs7), "rhs", "rhs", 7);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs7), "lhs", "lhs", 7);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "context", "context", 8);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 8);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 8);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 8);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1381850300U), "fileTimeLo",
                  "fileTimeLo", 8);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 8);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 8);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 8);
  sf_mex_assign(&c2_rhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs8), "rhs", "rhs", 8);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs8), "lhs", "lhs", 8);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xnrm2.p"),
                  "context", "context", 9);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.refblas.xnrm2"),
                  "name", "name", 9);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 9);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "resolved", "resolved", 9);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 9);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 9);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 9);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 9);
  sf_mex_assign(&c2_rhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs9), "rhs", "rhs", 9);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs9), "lhs", "lhs", 9);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "context", "context", 10);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("realmin"), "name", "name", 10);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 10);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m"), "resolved",
                  "resolved", 10);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1307651242U), "fileTimeLo",
                  "fileTimeLo", 10);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 10);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 10);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 10);
  sf_mex_assign(&c2_rhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs10), "rhs", "rhs",
                  10);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs10), "lhs", "lhs",
                  10);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m"), "context",
                  "context", 11);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_realmin"), "name", "name",
                  11);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 11);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmin.m"), "resolved",
                  "resolved", 11);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1307651244U), "fileTimeLo",
                  "fileTimeLo", 11);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 11);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 11);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 11);
  sf_mex_assign(&c2_rhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs11), "rhs", "rhs",
                  11);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs11), "lhs", "lhs",
                  11);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmin.m"), "context",
                  "context", 12);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_float_model"), "name",
                  "name", 12);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 12);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m"),
                  "resolved", "resolved", 12);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1326727996U), "fileTimeLo",
                  "fileTimeLo", 12);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 12);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 12);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 12);
  sf_mex_assign(&c2_rhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs12), "rhs", "rhs",
                  12);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs12), "lhs", "lhs",
                  12);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "context", "context", 13);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexMinus"),
                  "name", "name", 13);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 13);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexMinus.m"),
                  "resolved", "resolved", 13);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 13);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 13);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 13);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 13);
  sf_mex_assign(&c2_rhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs13), "rhs", "rhs",
                  13);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs13), "lhs", "lhs",
                  13);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "context", "context", 14);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexTimes"),
                  "name", "name", 14);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 14);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexTimes.m"),
                  "resolved", "resolved", 14);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 14);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 14);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 14);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 14);
  sf_mex_assign(&c2_rhs14, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs14, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs14), "rhs", "rhs",
                  14);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs14), "lhs", "lhs",
                  14);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "context", "context", 15);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 15);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 15);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 15);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 15);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 15);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 15);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 15);
  sf_mex_assign(&c2_rhs15, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs15, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs15), "rhs", "rhs",
                  15);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs15), "lhs", "lhs",
                  15);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "context", "context", 16);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 16);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 16);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 16);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 16);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 16);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 16);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 16);
  sf_mex_assign(&c2_rhs16, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs16, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs16), "rhs", "rhs",
                  16);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs16), "lhs", "lhs",
                  16);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper"),
                  "context", "context", 17);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("intmax"), "name", "name", 17);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 17);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "resolved",
                  "resolved", 17);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1362261882U), "fileTimeLo",
                  "fileTimeLo", 17);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 17);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 17);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 17);
  sf_mex_assign(&c2_rhs17, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs17, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs17), "rhs", "rhs",
                  17);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs17), "lhs", "lhs",
                  17);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "context",
                  "context", 18);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 18);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 18);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 18);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1381850300U), "fileTimeLo",
                  "fileTimeLo", 18);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 18);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 18);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 18);
  sf_mex_assign(&c2_rhs18, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs18, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs18), "rhs", "rhs",
                  18);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs18), "lhs", "lhs",
                  18);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "context", "context", 19);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("abs"), "name", "name", 19);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 19);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "resolved",
                  "resolved", 19);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363713852U), "fileTimeLo",
                  "fileTimeLo", 19);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 19);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 19);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 19);
  sf_mex_assign(&c2_rhs19, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs19, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs19), "rhs", "rhs",
                  19);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs19), "lhs", "lhs",
                  19);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 20);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 20);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 20);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 20);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 20);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 20);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 20);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 20);
  sf_mex_assign(&c2_rhs20, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs20, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs20), "rhs", "rhs",
                  20);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs20), "lhs", "lhs",
                  20);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 21);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_abs"), "name",
                  "name", 21);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 21);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m"),
                  "resolved", "resolved", 21);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286818712U), "fileTimeLo",
                  "fileTimeLo", 21);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 21);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 21);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 21);
  sf_mex_assign(&c2_rhs21, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs21, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs21), "rhs", "rhs",
                  21);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs21), "lhs", "lhs",
                  21);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 22);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("atan2d"), "name", "name", 22);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 22);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan2d.m"), "resolved",
                  "resolved", 22);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1343830372U), "fileTimeLo",
                  "fileTimeLo", 22);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 22);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 22);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 22);
  sf_mex_assign(&c2_rhs22, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs22, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs22), "rhs", "rhs",
                  22);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs22), "lhs", "lhs",
                  22);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan2d.m"), "context",
                  "context", 23);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 23);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 23);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 23);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 23);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 23);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 23);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 23);
  sf_mex_assign(&c2_rhs23, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs23, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs23), "rhs", "rhs",
                  23);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs23), "lhs", "lhs",
                  23);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "context",
                  "context", 24);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 24);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 24);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 24);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389307920U), "fileTimeLo",
                  "fileTimeLo", 24);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 24);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 24);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 24);
  sf_mex_assign(&c2_rhs24, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs24, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs24), "rhs", "rhs",
                  24);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs24), "lhs", "lhs",
                  24);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan2d.m"), "context",
                  "context", 25);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalexp_alloc"), "name",
                  "name", 25);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 25);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "resolved", "resolved", 25);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 25);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 25);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 25);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 25);
  sf_mex_assign(&c2_rhs25, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs25, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs25), "rhs", "rhs",
                  25);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs25), "lhs", "lhs",
                  25);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "context", "context", 26);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.scalexpAlloc"),
                  "name", "name", 26);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 26);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalexpAlloc.p"),
                  "resolved", "resolved", 26);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389307920U), "fileTimeLo",
                  "fileTimeLo", 26);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 26);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 26);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 26);
  sf_mex_assign(&c2_rhs26, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs26, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs26), "rhs", "rhs",
                  26);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs26), "lhs", "lhs",
                  26);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan2d.m"), "context",
                  "context", 27);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_atan2"), "name",
                  "name", 27);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 27);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_atan2.m"),
                  "resolved", "resolved", 27);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286818720U), "fileTimeLo",
                  "fileTimeLo", 27);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 27);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 27);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 27);
  sf_mex_assign(&c2_rhs27, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs27, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs27), "rhs", "rhs",
                  27);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs27), "lhs", "lhs",
                  27);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan2d.m"), "context",
                  "context", 28);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_mtimes_helper"), "name",
                  "name", 28);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 28);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "resolved", "resolved", 28);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1383877294U), "fileTimeLo",
                  "fileTimeLo", 28);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 28);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 28);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 28);
  sf_mex_assign(&c2_rhs28, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs28, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs28), "rhs", "rhs",
                  28);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs28), "lhs", "lhs",
                  28);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m!common_checks"),
                  "context", "context", 29);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 29);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 29);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 29);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 29);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 29);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 29);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 29);
  sf_mex_assign(&c2_rhs29, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs29, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs29), "rhs", "rhs",
                  29);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs29), "lhs", "lhs",
                  29);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m!common_checks"),
                  "context", "context", 30);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 30);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 30);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 30);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 30);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 30);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 30);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 30);
  sf_mex_assign(&c2_rhs30, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs30, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs30), "rhs", "rhs",
                  30);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs30), "lhs", "lhs",
                  30);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 31);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("length"), "name", "name", 31);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("int16"), "dominantType",
                  "dominantType", 31);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m"), "resolved",
                  "resolved", 31);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1303146206U), "fileTimeLo",
                  "fileTimeLo", 31);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 31);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 31);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 31);
  sf_mex_assign(&c2_rhs31, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs31, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs31), "rhs", "rhs",
                  31);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs31), "lhs", "lhs",
                  31);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 32);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 32);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 32);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 32);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 32);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 32);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 32);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 32);
  sf_mex_assign(&c2_rhs32, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs32, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs32), "rhs", "rhs",
                  32);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs32), "lhs", "lhs",
                  32);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 33);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("abs"), "name", "name", 33);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("int16"), "dominantType",
                  "dominantType", 33);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "resolved",
                  "resolved", 33);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363713852U), "fileTimeLo",
                  "fileTimeLo", 33);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 33);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 33);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 33);
  sf_mex_assign(&c2_rhs33, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs33, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs33), "rhs", "rhs",
                  33);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs33), "lhs", "lhs",
                  33);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 34);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 34);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("int16"), "dominantType",
                  "dominantType", 34);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 34);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 34);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 34);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 34);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 34);
  sf_mex_assign(&c2_rhs34, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs34, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs34), "rhs", "rhs",
                  34);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs34), "lhs", "lhs",
                  34);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 35);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_abs"), "name",
                  "name", 35);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("int16"), "dominantType",
                  "dominantType", 35);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m"),
                  "resolved", "resolved", 35);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286818712U), "fileTimeLo",
                  "fileTimeLo", 35);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 35);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 35);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 35);
  sf_mex_assign(&c2_rhs35, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs35, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs35), "rhs", "rhs",
                  35);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs35), "lhs", "lhs",
                  35);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 36);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("mrdivide"), "name", "name", 36);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 36);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "resolved",
                  "resolved", 36);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1388460096U), "fileTimeLo",
                  "fileTimeLo", 36);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 36);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1370009886U), "mFileTimeLo",
                  "mFileTimeLo", 36);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 36);
  sf_mex_assign(&c2_rhs36, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs36, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs36), "rhs", "rhs",
                  36);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs36), "lhs", "lhs",
                  36);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 37);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.assert"),
                  "name", "name", 37);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 37);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assert.m"),
                  "resolved", "resolved", 37);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 37);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 37);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 37);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 37);
  sf_mex_assign(&c2_rhs37, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs37, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs37), "rhs", "rhs",
                  37);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs37), "lhs", "lhs",
                  37);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 38);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("rdivide"), "name", "name", 38);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 38);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "resolved",
                  "resolved", 38);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363713880U), "fileTimeLo",
                  "fileTimeLo", 38);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 38);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 38);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 38);
  sf_mex_assign(&c2_rhs38, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs38, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs38), "rhs", "rhs",
                  38);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs38), "lhs", "lhs",
                  38);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 39);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 39);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 39);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 39);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 39);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 39);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 39);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 39);
  sf_mex_assign(&c2_rhs39, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs39, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs39), "rhs", "rhs",
                  39);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs39), "lhs", "lhs",
                  39);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 40);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 40);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 40);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 40);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 40);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 40);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 40);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 40);
  sf_mex_assign(&c2_rhs40, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs40, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs40), "rhs", "rhs",
                  40);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs40), "lhs", "lhs",
                  40);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 41);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalexp_compatible"),
                  "name", "name", 41);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 41);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m"),
                  "resolved", "resolved", 41);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286818796U), "fileTimeLo",
                  "fileTimeLo", 41);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 41);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 41);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 41);
  sf_mex_assign(&c2_rhs41, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs41, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs41), "rhs", "rhs",
                  41);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs41), "lhs", "lhs",
                  41);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 42);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_div"), "name", "name", 42);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 42);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "resolved",
                  "resolved", 42);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 42);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 42);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 42);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 42);
  sf_mex_assign(&c2_rhs42, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs42, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs42), "rhs", "rhs",
                  42);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs42), "lhs", "lhs",
                  42);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "context",
                  "context", 43);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.div"), "name",
                  "name", 43);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 43);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/div.p"), "resolved",
                  "resolved", 43);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389307920U), "fileTimeLo",
                  "fileTimeLo", 43);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 43);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 43);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 43);
  sf_mex_assign(&c2_rhs43, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs43, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs43), "rhs", "rhs",
                  43);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs43), "lhs", "lhs",
                  43);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 44);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_mtimes_helper"), "name",
                  "name", 44);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 44);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "resolved", "resolved", 44);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1383877294U), "fileTimeLo",
                  "fileTimeLo", 44);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 44);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 44);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 44);
  sf_mex_assign(&c2_rhs44, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs44, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs44), "rhs", "rhs",
                  44);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs44), "lhs", "lhs",
                  44);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 45);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("length"), "name", "name", 45);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 45);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m"), "resolved",
                  "resolved", 45);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1303146206U), "fileTimeLo",
                  "fileTimeLo", 45);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 45);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 45);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 45);
  sf_mex_assign(&c2_rhs45, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs45, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs45), "rhs", "rhs",
                  45);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs45), "lhs", "lhs",
                  45);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m!intlength"),
                  "context", "context", 46);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 46);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 46);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 46);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1323170578U), "fileTimeLo",
                  "fileTimeLo", 46);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 46);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 46);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 46);
  sf_mex_assign(&c2_rhs46, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c2_lhs46, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs46), "rhs", "rhs",
                  46);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs46), "lhs", "lhs",
                  46);
  sf_mex_destroy(&c2_rhs0);
  sf_mex_destroy(&c2_lhs0);
  sf_mex_destroy(&c2_rhs1);
  sf_mex_destroy(&c2_lhs1);
  sf_mex_destroy(&c2_rhs2);
  sf_mex_destroy(&c2_lhs2);
  sf_mex_destroy(&c2_rhs3);
  sf_mex_destroy(&c2_lhs3);
  sf_mex_destroy(&c2_rhs4);
  sf_mex_destroy(&c2_lhs4);
  sf_mex_destroy(&c2_rhs5);
  sf_mex_destroy(&c2_lhs5);
  sf_mex_destroy(&c2_rhs6);
  sf_mex_destroy(&c2_lhs6);
  sf_mex_destroy(&c2_rhs7);
  sf_mex_destroy(&c2_lhs7);
  sf_mex_destroy(&c2_rhs8);
  sf_mex_destroy(&c2_lhs8);
  sf_mex_destroy(&c2_rhs9);
  sf_mex_destroy(&c2_lhs9);
  sf_mex_destroy(&c2_rhs10);
  sf_mex_destroy(&c2_lhs10);
  sf_mex_destroy(&c2_rhs11);
  sf_mex_destroy(&c2_lhs11);
  sf_mex_destroy(&c2_rhs12);
  sf_mex_destroy(&c2_lhs12);
  sf_mex_destroy(&c2_rhs13);
  sf_mex_destroy(&c2_lhs13);
  sf_mex_destroy(&c2_rhs14);
  sf_mex_destroy(&c2_lhs14);
  sf_mex_destroy(&c2_rhs15);
  sf_mex_destroy(&c2_lhs15);
  sf_mex_destroy(&c2_rhs16);
  sf_mex_destroy(&c2_lhs16);
  sf_mex_destroy(&c2_rhs17);
  sf_mex_destroy(&c2_lhs17);
  sf_mex_destroy(&c2_rhs18);
  sf_mex_destroy(&c2_lhs18);
  sf_mex_destroy(&c2_rhs19);
  sf_mex_destroy(&c2_lhs19);
  sf_mex_destroy(&c2_rhs20);
  sf_mex_destroy(&c2_lhs20);
  sf_mex_destroy(&c2_rhs21);
  sf_mex_destroy(&c2_lhs21);
  sf_mex_destroy(&c2_rhs22);
  sf_mex_destroy(&c2_lhs22);
  sf_mex_destroy(&c2_rhs23);
  sf_mex_destroy(&c2_lhs23);
  sf_mex_destroy(&c2_rhs24);
  sf_mex_destroy(&c2_lhs24);
  sf_mex_destroy(&c2_rhs25);
  sf_mex_destroy(&c2_lhs25);
  sf_mex_destroy(&c2_rhs26);
  sf_mex_destroy(&c2_lhs26);
  sf_mex_destroy(&c2_rhs27);
  sf_mex_destroy(&c2_lhs27);
  sf_mex_destroy(&c2_rhs28);
  sf_mex_destroy(&c2_lhs28);
  sf_mex_destroy(&c2_rhs29);
  sf_mex_destroy(&c2_lhs29);
  sf_mex_destroy(&c2_rhs30);
  sf_mex_destroy(&c2_lhs30);
  sf_mex_destroy(&c2_rhs31);
  sf_mex_destroy(&c2_lhs31);
  sf_mex_destroy(&c2_rhs32);
  sf_mex_destroy(&c2_lhs32);
  sf_mex_destroy(&c2_rhs33);
  sf_mex_destroy(&c2_lhs33);
  sf_mex_destroy(&c2_rhs34);
  sf_mex_destroy(&c2_lhs34);
  sf_mex_destroy(&c2_rhs35);
  sf_mex_destroy(&c2_lhs35);
  sf_mex_destroy(&c2_rhs36);
  sf_mex_destroy(&c2_lhs36);
  sf_mex_destroy(&c2_rhs37);
  sf_mex_destroy(&c2_lhs37);
  sf_mex_destroy(&c2_rhs38);
  sf_mex_destroy(&c2_lhs38);
  sf_mex_destroy(&c2_rhs39);
  sf_mex_destroy(&c2_lhs39);
  sf_mex_destroy(&c2_rhs40);
  sf_mex_destroy(&c2_lhs40);
  sf_mex_destroy(&c2_rhs41);
  sf_mex_destroy(&c2_lhs41);
  sf_mex_destroy(&c2_rhs42);
  sf_mex_destroy(&c2_lhs42);
  sf_mex_destroy(&c2_rhs43);
  sf_mex_destroy(&c2_lhs43);
  sf_mex_destroy(&c2_rhs44);
  sf_mex_destroy(&c2_lhs44);
  sf_mex_destroy(&c2_rhs45);
  sf_mex_destroy(&c2_lhs45);
  sf_mex_destroy(&c2_rhs46);
  sf_mex_destroy(&c2_lhs46);
}

static const mxArray *c2_emlrt_marshallOut(const char * c2_u)
{
  const mxArray *c2_y = NULL;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 15, 0U, 0U, 0U, 2, 1, strlen
    (c2_u)), false);
  return c2_y;
}

static const mxArray *c2_b_emlrt_marshallOut(const uint32_T c2_u)
{
  const mxArray *c2_y = NULL;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 7, 0U, 0U, 0U, 0), false);
  return c2_y;
}

static const mxArray *c2_c_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc2_SimulationModelInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(int32_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static int32_T c2_d_emlrt_marshallIn(SFc2_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c2_b_sfEvent, const char_T *c2_identifier)
{
  int32_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_sfEvent),
    &c2_thisId);
  sf_mex_destroy(&c2_b_sfEvent);
  return c2_y;
}

static int32_T c2_e_emlrt_marshallIn(SFc2_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  int32_T c2_y;
  int32_T c2_i30;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_i30, 1, 6, 0U, 0, 0U, 0);
  c2_y = c2_i30;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_sfEvent;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  int32_T c2_y;
  SFc2_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc2_SimulationModelInstanceStruct *)chartInstanceVoid;
  c2_b_sfEvent = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_sfEvent),
    &c2_thisId);
  sf_mex_destroy(&c2_b_sfEvent);
  *(int32_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_d_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  uint8_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc2_SimulationModelInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(uint8_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static uint8_T c2_f_emlrt_marshallIn(SFc2_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c2_b_tp_Standby, const char_T *c2_identifier)
{
  uint8_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_g_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_tp_Standby),
    &c2_thisId);
  sf_mex_destroy(&c2_b_tp_Standby);
  return c2_y;
}

static uint8_T c2_g_emlrt_marshallIn(SFc2_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  uint8_T c2_y;
  uint8_T c2_u1;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_u1, 1, 3, 0U, 0, 0U, 0);
  c2_y = c2_u1;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_tp_Standby;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  uint8_T c2_y;
  SFc2_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc2_SimulationModelInstanceStruct *)chartInstanceVoid;
  c2_b_tp_Standby = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_g_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_tp_Standby),
    &c2_thisId);
  sf_mex_destroy(&c2_b_tp_Standby);
  *(uint8_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_e_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  real32_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc2_SimulationModelInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(real32_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 1, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static real32_T c2_h_emlrt_marshallIn(SFc2_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c2_BearingRemainToTarget, const char_T
  *c2_identifier)
{
  real32_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_i_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c2_BearingRemainToTarget), &c2_thisId);
  sf_mex_destroy(&c2_BearingRemainToTarget);
  return c2_y;
}

static real32_T c2_i_emlrt_marshallIn(SFc2_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  real32_T c2_y;
  real32_T c2_f1;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_f1, 1, 1, 0U, 0, 0U, 0);
  c2_y = c2_f1;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_BearingRemainToTarget;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real32_T c2_y;
  SFc2_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc2_SimulationModelInstanceStruct *)chartInstanceVoid;
  c2_BearingRemainToTarget = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_i_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c2_BearingRemainToTarget), &c2_thisId);
  sf_mex_destroy(&c2_BearingRemainToTarget);
  *(real32_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_f_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i31;
  int16_T c2_b_inData[6];
  int32_T c2_i32;
  int16_T c2_u[6];
  const mxArray *c2_y = NULL;
  SFc2_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc2_SimulationModelInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i31 = 0; c2_i31 < 6; c2_i31++) {
    c2_b_inData[c2_i31] = (*(int16_T (*)[6])c2_inData)[c2_i31];
  }

  for (c2_i32 = 0; c2_i32 < 6; c2_i32++) {
    c2_u[c2_i32] = c2_b_inData[c2_i32];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 4, 0U, 1U, 0U, 1, 6), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static const mxArray *c2_g_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i33;
  int8_T c2_b_inData[6];
  int32_T c2_i34;
  int8_T c2_u[6];
  const mxArray *c2_y = NULL;
  SFc2_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc2_SimulationModelInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i33 = 0; c2_i33 < 6; c2_i33++) {
    c2_b_inData[c2_i33] = (*(int8_T (*)[6])c2_inData)[c2_i33];
  }

  for (c2_i34 = 0; c2_i34 < 6; c2_i34++) {
    c2_u[c2_i34] = c2_b_inData[c2_i34];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 2, 0U, 1U, 0U, 1, 6), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static const mxArray *c2_h_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int16_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc2_SimulationModelInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(int16_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 4, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static int16_T c2_j_emlrt_marshallIn(SFc2_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c2_b_dist_remaining, const char_T
  *c2_identifier)
{
  int16_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_k_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_dist_remaining),
    &c2_thisId);
  sf_mex_destroy(&c2_b_dist_remaining);
  return c2_y;
}

static int16_T c2_k_emlrt_marshallIn(SFc2_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  int16_T c2_y;
  int16_T c2_i35;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_i35, 1, 4, 0U, 0, 0U, 0);
  c2_y = c2_i35;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_dist_remaining;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  int16_T c2_y;
  SFc2_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc2_SimulationModelInstanceStruct *)chartInstanceVoid;
  c2_b_dist_remaining = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_k_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_dist_remaining),
    &c2_thisId);
  sf_mex_destroy(&c2_b_dist_remaining);
  *(int16_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_i_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i36;
  real_T c2_b_inData[2];
  int32_T c2_i37;
  real_T c2_u[2];
  const mxArray *c2_y = NULL;
  SFc2_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc2_SimulationModelInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i36 = 0; c2_i36 < 2; c2_i36++) {
    c2_b_inData[c2_i36] = (*(real_T (*)[2])c2_inData)[c2_i36];
  }

  for (c2_i37 = 0; c2_i37 < 2; c2_i37++) {
    c2_u[c2_i37] = c2_b_inData[c2_i37];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 1, 2), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_l_emlrt_marshallIn(SFc2_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[2])
{
  real_T c2_dv3[2];
  int32_T c2_i38;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv3, 1, 0, 0U, 1, 0U, 2, 1, 2);
  for (c2_i38 = 0; c2_i38 < 2; c2_i38++) {
    c2_y[c2_i38] = c2_dv3[c2_i38];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_startPos;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[2];
  int32_T c2_i39;
  SFc2_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc2_SimulationModelInstanceStruct *)chartInstanceVoid;
  c2_b_startPos = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_l_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_startPos), &c2_thisId,
                        c2_y);
  sf_mex_destroy(&c2_b_startPos);
  for (c2_i39 = 0; c2_i39 < 2; c2_i39++) {
    (*(real_T (*)[2])c2_outData)[c2_i39] = c2_y[c2_i39];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_j_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i40;
  int32_T c2_i41;
  int32_T c2_i42;
  real32_T c2_b_inData[16];
  int32_T c2_i43;
  int32_T c2_i44;
  int32_T c2_i45;
  real32_T c2_u[16];
  const mxArray *c2_y = NULL;
  SFc2_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc2_SimulationModelInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i40 = 0;
  for (c2_i41 = 0; c2_i41 < 2; c2_i41++) {
    for (c2_i42 = 0; c2_i42 < 8; c2_i42++) {
      c2_b_inData[c2_i42 + c2_i40] = (*(real32_T (*)[16])c2_inData)[c2_i42 +
        c2_i40];
    }

    c2_i40 += 8;
  }

  c2_i43 = 0;
  for (c2_i44 = 0; c2_i44 < 2; c2_i44++) {
    for (c2_i45 = 0; c2_i45 < 8; c2_i45++) {
      c2_u[c2_i45 + c2_i43] = c2_b_inData[c2_i45 + c2_i43];
    }

    c2_i43 += 8;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 1, 0U, 1U, 0U, 2, 8, 2), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static const mxArray *c2_k_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i46;
  real32_T c2_b_inData[2];
  int32_T c2_i47;
  real32_T c2_u[2];
  const mxArray *c2_y = NULL;
  SFc2_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc2_SimulationModelInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i46 = 0; c2_i46 < 2; c2_i46++) {
    c2_b_inData[c2_i46] = (*(real32_T (*)[2])c2_inData)[c2_i46];
  }

  for (c2_i47 = 0; c2_i47 < 2; c2_i47++) {
    c2_u[c2_i47] = c2_b_inData[c2_i47];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 1, 0U, 1U, 0U, 2, 1, 2), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_m_emlrt_marshallIn(SFc2_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c2_b_prev_target, const char_T *c2_identifier,
  real32_T c2_y[2])
{
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_n_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_prev_target), &c2_thisId,
                        c2_y);
  sf_mex_destroy(&c2_b_prev_target);
}

static void c2_n_emlrt_marshallIn(SFc2_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real32_T c2_y[2])
{
  real32_T c2_fv1[2];
  int32_T c2_i48;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_fv1, 1, 1, 0U, 1, 0U, 2, 1, 2);
  for (c2_i48 = 0; c2_i48 < 2; c2_i48++) {
    c2_y[c2_i48] = c2_fv1[c2_i48];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_prev_target;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real32_T c2_y[2];
  int32_T c2_i49;
  SFc2_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc2_SimulationModelInstanceStruct *)chartInstanceVoid;
  c2_b_prev_target = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_n_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_prev_target), &c2_thisId,
                        c2_y);
  sf_mex_destroy(&c2_b_prev_target);
  for (c2_i49 = 0; c2_i49 < 2; c2_i49++) {
    (*(real32_T (*)[2])c2_outData)[c2_i49] = c2_y[c2_i49];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_o_emlrt_marshallIn(SFc2_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c2_b_setSimStateSideEffectsInfo, const char_T
  *c2_identifier)
{
  const mxArray *c2_y = NULL;
  emlrtMsgIdentifier c2_thisId;
  c2_y = NULL;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  sf_mex_assign(&c2_y, c2_p_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c2_b_setSimStateSideEffectsInfo), &c2_thisId), false);
  sf_mex_destroy(&c2_b_setSimStateSideEffectsInfo);
  return c2_y;
}

static const mxArray *c2_p_emlrt_marshallIn(SFc2_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  const mxArray *c2_y = NULL;
  (void)chartInstance;
  (void)c2_parentId;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_duplicatearraysafe(&c2_u), false);
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_updateDataWrittenToVector(SFc2_SimulationModelInstanceStruct
  *chartInstance, uint32_T c2_vectorIndex)
{
  (void)chartInstance;
  c2_dataWrittenToVector[(uint32_T)_SFD_EML_ARRAY_BOUNDS_CHECK(0, (int32_T)
    c2_vectorIndex, 0, 18, 1, 0)] = true;
}

static void c2_errorIfDataNotWrittenToFcn(SFc2_SimulationModelInstanceStruct
  *chartInstance, uint32_T c2_vectorIndex, uint32_T c2_dataNumber)
{
  (void)chartInstance;
  _SFD_DATA_READ_BEFORE_WRITE_CHECK(c2_dataNumber, c2_dataWrittenToVector
    [(uint32_T)_SFD_EML_ARRAY_BOUNDS_CHECK(0, (int32_T)c2_vectorIndex, 0, 18, 1,
    0)]);
}

static void init_dsm_address_info(SFc2_SimulationModelInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

/* SFunction Glue Code */
#ifdef utFree
#undef utFree
#endif

#ifdef utMalloc
#undef utMalloc
#endif

#ifdef __cplusplus

extern "C" void *utMalloc(size_t size);
extern "C" void utFree(void*);

#else

extern void *utMalloc(size_t size);
extern void utFree(void*);

#endif

void sf_c2_SimulationModel_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1105560599U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3584600140U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2131270470U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2630997672U);
}

mxArray *sf_c2_SimulationModel_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("71uuUKZ6WmSGXBH0fUl7AD");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,6,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(8));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(8));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(6);
      pr[1] = (double)(1);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(6));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(6);
      pr[1] = (double)(1);
      mxSetField(mxData,3,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(4));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,3,"type",mxType);
    }

    mxSetField(mxData,3,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,4,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(1));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,4,"type",mxType);
    }

    mxSetField(mxData,4,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(8);
      pr[1] = (double)(2);
      mxSetField(mxData,5,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(9));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,5,"type",mxType);
    }

    mxSetField(mxData,5,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,6,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(9));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(9));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(9));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(2);
      mxSetField(mxData,3,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,3,"type",mxType);
    }

    mxSetField(mxData,3,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(2);
      mxSetField(mxData,4,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,4,"type",mxType);
    }

    mxSetField(mxData,4,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(2);
      mxSetField(mxData,5,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,5,"type",mxType);
    }

    mxSetField(mxData,5,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxData);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,5,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(9));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(9));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(1));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,3,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(1));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,3,"type",mxType);
    }

    mxSetField(mxData,3,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,4,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(1));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,4,"type",mxType);
    }

    mxSetField(mxData,4,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,14,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(3));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(3));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(8));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,3,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(3));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,3,"type",mxType);
    }

    mxSetField(mxData,3,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,4,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(9));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,4,"type",mxType);
    }

    mxSetField(mxData,4,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,5,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(9));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,5,"type",mxType);
    }

    mxSetField(mxData,5,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,6,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(9));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,6,"type",mxType);
    }

    mxSetField(mxData,6,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,7,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(9));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,7,"type",mxType);
    }

    mxSetField(mxData,7,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,8,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(6));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,8,"type",mxType);
    }

    mxSetField(mxData,8,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,9,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(8));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,9,"type",mxType);
    }

    mxSetField(mxData,9,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(2);
      mxSetField(mxData,10,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(9));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,10,"type",mxType);
    }

    mxSetField(mxData,10,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,11,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(9));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,11,"type",mxType);
    }

    mxSetField(mxData,11,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,12,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(9));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,12,"type",mxType);
    }

    mxSetField(mxData,12,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,13,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(9));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,13,"type",mxType);
    }

    mxSetField(mxData,13,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxData);
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c2_SimulationModel_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c2_SimulationModel_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

static const mxArray *sf_get_sim_state_info_c2_SimulationModel(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x10'type','srcId','name','auxInfo'{{M[1],M[141],T\"BearingRemainToTarget\",},{M[1],M[124],T\"DistanceRemainToTarget\",},{M[1],M[190],T\"allTargetsDone\",},{M[1],M[198],T\"doBeep\",},{M[1],M[199],T\"doScan\",},{M[3],M[183],T\"BearingToTarget\",},{M[3],M[158],T\"CurrentBearing\",},{M[3],M[156],T\"CurrentDistance\",},{M[3],M[184],T\"DistanceToTarget\",},{M[3],M[125],T\"TargetFound\",}}",
    "100 S1x10'type','srcId','name','auxInfo'{{M[3],M[131],T\"TargetIndex\",},{M[3],M[155],T\"dL\",},{M[3],M[154],T\"dR\",},{M[3],M[160],T\"dist_remaining\",},{M[3],M[161],T\"encoder_left_prev_target\",},{M[3],M[152],T\"encoder_right_prev_target\",},{M[3],M[153],T\"ind\",},{M[3],M[182],T\"prev_bearing\",},{M[3],M[181],T\"prev_target\",},{M[8],M[0],T\"is_active_c2_SimulationModel\",}}",
    "100 S1x3'type','srcId','name','auxInfo'{{M[9],M[0],T\"is_c2_SimulationModel\",},{M[9],M[3],T\"is_Running\",},{M[11],M[0],T\"temporalCounter_i1\",S'et','os','ct'{{T\"at\",M1x2[174 187],M[1]}}}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 23, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c2_SimulationModel_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc2_SimulationModelInstanceStruct *chartInstance;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    chartInstance = (SFc2_SimulationModelInstanceStruct *)
      chartInfo->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _SimulationModelMachineNumber_,
           2,
           8,
           10,
           0,
           31,
           0,
           0,
           0,
           0,
           0,
           &(chartInstance->chartNumber),
           &(chartInstance->instanceNumber),
           (void *)S);

        /* Each instance must initialize ist own list of scripts */
        init_script_number_translation(_SimulationModelMachineNumber_,
          chartInstance->chartNumber,chartInstance->instanceNumber);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          sf_debug_set_chart_disable_implicit_casting
            (sfGlobalDebugInstanceStruct,_SimulationModelMachineNumber_,
             chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(sfGlobalDebugInstanceStruct,
            _SimulationModelMachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,2,0,1,"BearingRemainToTarget");
          _SFD_SET_DATA_PROPS(1,0,0,0,"TargetIndex");
          _SFD_SET_DATA_PROPS(2,1,1,0,"EncoderLeft");
          _SFD_SET_DATA_PROPS(3,1,1,0,"EncoderRight");
          _SFD_SET_DATA_PROPS(4,2,0,1,"DistanceRemainToTarget");
          _SFD_SET_DATA_PROPS(5,1,1,0,"DistanceCameraTargets");
          _SFD_SET_DATA_PROPS(6,0,0,0,"TargetFound");
          _SFD_SET_DATA_PROPS(7,1,1,0,"BearingCameraTargets");
          _SFD_SET_DATA_PROPS(8,10,0,0,"AxleLength");
          _SFD_SET_DATA_PROPS(9,10,0,0,"WheelRadius");
          _SFD_SET_DATA_PROPS(10,10,0,0,"EncRes");
          _SFD_SET_DATA_PROPS(11,0,0,0,"encoder_right_prev_target");
          _SFD_SET_DATA_PROPS(12,0,0,0,"ind");
          _SFD_SET_DATA_PROPS(13,0,0,0,"dR");
          _SFD_SET_DATA_PROPS(14,0,0,0,"dL");
          _SFD_SET_DATA_PROPS(15,0,0,0,"CurrentDistance");
          _SFD_SET_DATA_PROPS(16,0,0,0,"CurrentBearing");
          _SFD_SET_DATA_PROPS(17,0,0,0,"dist_remaining");
          _SFD_SET_DATA_PROPS(18,0,0,0,"encoder_left_prev_target");
          _SFD_SET_DATA_PROPS(19,1,1,0,"Go");
          _SFD_SET_DATA_PROPS(20,10,0,0,"startPos");
          _SFD_SET_DATA_PROPS(21,1,1,0,"Targets");
          _SFD_SET_DATA_PROPS(22,0,0,0,"prev_target");
          _SFD_SET_DATA_PROPS(23,0,0,0,"prev_bearing");
          _SFD_SET_DATA_PROPS(24,0,0,0,"BearingToTarget");
          _SFD_SET_DATA_PROPS(25,0,0,0,"DistanceToTarget");
          _SFD_SET_DATA_PROPS(26,2,0,1,"allTargetsDone");
          _SFD_SET_DATA_PROPS(27,2,0,1,"doBeep");
          _SFD_SET_DATA_PROPS(28,2,0,1,"doScan");
          _SFD_SET_DATA_PROPS(29,10,0,0,"pcam");
          _SFD_SET_DATA_PROPS(30,10,0,0,"lcam");
          _SFD_STATE_INFO(0,0,0);
          _SFD_STATE_INFO(1,0,0);
          _SFD_STATE_INFO(2,0,0);
          _SFD_STATE_INFO(3,0,0);
          _SFD_STATE_INFO(4,0,0);
          _SFD_STATE_INFO(5,0,0);
          _SFD_STATE_INFO(6,0,0);
          _SFD_STATE_INFO(7,0,0);
          _SFD_CH_SUBSTATE_COUNT(4);
          _SFD_CH_SUBSTATE_DECOMP(0);
          _SFD_CH_SUBSTATE_INDEX(0,0);
          _SFD_CH_SUBSTATE_INDEX(1,1);
          _SFD_CH_SUBSTATE_INDEX(2,2);
          _SFD_CH_SUBSTATE_INDEX(3,7);
          _SFD_ST_SUBSTATE_COUNT(0,0);
          _SFD_ST_SUBSTATE_COUNT(1,0);
          _SFD_ST_SUBSTATE_COUNT(2,4);
          _SFD_ST_SUBSTATE_INDEX(2,0,3);
          _SFD_ST_SUBSTATE_INDEX(2,1,4);
          _SFD_ST_SUBSTATE_INDEX(2,2,5);
          _SFD_ST_SUBSTATE_INDEX(2,3,6);
          _SFD_ST_SUBSTATE_COUNT(3,0);
          _SFD_ST_SUBSTATE_COUNT(4,0);
          _SFD_ST_SUBSTATE_COUNT(5,0);
          _SFD_ST_SUBSTATE_COUNT(6,0);
          _SFD_ST_SUBSTATE_COUNT(7,0);
        }

        _SFD_CV_INIT_CHART(4,1,0,0);

        {
          _SFD_CV_INIT_STATE(0,0,0,0,0,0,NULL,NULL);
        }

        {
          _SFD_CV_INIT_STATE(1,0,0,0,0,0,NULL,NULL);
        }

        {
          _SFD_CV_INIT_STATE(2,4,1,1,0,0,NULL,NULL);
        }

        {
          _SFD_CV_INIT_STATE(3,0,0,0,0,0,NULL,NULL);
        }

        {
          _SFD_CV_INIT_STATE(4,0,0,0,0,0,NULL,NULL);
        }

        {
          _SFD_CV_INIT_STATE(5,0,0,0,0,0,NULL,NULL);
        }

        {
          _SFD_CV_INIT_STATE(6,0,0,0,0,0,NULL,NULL);
        }

        {
          _SFD_CV_INIT_STATE(7,0,0,0,0,0,NULL,NULL);
        }

        _SFD_CV_INIT_TRANS(2,0,NULL,NULL,0,NULL);
        _SFD_CV_INIT_TRANS(5,0,NULL,NULL,0,NULL);
        _SFD_CV_INIT_TRANS(4,0,NULL,NULL,0,NULL);
        _SFD_CV_INIT_TRANS(3,0,NULL,NULL,0,NULL);
        _SFD_CV_INIT_TRANS(6,0,NULL,NULL,0,NULL);
        _SFD_CV_INIT_TRANS(7,0,NULL,NULL,0,NULL);
        _SFD_CV_INIT_TRANS(1,0,NULL,NULL,0,NULL);
        _SFD_CV_INIT_TRANS(0,0,NULL,NULL,0,NULL);
        _SFD_CV_INIT_TRANS(8,0,NULL,NULL,0,NULL);
        _SFD_CV_INIT_TRANS(9,0,NULL,NULL,0,NULL);

        /* Initialization of MATLAB Function Model Coverage */
        _SFD_CV_INIT_EML(7,1,0,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML(5,1,0,0,0,1,0,0,0,0,0);
        _SFD_CV_INIT_EML_SATURATION(5,1,0,53,-1,68);
        _SFD_CV_INIT_EML(3,1,0,4,0,7,0,1,0,2,1);
        _SFD_CV_INIT_EML_SATURATION(3,1,0,156,-1,185);
        _SFD_CV_INIT_EML_SATURATION(3,1,1,301,-1,349);
        _SFD_CV_INIT_EML_SATURATION(3,1,2,305,-1,348);
        _SFD_CV_INIT_EML_SATURATION(3,1,3,397,-1,483);
        _SFD_CV_INIT_EML_SATURATION(3,1,4,401,-1,482);
        _SFD_CV_INIT_EML_SATURATION(3,1,5,486,-1,534);
        _SFD_CV_INIT_EML_SATURATION(3,1,6,490,-1,533);
        _SFD_CV_INIT_EML_IF(3,1,0,248,282,-1,633);
        _SFD_CV_INIT_EML_IF(3,1,1,298,354,-1,620);
        _SFD_CV_INIT_EML_IF(3,1,2,373,534,-1,603);
        _SFD_CV_INIT_EML_IF(3,1,3,723,744,-1,1023);
        _SFD_CV_INIT_EML_FOR(3,1,0,192,239,642);

        {
          static int condStart[] = { 377, 397 };

          static int condEnd[] = { 393, 534 };

          static int pfixExpr[] = { 0, 1, -2 };

          _SFD_CV_INIT_EML_MCDC(3,1,0,377,534,2,0,&(condStart[0]),&(condEnd[0]),
                                3,&(pfixExpr[0]));
        }

        _SFD_CV_INIT_EML(6,1,0,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML(4,1,0,0,0,2,0,0,0,0,0);
        _SFD_CV_INIT_EML_SATURATION(4,1,0,139,-1,177);
        _SFD_CV_INIT_EML_SATURATION(4,1,1,229,-1,269);
        _SFD_CV_INIT_EML(0,1,0,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML(1,1,0,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML(4,0,0,1,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_IF(4,0,0,0,12,0,12);
        _SFD_CV_INIT_EML(3,0,0,1,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_IF(3,0,0,1,27,1,27);
        _SFD_CV_INIT_EML(6,0,0,1,0,0,0,0,0,2,1);
        _SFD_CV_INIT_EML_IF(6,0,0,1,58,1,48);

        {
          static int condStart[] = { 1, 24 };

          static int condEnd[] = { 20, 58 };

          static int pfixExpr[] = { 0, 1, -3 };

          _SFD_CV_INIT_EML_MCDC(6,0,0,1,58,2,0,&(condStart[0]),&(condEnd[0]),3,
                                &(pfixExpr[0]));
        }

        _SFD_CV_INIT_EML(0,0,0,1,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_IF(0,0,0,1,11,1,11);
        _SFD_CV_INIT_EML(8,0,0,1,0,0,0,0,0,2,1);
        _SFD_CV_INIT_EML_IF(8,0,0,1,61,1,48);

        {
          static int condStart[] = { 1, 31 };

          static int condEnd[] = { 27, 61 };

          static int pfixExpr[] = { 0, 1, -3 };

          _SFD_CV_INIT_EML_MCDC(8,0,0,1,61,2,0,&(condStart[0]),&(condEnd[0]),3,
                                &(pfixExpr[0]));
        }

        _SFD_CV_INIT_EML(9,0,0,1,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_IF(9,0,0,0,12,0,12);
        _SFD_SET_DATA_COMPILED_PROPS(0,SF_SINGLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_e_sf_marshallOut,(MexInFcnForType)c2_e_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(1,SF_UINT8,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_d_sf_marshallOut,(MexInFcnForType)c2_d_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(2,SF_INT32,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_c_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(3,SF_INT32,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_c_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(4,SF_SINGLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_e_sf_marshallOut,(MexInFcnForType)c2_e_sf_marshallIn);

        {
          unsigned int dimVector[1];
          dimVector[0]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(5,SF_INT16,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_f_sf_marshallOut,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(6,SF_UINT8,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_d_sf_marshallOut,(MexInFcnForType)c2_d_sf_marshallIn);

        {
          unsigned int dimVector[1];
          dimVector[0]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(7,SF_INT8,1,&(dimVector[0]),0,0,0,0.0,1.0,
            0,0,(MexFcnForType)c2_g_sf_marshallOut,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(8,SF_SINGLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_e_sf_marshallOut,(MexInFcnForType)c2_e_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(9,SF_SINGLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_e_sf_marshallOut,(MexInFcnForType)c2_e_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(10,SF_SINGLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_e_sf_marshallOut,(MexInFcnForType)c2_e_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(11,SF_INT32,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_c_sf_marshallOut,(MexInFcnForType)c2_c_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(12,SF_UINT8,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_d_sf_marshallOut,(MexInFcnForType)c2_d_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(13,SF_SINGLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_e_sf_marshallOut,(MexInFcnForType)c2_e_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(14,SF_SINGLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_e_sf_marshallOut,(MexInFcnForType)c2_e_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(15,SF_SINGLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_e_sf_marshallOut,(MexInFcnForType)c2_e_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(16,SF_SINGLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_e_sf_marshallOut,(MexInFcnForType)c2_e_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(17,SF_INT16,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_h_sf_marshallOut,(MexInFcnForType)c2_f_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(18,SF_INT32,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_c_sf_marshallOut,(MexInFcnForType)c2_c_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(19,SF_UINT8,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_b_sf_marshallOut,(MexInFcnForType)NULL);

        {
          unsigned int dimVector[2];
          dimVector[0]= 1;
          dimVector[1]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(20,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_i_sf_marshallOut,(MexInFcnForType)
            c2_g_sf_marshallIn);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 8;
          dimVector[1]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(21,SF_SINGLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_j_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 1;
          dimVector[1]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(22,SF_SINGLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_k_sf_marshallOut,(MexInFcnForType)
            c2_h_sf_marshallIn);
        }

        _SFD_SET_DATA_COMPILED_PROPS(23,SF_SINGLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_e_sf_marshallOut,(MexInFcnForType)c2_e_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(24,SF_SINGLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_e_sf_marshallOut,(MexInFcnForType)c2_e_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(25,SF_SINGLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_e_sf_marshallOut,(MexInFcnForType)c2_e_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(26,SF_UINT8,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_b_sf_marshallOut,(MexInFcnForType)c2_b_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(27,SF_UINT8,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_b_sf_marshallOut,(MexInFcnForType)c2_b_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(28,SF_UINT8,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_b_sf_marshallOut,(MexInFcnForType)c2_b_sf_marshallIn);

        {
          unsigned int dimVector[2];
          dimVector[0]= 1;
          dimVector[1]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(29,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_i_sf_marshallOut,(MexInFcnForType)
            c2_g_sf_marshallIn);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 1;
          dimVector[1]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(30,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_i_sf_marshallOut,(MexInFcnForType)
            c2_g_sf_marshallIn);
        }

        {
          real32_T *c2_BearingRemainToTarget;
          int32_T *c2_EncoderLeft;
          int32_T *c2_EncoderRight;
          real32_T *c2_DistanceRemainToTarget;
          boolean_T *c2_Go;
          boolean_T *c2_allTargetsDone;
          boolean_T *c2_doBeep;
          boolean_T *c2_doScan;
          int16_T (*c2_DistanceCameraTargets)[6];
          int8_T (*c2_BearingCameraTargets)[6];
          real32_T (*c2_Targets)[16];
          c2_doScan = (boolean_T *)ssGetOutputPortSignal(chartInstance->S, 5);
          c2_doBeep = (boolean_T *)ssGetOutputPortSignal(chartInstance->S, 4);
          c2_allTargetsDone = (boolean_T *)ssGetOutputPortSignal
            (chartInstance->S, 3);
          c2_Targets = (real32_T (*)[16])ssGetInputPortSignal(chartInstance->S,
            5);
          c2_Go = (boolean_T *)ssGetInputPortSignal(chartInstance->S, 4);
          c2_BearingCameraTargets = (int8_T (*)[6])ssGetInputPortSignal
            (chartInstance->S, 3);
          c2_DistanceCameraTargets = (int16_T (*)[6])ssGetInputPortSignal
            (chartInstance->S, 2);
          c2_DistanceRemainToTarget = (real32_T *)ssGetOutputPortSignal
            (chartInstance->S, 2);
          c2_EncoderRight = (int32_T *)ssGetInputPortSignal(chartInstance->S, 1);
          c2_EncoderLeft = (int32_T *)ssGetInputPortSignal(chartInstance->S, 0);
          c2_BearingRemainToTarget = (real32_T *)ssGetOutputPortSignal
            (chartInstance->S, 1);
          _SFD_SET_DATA_VALUE_PTR(0U, c2_BearingRemainToTarget);
          _SFD_SET_DATA_VALUE_PTR(1U, &chartInstance->c2_TargetIndex);
          _SFD_SET_DATA_VALUE_PTR(2U, c2_EncoderLeft);
          _SFD_SET_DATA_VALUE_PTR(3U, c2_EncoderRight);
          _SFD_SET_DATA_VALUE_PTR(4U, c2_DistanceRemainToTarget);
          _SFD_SET_DATA_VALUE_PTR(5U, *c2_DistanceCameraTargets);
          _SFD_SET_DATA_VALUE_PTR(6U, &chartInstance->c2_TargetFound);
          _SFD_SET_DATA_VALUE_PTR(7U, *c2_BearingCameraTargets);
          _SFD_SET_DATA_VALUE_PTR(8U, &chartInstance->c2_AxleLength);
          _SFD_SET_DATA_VALUE_PTR(9U, &chartInstance->c2_WheelRadius);
          _SFD_SET_DATA_VALUE_PTR(10U, &chartInstance->c2_EncRes);
          _SFD_SET_DATA_VALUE_PTR(11U,
            &chartInstance->c2_encoder_right_prev_target);
          _SFD_SET_DATA_VALUE_PTR(12U, &chartInstance->c2_ind);
          _SFD_SET_DATA_VALUE_PTR(13U, &chartInstance->c2_dR);
          _SFD_SET_DATA_VALUE_PTR(14U, &chartInstance->c2_dL);
          _SFD_SET_DATA_VALUE_PTR(15U, &chartInstance->c2_CurrentDistance);
          _SFD_SET_DATA_VALUE_PTR(16U, &chartInstance->c2_CurrentBearing);
          _SFD_SET_DATA_VALUE_PTR(17U, &chartInstance->c2_dist_remaining);
          _SFD_SET_DATA_VALUE_PTR(18U,
            &chartInstance->c2_encoder_left_prev_target);
          _SFD_SET_DATA_VALUE_PTR(19U, c2_Go);
          _SFD_SET_DATA_VALUE_PTR(20U, chartInstance->c2_startPos);
          _SFD_SET_DATA_VALUE_PTR(21U, *c2_Targets);
          _SFD_SET_DATA_VALUE_PTR(22U, chartInstance->c2_prev_target);
          _SFD_SET_DATA_VALUE_PTR(23U, &chartInstance->c2_prev_bearing);
          _SFD_SET_DATA_VALUE_PTR(24U, &chartInstance->c2_BearingToTarget);
          _SFD_SET_DATA_VALUE_PTR(25U, &chartInstance->c2_DistanceToTarget);
          _SFD_SET_DATA_VALUE_PTR(26U, c2_allTargetsDone);
          _SFD_SET_DATA_VALUE_PTR(27U, c2_doBeep);
          _SFD_SET_DATA_VALUE_PTR(28U, c2_doScan);
          _SFD_SET_DATA_VALUE_PTR(29U, chartInstance->c2_pcam);
          _SFD_SET_DATA_VALUE_PTR(30U, chartInstance->c2_lcam);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration(sfGlobalDebugInstanceStruct,
        _SimulationModelMachineNumber_,chartInstance->chartNumber,
        chartInstance->instanceNumber);
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "VxQPghVCdUExcSIgqE9kyC";
}

static void sf_opaque_initialize_c2_SimulationModel(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc2_SimulationModelInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c2_SimulationModel((SFc2_SimulationModelInstanceStruct*)
    chartInstanceVar);
  initialize_c2_SimulationModel((SFc2_SimulationModelInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c2_SimulationModel(void *chartInstanceVar)
{
  enable_c2_SimulationModel((SFc2_SimulationModelInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c2_SimulationModel(void *chartInstanceVar)
{
  disable_c2_SimulationModel((SFc2_SimulationModelInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c2_SimulationModel(void *chartInstanceVar)
{
  sf_gateway_c2_SimulationModel((SFc2_SimulationModelInstanceStruct*)
    chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c2_SimulationModel(SimStruct* S)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c2_SimulationModel
    ((SFc2_SimulationModelInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c2_SimulationModel();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_raw2high'.\n");
  }

  return plhs[0];
}

extern void sf_internal_set_sim_state_c2_SimulationModel(SimStruct* S, const
  mxArray *st)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[3];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxDuplicateArray(st);      /* high level simctx */
  prhs[2] = (mxArray*) sf_get_sim_state_info_c2_SimulationModel();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 3, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c2_SimulationModel((SFc2_SimulationModelInstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c2_SimulationModel(SimStruct* S)
{
  return sf_internal_get_sim_state_c2_SimulationModel(S);
}

static void sf_opaque_set_sim_state_c2_SimulationModel(SimStruct* S, const
  mxArray *st)
{
  sf_internal_set_sim_state_c2_SimulationModel(S, st);
}

static void sf_opaque_terminate_c2_SimulationModel(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc2_SimulationModelInstanceStruct*) chartInstanceVar)->S;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_SimulationModel_optimization_info();
    }

    finalize_c2_SimulationModel((SFc2_SimulationModelInstanceStruct*)
      chartInstanceVar);
    utFree((void *)chartInstanceVar);
    if (crtInfo != NULL) {
      utFree((void *)crtInfo);
    }

    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc2_SimulationModel((SFc2_SimulationModelInstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c2_SimulationModel(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    initialize_params_c2_SimulationModel((SFc2_SimulationModelInstanceStruct*)
      (chartInfo->chartInstance));
  }
}

static void mdlSetWorkWidths_c2_SimulationModel(SimStruct *S)
{
  /* Actual parameters from chart:
     AxleLength EncRes WheelRadius lcam pcam startPos
   */
  const char_T *rtParamNames[] = { "AxleLength", "EncRes", "WheelRadius", "lcam",
    "pcam", "startPos" };

  ssSetNumRunTimeParams(S,ssGetSFcnParamsCount(S));

  /* registration for AxleLength*/
  ssRegDlgParamAsRunTimeParam(S, 0, 0, rtParamNames[0], SS_SINGLE);

  /* registration for EncRes*/
  ssRegDlgParamAsRunTimeParam(S, 1, 1, rtParamNames[1], SS_SINGLE);

  /* registration for WheelRadius*/
  ssRegDlgParamAsRunTimeParam(S, 2, 2, rtParamNames[2], SS_SINGLE);

  /* registration for lcam*/
  ssRegDlgParamAsRunTimeParam(S, 3, 3, rtParamNames[3], SS_DOUBLE);

  /* registration for pcam*/
  ssRegDlgParamAsRunTimeParam(S, 4, 4, rtParamNames[4], SS_DOUBLE);

  /* registration for startPos*/
  ssRegDlgParamAsRunTimeParam(S, 5, 5, rtParamNames[5], SS_DOUBLE);
  ssSetModelReferenceSampleTimeDisallowInheritance(S);
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_SimulationModel_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,2);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(sf_get_instance_specialization(),
                infoStruct,2,"RTWCG"));
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,2,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(sf_get_instance_specialization(),infoStruct,2);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 4, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 5, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,2,6);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,2,5);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=5; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 6; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,2);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(3164328687U));
  ssSetChecksum1(S,(698319931U));
  ssSetChecksum2(S,(774429634U));
  ssSetChecksum3(S,(4038583264U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c2_SimulationModel(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Stateflow");
  }
}

static void mdlStart_c2_SimulationModel(SimStruct *S)
{
  SFc2_SimulationModelInstanceStruct *chartInstance;
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)utMalloc(sizeof
    (ChartRunTimeInfo));
  chartInstance = (SFc2_SimulationModelInstanceStruct *)utMalloc(sizeof
    (SFc2_SimulationModelInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc2_SimulationModelInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 0;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c2_SimulationModel;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c2_SimulationModel;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c2_SimulationModel;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c2_SimulationModel;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c2_SimulationModel;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c2_SimulationModel;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c2_SimulationModel;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c2_SimulationModel;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c2_SimulationModel;
  chartInstance->chartInfo.mdlStart = mdlStart_c2_SimulationModel;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c2_SimulationModel;
  chartInstance->chartInfo.extModeExec = NULL;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->chartInfo.debugInstance = sfGlobalDebugInstanceStruct;
  chartInstance->S = S;
  crtInfo->instanceInfo = (&(chartInstance->chartInfo));
  crtInfo->isJITEnabled = false;
  ssSetUserData(S,(void *)(crtInfo));  /* register the chart instance with simstruct */
  init_dsm_address_info(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  sf_opaque_init_subchart_simstructs(chartInstance->chartInfo.chartInstance);
  chart_debug_initialization(S,1);
}

void c2_SimulationModel_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c2_SimulationModel(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c2_SimulationModel(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c2_SimulationModel(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c2_SimulationModel_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
