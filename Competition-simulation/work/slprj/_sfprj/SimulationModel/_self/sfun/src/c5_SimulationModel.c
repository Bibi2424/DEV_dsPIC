/* Include files */

#include <stddef.h>
#include "blas.h"
#include "SimulationModel_sfun.h"
#include "c5_SimulationModel.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "SimulationModel_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(sfGlobalDebugInstanceStruct,S);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)
#define c5_IN_NO_ACTIVE_CHILD          ((uint8_T)0U)
#define c5_IN_Finished                 ((uint8_T)1U)
#define c5_IN_Running                  ((uint8_T)2U)
#define c5_IN_Standby                  ((uint8_T)3U)
#define c5_IN_NearLocation             ((uint8_T)1U)
#define c5_IN_Roaming                  ((uint8_T)2U)

/* Variable Declarations */

/* Variable Definitions */
static real_T _sfTime_;
static const char * c5_debug_family_names[2] = { "nargin", "nargout" };

static const char * c5_b_debug_family_names[2] = { "nargin", "nargout" };

static const char * c5_c_debug_family_names[2] = { "nargin", "nargout" };

static const char * c5_d_debug_family_names[2] = { "nargin", "nargout" };

static const char * c5_e_debug_family_names[4] = { "lt", "nargin", "nargout",
  "et" };

static const char * c5_f_debug_family_names[9] = { "delta", "dist", "nargin",
  "nargout", "robotPos", "targetPos", "proximityVal", "nearLoc", "currentLoc" };

static const char * c5_g_debug_family_names[2] = { "nargin", "nargout" };

static const char * c5_h_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c5_i_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c5_j_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c5_k_debug_family_names[2] = { "nargin", "nargout" };

static const char * c5_l_debug_family_names[8] = { "nargin", "nargout",
  "tfound_in", "currentT", "lastTime_in", "time", "tfound_out", "lastTime_out" };

static const char * c5_m_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c5_n_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c5_o_debug_family_names[5] = { "nargin", "nargout", "pos",
  "x0", "d" };

static const char * c5_p_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c5_q_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c5_r_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c5_s_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c5_t_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c5_u_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c5_v_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c5_w_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static const char * c5_x_debug_family_names[3] = { "nargin", "nargout",
  "sf_internal_predicateOutput" };

static boolean_T c5_dataWrittenToVector[19];

/* Function Declarations */
static void initialize_c5_SimulationModel(SFc5_SimulationModelInstanceStruct
  *chartInstance);
static void initialize_params_c5_SimulationModel
  (SFc5_SimulationModelInstanceStruct *chartInstance);
static void enable_c5_SimulationModel(SFc5_SimulationModelInstanceStruct
  *chartInstance);
static void disable_c5_SimulationModel(SFc5_SimulationModelInstanceStruct
  *chartInstance);
static void c5_update_debugger_state_c5_SimulationModel
  (SFc5_SimulationModelInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c5_SimulationModel
  (SFc5_SimulationModelInstanceStruct *chartInstance);
static void set_sim_state_c5_SimulationModel(SFc5_SimulationModelInstanceStruct *
  chartInstance, const mxArray *c5_st);
static void c5_set_sim_state_side_effects_c5_SimulationModel
  (SFc5_SimulationModelInstanceStruct *chartInstance);
static void finalize_c5_SimulationModel(SFc5_SimulationModelInstanceStruct
  *chartInstance);
static void sf_gateway_c5_SimulationModel(SFc5_SimulationModelInstanceStruct
  *chartInstance);
static void initSimStructsc5_SimulationModel(SFc5_SimulationModelInstanceStruct *
  chartInstance);
static void c5_enter_atomic_Standby(SFc5_SimulationModelInstanceStruct
  *chartInstance);
static void c5_Running(SFc5_SimulationModelInstanceStruct *chartInstance);
static void c5_exit_internal_Running(SFc5_SimulationModelInstanceStruct
  *chartInstance);
static void c5_bsxfun(SFc5_SimulationModelInstanceStruct *chartInstance,
                      real32_T c5_a[16], real_T c5_b[2], real32_T c5_c[16]);
static void c5_eml_scalar_eg(SFc5_SimulationModelInstanceStruct *chartInstance);
static boolean_T c5_any(SFc5_SimulationModelInstanceStruct *chartInstance,
  boolean_T c5_x[8]);
static void c5_isequal(SFc5_SimulationModelInstanceStruct *chartInstance);
static void c5_eml_error(SFc5_SimulationModelInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c5_machineNumber, uint32_T
  c5_chartNumber, uint32_T c5_instanceNumber);
static const mxArray *c5_sf_marshallOut(void *chartInstanceVoid, void *c5_inData);
static real_T c5_emlrt_marshallIn(SFc5_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c5_nargout, const char_T *c5_identifier);
static real_T c5_b_emlrt_marshallIn(SFc5_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId);
static void c5_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData);
static const mxArray *c5_b_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData);
static void c5_c_emlrt_marshallIn(SFc5_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c5_currentLoc, const char_T *c5_identifier,
  boolean_T c5_y[8]);
static void c5_d_emlrt_marshallIn(SFc5_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId,
  boolean_T c5_y[8]);
static void c5_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData);
static const mxArray *c5_c_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData);
static boolean_T c5_e_emlrt_marshallIn(SFc5_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c5_nearLoc, const char_T *c5_identifier);
static boolean_T c5_f_emlrt_marshallIn(SFc5_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId);
static void c5_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData);
static const mxArray *c5_d_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData);
static void c5_g_emlrt_marshallIn(SFc5_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId,
  real32_T c5_y[16]);
static void c5_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData);
static const mxArray *c5_e_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData);
static void c5_h_emlrt_marshallIn(SFc5_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId,
  real_T c5_y[2]);
static void c5_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData);
static const mxArray *c5_f_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData);
static void c5_i_emlrt_marshallIn(SFc5_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId,
  real32_T c5_y[8]);
static void c5_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData);
static void c5_info_helper(const mxArray **c5_info);
static const mxArray *c5_emlrt_marshallOut(const char * c5_u);
static const mxArray *c5_b_emlrt_marshallOut(const uint32_T c5_u);
static void c5_b_info_helper(const mxArray **c5_info);
static void c5_startTiming(SFc5_SimulationModelInstanceStruct *chartInstance);
static real_T c5_getTiming(SFc5_SimulationModelInstanceStruct *chartInstance);
static void c5_testIfNearLocation(SFc5_SimulationModelInstanceStruct
  *chartInstance, real_T c5_robotPos[2], real32_T c5_targetPos[16], real_T
  c5_proximityVal, boolean_T *c5_nearLoc, boolean_T c5_currentLoc[8]);
static void c5_updateTargetsFound(SFc5_SimulationModelInstanceStruct
  *chartInstance, boolean_T c5_tfound_in[8], boolean_T c5_currentT[8], real_T
  c5_lastTime_in, real_T c5_time, boolean_T c5_tfound_out[8], real_T
  *c5_lastTime_out);
static real_T c5_distancefrom(SFc5_SimulationModelInstanceStruct *chartInstance,
  real_T c5_pos[2], real_T c5_x0[2]);
static const mxArray *c5_g_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData);
static int32_T c5_j_emlrt_marshallIn(SFc5_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId);
static void c5_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData);
static const mxArray *c5_h_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData);
static uint8_T c5_k_emlrt_marshallIn(SFc5_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c5_b_tp_Standby, const char_T *c5_identifier);
static uint8_T c5_l_emlrt_marshallIn(SFc5_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId);
static void c5_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData);
static const mxArray *c5_m_emlrt_marshallIn(SFc5_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c5_b_setSimStateSideEffectsInfo, const char_T
  *c5_identifier);
static const mxArray *c5_n_emlrt_marshallIn(SFc5_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId);
static void c5_updateDataWrittenToVector(SFc5_SimulationModelInstanceStruct
  *chartInstance, uint32_T c5_vectorIndex);
static void c5_errorIfDataNotWrittenToFcn(SFc5_SimulationModelInstanceStruct
  *chartInstance, uint32_T c5_vectorIndex, uint32_T c5_dataNumber);
static void init_dsm_address_info(SFc5_SimulationModelInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c5_SimulationModel(SFc5_SimulationModelInstanceStruct
  *chartInstance)
{
  chartInstance->c5_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c5_doSetSimStateSideEffects = 0U;
  chartInstance->c5_setSimStateSideEffectsInfo = NULL;
  chartInstance->c5_tp_Finished = 0U;
  chartInstance->c5_is_Running = c5_IN_NO_ACTIVE_CHILD;
  chartInstance->c5_tp_Running = 0U;
  chartInstance->c5_tp_NearLocation = 0U;
  chartInstance->c5_tp_Roaming = 0U;
  chartInstance->c5_tp_Standby = 0U;
  chartInstance->c5_is_active_c5_SimulationModel = 0U;
  chartInstance->c5_is_c5_SimulationModel = c5_IN_NO_ACTIVE_CHILD;
}

static void initialize_params_c5_SimulationModel
  (SFc5_SimulationModelInstanceStruct *chartInstance)
{
  real_T c5_dv0[2];
  int32_T c5_i0;
  sf_mex_import_named("startPos", sf_mex_get_sfun_param(chartInstance->S, 0, 0),
                      c5_dv0, 0, 0, 0U, 1, 0U, 2, 1, 2);
  for (c5_i0 = 0; c5_i0 < 2; c5_i0++) {
    chartInstance->c5_startPos[c5_i0] = c5_dv0[c5_i0];
  }
}

static void enable_c5_SimulationModel(SFc5_SimulationModelInstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c5_SimulationModel(SFc5_SimulationModelInstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c5_update_debugger_state_c5_SimulationModel
  (SFc5_SimulationModelInstanceStruct *chartInstance)
{
  uint32_T c5_prevAniVal;
  c5_prevAniVal = _SFD_GET_ANIMATION();
  _SFD_SET_ANIMATION(0U);
  _SFD_SET_HONOR_BREAKPOINTS(0U);
  if (chartInstance->c5_is_active_c5_SimulationModel == 1U) {
    _SFD_CC_CALL(CHART_ACTIVE_TAG, 1U, chartInstance->c5_sfEvent);
  }

  if (chartInstance->c5_is_c5_SimulationModel == c5_IN_Standby) {
    _SFD_CS_CALL(STATE_ACTIVE_TAG, 4U, chartInstance->c5_sfEvent);
  } else {
    _SFD_CS_CALL(STATE_INACTIVE_TAG, 4U, chartInstance->c5_sfEvent);
  }

  if (chartInstance->c5_is_c5_SimulationModel == c5_IN_Running) {
    _SFD_CS_CALL(STATE_ACTIVE_TAG, 1U, chartInstance->c5_sfEvent);
  } else {
    _SFD_CS_CALL(STATE_INACTIVE_TAG, 1U, chartInstance->c5_sfEvent);
  }

  if (chartInstance->c5_is_Running == c5_IN_NearLocation) {
    _SFD_CS_CALL(STATE_ACTIVE_TAG, 2U, chartInstance->c5_sfEvent);
  } else {
    _SFD_CS_CALL(STATE_INACTIVE_TAG, 2U, chartInstance->c5_sfEvent);
  }

  if (chartInstance->c5_is_Running == c5_IN_Roaming) {
    _SFD_CS_CALL(STATE_ACTIVE_TAG, 3U, chartInstance->c5_sfEvent);
  } else {
    _SFD_CS_CALL(STATE_INACTIVE_TAG, 3U, chartInstance->c5_sfEvent);
  }

  if (chartInstance->c5_is_c5_SimulationModel == c5_IN_Finished) {
    _SFD_CS_CALL(STATE_ACTIVE_TAG, 0U, chartInstance->c5_sfEvent);
  } else {
    _SFD_CS_CALL(STATE_INACTIVE_TAG, 0U, chartInstance->c5_sfEvent);
  }

  _SFD_SET_ANIMATION(c5_prevAniVal);
  _SFD_SET_HONOR_BREAKPOINTS(1U);
  _SFD_ANIMATE();
}

static const mxArray *get_sim_state_c5_SimulationModel
  (SFc5_SimulationModelInstanceStruct *chartInstance)
{
  const mxArray *c5_st;
  const mxArray *c5_y = NULL;
  real_T c5_hoistedGlobal;
  real_T c5_u;
  const mxArray *c5_b_y = NULL;
  int32_T c5_i1;
  boolean_T c5_b_u[8];
  const mxArray *c5_c_y = NULL;
  real_T c5_b_hoistedGlobal;
  real_T c5_c_u;
  const mxArray *c5_d_y = NULL;
  real_T c5_c_hoistedGlobal;
  real_T c5_d_u;
  const mxArray *c5_e_y = NULL;
  int32_T c5_i2;
  boolean_T c5_e_u[8];
  const mxArray *c5_f_y = NULL;
  real_T c5_d_hoistedGlobal;
  real_T c5_f_u;
  const mxArray *c5_g_y = NULL;
  boolean_T c5_e_hoistedGlobal;
  boolean_T c5_g_u;
  const mxArray *c5_h_y = NULL;
  real_T c5_f_hoistedGlobal;
  real_T c5_h_u;
  const mxArray *c5_i_y = NULL;
  real_T c5_g_hoistedGlobal;
  real_T c5_i_u;
  const mxArray *c5_j_y = NULL;
  real_T c5_h_hoistedGlobal;
  real_T c5_j_u;
  const mxArray *c5_k_y = NULL;
  real_T c5_i_hoistedGlobal;
  real_T c5_k_u;
  const mxArray *c5_l_y = NULL;
  real_T c5_j_hoistedGlobal;
  real_T c5_l_u;
  const mxArray *c5_m_y = NULL;
  real_T c5_k_hoistedGlobal;
  real_T c5_m_u;
  const mxArray *c5_n_y = NULL;
  uint8_T c5_l_hoistedGlobal;
  uint8_T c5_n_u;
  const mxArray *c5_o_y = NULL;
  uint8_T c5_m_hoistedGlobal;
  uint8_T c5_o_u;
  const mxArray *c5_p_y = NULL;
  uint8_T c5_n_hoistedGlobal;
  uint8_T c5_p_u;
  const mxArray *c5_q_y = NULL;
  real_T *c5_elapsedTime;
  real_T *c5_timeLastTargetFound;
  boolean_T (*c5_targetsFound)[8];
  c5_targetsFound = (boolean_T (*)[8])ssGetOutputPortSignal(chartInstance->S, 3);
  c5_elapsedTime = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c5_timeLastTargetFound = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c5_st = NULL;
  c5_st = NULL;
  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_createcellmatrix(16, 1), false);
  c5_hoistedGlobal = *c5_elapsedTime;
  c5_u = c5_hoistedGlobal;
  c5_b_y = NULL;
  sf_mex_assign(&c5_b_y, sf_mex_create("y", &c5_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c5_y, 0, c5_b_y);
  for (c5_i1 = 0; c5_i1 < 8; c5_i1++) {
    c5_b_u[c5_i1] = (*c5_targetsFound)[c5_i1];
  }

  c5_c_y = NULL;
  sf_mex_assign(&c5_c_y, sf_mex_create("y", c5_b_u, 11, 0U, 1U, 0U, 1, 8), false);
  sf_mex_setcell(c5_y, 1, c5_c_y);
  c5_b_hoistedGlobal = *c5_timeLastTargetFound;
  c5_c_u = c5_b_hoistedGlobal;
  c5_d_y = NULL;
  sf_mex_assign(&c5_d_y, sf_mex_create("y", &c5_c_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c5_y, 2, c5_d_y);
  c5_c_hoistedGlobal = chartInstance->c5_allowedRunTime;
  c5_d_u = c5_c_hoistedGlobal;
  c5_e_y = NULL;
  sf_mex_assign(&c5_e_y, sf_mex_create("y", &c5_d_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c5_y, 3, c5_e_y);
  for (c5_i2 = 0; c5_i2 < 8; c5_i2++) {
    c5_e_u[c5_i2] = chartInstance->c5_currentTarget[c5_i2];
  }

  c5_f_y = NULL;
  sf_mex_assign(&c5_f_y, sf_mex_create("y", c5_e_u, 11, 0U, 1U, 0U, 1, 8), false);
  sf_mex_setcell(c5_y, 4, c5_f_y);
  c5_d_hoistedGlobal = chartInstance->local_t;
  c5_f_u = c5_d_hoistedGlobal;
  c5_g_y = NULL;
  sf_mex_assign(&c5_g_y, sf_mex_create("y", &c5_f_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c5_y, 5, c5_g_y);
  c5_e_hoistedGlobal = chartInstance->c5_nearLocation;
  c5_g_u = c5_e_hoistedGlobal;
  c5_h_y = NULL;
  sf_mex_assign(&c5_h_y, sf_mex_create("y", &c5_g_u, 11, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c5_y, 6, c5_h_y);
  c5_f_hoistedGlobal = chartInstance->c5_numTargets;
  c5_h_u = c5_f_hoistedGlobal;
  c5_i_y = NULL;
  sf_mex_assign(&c5_i_y, sf_mex_create("y", &c5_h_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c5_y, 7, c5_i_y);
  c5_g_hoistedGlobal = chartInstance->c5_numTargetsFound;
  c5_i_u = c5_g_hoistedGlobal;
  c5_j_y = NULL;
  sf_mex_assign(&c5_j_y, sf_mex_create("y", &c5_i_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c5_y, 8, c5_j_y);
  c5_h_hoistedGlobal = chartInstance->c5_snapshotTime;
  c5_j_u = c5_h_hoistedGlobal;
  c5_k_y = NULL;
  sf_mex_assign(&c5_k_y, sf_mex_create("y", &c5_j_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c5_y, 9, c5_k_y);
  c5_i_hoistedGlobal = chartInstance->c5_startingRadius;
  c5_k_u = c5_i_hoistedGlobal;
  c5_l_y = NULL;
  sf_mex_assign(&c5_l_y, sf_mex_create("y", &c5_k_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c5_y, 10, c5_l_y);
  c5_j_hoistedGlobal = chartInstance->c5_statTimeReq;
  c5_l_u = c5_j_hoistedGlobal;
  c5_m_y = NULL;
  sf_mex_assign(&c5_m_y, sf_mex_create("y", &c5_l_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c5_y, 11, c5_m_y);
  c5_k_hoistedGlobal = chartInstance->c5_targetBoundaryRadius;
  c5_m_u = c5_k_hoistedGlobal;
  c5_n_y = NULL;
  sf_mex_assign(&c5_n_y, sf_mex_create("y", &c5_m_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c5_y, 12, c5_n_y);
  c5_l_hoistedGlobal = chartInstance->c5_is_active_c5_SimulationModel;
  c5_n_u = c5_l_hoistedGlobal;
  c5_o_y = NULL;
  sf_mex_assign(&c5_o_y, sf_mex_create("y", &c5_n_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c5_y, 13, c5_o_y);
  c5_m_hoistedGlobal = chartInstance->c5_is_c5_SimulationModel;
  c5_o_u = c5_m_hoistedGlobal;
  c5_p_y = NULL;
  sf_mex_assign(&c5_p_y, sf_mex_create("y", &c5_o_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c5_y, 14, c5_p_y);
  c5_n_hoistedGlobal = chartInstance->c5_is_Running;
  c5_p_u = c5_n_hoistedGlobal;
  c5_q_y = NULL;
  sf_mex_assign(&c5_q_y, sf_mex_create("y", &c5_p_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c5_y, 15, c5_q_y);
  sf_mex_assign(&c5_st, c5_y, false);
  return c5_st;
}

static void set_sim_state_c5_SimulationModel(SFc5_SimulationModelInstanceStruct *
  chartInstance, const mxArray *c5_st)
{
  const mxArray *c5_u;
  boolean_T c5_bv0[8];
  int32_T c5_i3;
  boolean_T c5_bv1[8];
  int32_T c5_i4;
  real_T *c5_elapsedTime;
  real_T *c5_timeLastTargetFound;
  boolean_T (*c5_targetsFound)[8];
  c5_targetsFound = (boolean_T (*)[8])ssGetOutputPortSignal(chartInstance->S, 3);
  c5_elapsedTime = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c5_timeLastTargetFound = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c5_u = sf_mex_dup(c5_st);
  *c5_elapsedTime = c5_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell
    (c5_u, 0)), "elapsedTime");
  c5_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c5_u, 1)),
                        "targetsFound", c5_bv0);
  for (c5_i3 = 0; c5_i3 < 8; c5_i3++) {
    (*c5_targetsFound)[c5_i3] = c5_bv0[c5_i3];
  }

  *c5_timeLastTargetFound = c5_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c5_u, 2)), "timeLastTargetFound");
  chartInstance->c5_allowedRunTime = c5_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell(c5_u, 3)), "allowedRunTime");
  c5_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c5_u, 4)),
                        "currentTarget", c5_bv1);
  for (c5_i4 = 0; c5_i4 < 8; c5_i4++) {
    chartInstance->c5_currentTarget[c5_i4] = c5_bv1[c5_i4];
  }

  chartInstance->local_t = c5_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c5_u, 5)), "local_t");
  chartInstance->c5_nearLocation = c5_e_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell(c5_u, 6)), "nearLocation");
  chartInstance->c5_numTargets = c5_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c5_u, 7)), "numTargets");
  chartInstance->c5_numTargetsFound = c5_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell(c5_u, 8)), "numTargetsFound");
  chartInstance->c5_snapshotTime = c5_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c5_u, 9)), "snapshotTime");
  chartInstance->c5_startingRadius = c5_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell(c5_u, 10)), "startingRadius");
  chartInstance->c5_statTimeReq = c5_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c5_u, 11)), "statTimeReq");
  chartInstance->c5_targetBoundaryRadius = c5_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell(c5_u, 12)), "targetBoundaryRadius");
  chartInstance->c5_is_active_c5_SimulationModel = c5_k_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c5_u, 13)),
     "is_active_c5_SimulationModel");
  chartInstance->c5_is_c5_SimulationModel = c5_k_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell(c5_u, 14)), "is_c5_SimulationModel");
  chartInstance->c5_is_Running = c5_k_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c5_u, 15)), "is_Running");
  sf_mex_assign(&chartInstance->c5_setSimStateSideEffectsInfo,
                c5_m_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell
    (c5_u, 16)), "setSimStateSideEffectsInfo"), true);
  sf_mex_destroy(&c5_u);
  chartInstance->c5_doSetSimStateSideEffects = 1U;
  c5_update_debugger_state_c5_SimulationModel(chartInstance);
  sf_mex_destroy(&c5_st);
}

static void c5_set_sim_state_side_effects_c5_SimulationModel
  (SFc5_SimulationModelInstanceStruct *chartInstance)
{
  if (chartInstance->c5_doSetSimStateSideEffects != 0) {
    if (chartInstance->c5_is_c5_SimulationModel == c5_IN_Finished) {
      chartInstance->c5_tp_Finished = 1U;
    } else {
      chartInstance->c5_tp_Finished = 0U;
    }

    if (chartInstance->c5_is_c5_SimulationModel == c5_IN_Running) {
      chartInstance->c5_tp_Running = 1U;
    } else {
      chartInstance->c5_tp_Running = 0U;
    }

    if (chartInstance->c5_is_Running == c5_IN_NearLocation) {
      chartInstance->c5_tp_NearLocation = 1U;
    } else {
      chartInstance->c5_tp_NearLocation = 0U;
    }

    if (chartInstance->c5_is_Running == c5_IN_Roaming) {
      chartInstance->c5_tp_Roaming = 1U;
    } else {
      chartInstance->c5_tp_Roaming = 0U;
    }

    if (chartInstance->c5_is_c5_SimulationModel == c5_IN_Standby) {
      chartInstance->c5_tp_Standby = 1U;
    } else {
      chartInstance->c5_tp_Standby = 0U;
    }

    chartInstance->c5_doSetSimStateSideEffects = 0U;
  }
}

static void finalize_c5_SimulationModel(SFc5_SimulationModelInstanceStruct
  *chartInstance)
{
  sf_mex_destroy(&chartInstance->c5_setSimStateSideEffectsInfo);
}

static void sf_gateway_c5_SimulationModel(SFc5_SimulationModelInstanceStruct
  *chartInstance)
{
  int32_T c5_i5;
  int32_T c5_i6;
  int32_T c5_i7;
  int32_T c5_i8;
  int32_T c5_i9;
  uint32_T c5_debug_family_var_map[3];
  real_T c5_nargin = 0.0;
  real_T c5_nargout = 1.0;
  boolean_T c5_out;
  real_T c5_b_nargin = 0.0;
  real_T c5_b_nargout = 1.0;
  boolean_T c5_b_out;
  int32_T c5_i10;
  real_T c5_dv1[2];
  int32_T c5_i11;
  real_T c5_dv2[2];
  uint32_T c5_b_debug_family_var_map[2];
  real_T c5_c_nargin = 0.0;
  real_T c5_c_nargout = 0.0;
  boolean_T *c5_StartFlag;
  real_T *c5_timeLastTargetFound;
  real_T *c5_elapsedTime;
  real_T *c5_sim_time;
  real_T (*c5_robotPosition)[2];
  boolean_T (*c5_targetsFound)[8];
  real32_T (*c5_targetPositions)[16];
  boolean_T guard1 = false;
  c5_sim_time = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
  c5_targetsFound = (boolean_T (*)[8])ssGetOutputPortSignal(chartInstance->S, 3);
  c5_elapsedTime = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c5_timeLastTargetFound = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c5_targetPositions = (real32_T (*)[16])ssGetInputPortSignal(chartInstance->S,
    2);
  c5_StartFlag = (boolean_T *)ssGetInputPortSignal(chartInstance->S, 1);
  c5_robotPosition = (real_T (*)[2])ssGetInputPortSignal(chartInstance->S, 0);
  c5_set_sim_state_side_effects_c5_SimulationModel(chartInstance);
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 1U, chartInstance->c5_sfEvent);
  for (c5_i5 = 0; c5_i5 < 2; c5_i5++) {
    _SFD_DATA_RANGE_CHECK(chartInstance->c5_startPos[c5_i5], 0U);
  }

  _SFD_DATA_RANGE_CHECK(chartInstance->c5_allowedRunTime, 1U);
  _SFD_DATA_RANGE_CHECK(chartInstance->c5_statTimeReq, 2U);
  _SFD_DATA_RANGE_CHECK(chartInstance->c5_startingRadius, 3U);
  for (c5_i6 = 0; c5_i6 < 2; c5_i6++) {
    _SFD_DATA_RANGE_CHECK((*c5_robotPosition)[c5_i6], 4U);
  }

  _SFD_DATA_RANGE_CHECK(chartInstance->c5_targetBoundaryRadius, 5U);
  _SFD_DATA_RANGE_CHECK((real_T)*c5_StartFlag, 6U);
  for (c5_i7 = 0; c5_i7 < 16; c5_i7++) {
    _SFD_DATA_RANGE_CHECK((real_T)(*c5_targetPositions)[c5_i7], 7U);
  }

  _SFD_DATA_RANGE_CHECK(*c5_timeLastTargetFound, 8U);
  _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c5_nearLocation, 9U);
  _SFD_DATA_RANGE_CHECK(chartInstance->c5_numTargetsFound, 10U);
  _SFD_DATA_RANGE_CHECK(*c5_elapsedTime, 11U);
  _SFD_DATA_RANGE_CHECK(chartInstance->c5_numTargets, 12U);
  for (c5_i8 = 0; c5_i8 < 8; c5_i8++) {
    _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c5_currentTarget[c5_i8], 13U);
  }

  for (c5_i9 = 0; c5_i9 < 8; c5_i9++) {
    _SFD_DATA_RANGE_CHECK((real_T)(*c5_targetsFound)[c5_i9], 14U);
  }

  _SFD_DATA_RANGE_CHECK(chartInstance->c5_snapshotTime, 15U);
  _SFD_DATA_RANGE_CHECK(chartInstance->local_t, 16U);
  _SFD_DATA_RANGE_CHECK(*c5_sim_time, 17U);
  chartInstance->c5_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 1U, chartInstance->c5_sfEvent);
  if (chartInstance->c5_is_active_c5_SimulationModel == 0U) {
    _SFD_CC_CALL(CHART_ENTER_ENTRY_FUNCTION_TAG, 1U, chartInstance->c5_sfEvent);
    chartInstance->c5_is_active_c5_SimulationModel = 1U;
    _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 1U, chartInstance->c5_sfEvent);
    _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 6U, chartInstance->c5_sfEvent);
    chartInstance->c5_is_c5_SimulationModel = c5_IN_Standby;
    _SFD_CS_CALL(STATE_ACTIVE_TAG, 4U, chartInstance->c5_sfEvent);
    chartInstance->c5_tp_Standby = 1U;
    c5_enter_atomic_Standby(chartInstance);
  } else {
    switch (chartInstance->c5_is_c5_SimulationModel) {
     case c5_IN_Finished:
      CV_CHART_EVAL(1, 0, 1);
      _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG, 10U,
                   chartInstance->c5_sfEvent);
      _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 3U, 3U, c5_m_debug_family_names,
        c5_debug_family_var_map);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_nargin, 0U, c5_sf_marshallOut,
        c5_sf_marshallIn);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_nargout, 1U, c5_sf_marshallOut,
        c5_sf_marshallIn);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_out, 2U, c5_c_sf_marshallOut,
        c5_c_sf_marshallIn);
      c5_out = CV_EML_IF(10, 0, 0, !*c5_StartFlag);
      _SFD_SYMBOL_SCOPE_POP();
      if (c5_out) {
        _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 10U, chartInstance->c5_sfEvent);
        chartInstance->c5_tp_Finished = 0U;
        _SFD_CS_CALL(STATE_INACTIVE_TAG, 0U, chartInstance->c5_sfEvent);
        chartInstance->c5_is_c5_SimulationModel = c5_IN_Standby;
        _SFD_CS_CALL(STATE_ACTIVE_TAG, 4U, chartInstance->c5_sfEvent);
        chartInstance->c5_tp_Standby = 1U;
        c5_enter_atomic_Standby(chartInstance);
      } else {
        _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG, 0U,
                     chartInstance->c5_sfEvent);
      }

      _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG, 0U, chartInstance->c5_sfEvent);
      break;

     case c5_IN_Running:
      CV_CHART_EVAL(1, 0, 2);
      c5_Running(chartInstance);
      break;

     case c5_IN_Standby:
      CV_CHART_EVAL(1, 0, 3);
      _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG, 0U,
                   chartInstance->c5_sfEvent);
      _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 3U, 3U, c5_n_debug_family_names,
        c5_debug_family_var_map);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_b_nargin, 0U, c5_sf_marshallOut,
        c5_sf_marshallIn);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_b_nargout, 1U, c5_sf_marshallOut,
        c5_sf_marshallIn);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_b_out, 2U, c5_c_sf_marshallOut,
        c5_c_sf_marshallIn);
      guard1 = false;
      if (CV_EML_COND(0, 0, 0, *c5_StartFlag)) {
        for (c5_i10 = 0; c5_i10 < 2; c5_i10++) {
          c5_dv1[c5_i10] = (*c5_robotPosition)[c5_i10];
        }

        for (c5_i11 = 0; c5_i11 < 2; c5_i11++) {
          c5_dv2[c5_i11] = chartInstance->c5_startPos[c5_i11];
        }

        if (CV_EML_COND(0, 0, 1, c5_distancefrom(chartInstance, c5_dv1, c5_dv2) <
                        chartInstance->c5_startingRadius)) {
          CV_EML_MCDC(0, 0, 0, true);
          CV_EML_IF(0, 0, 0, true);
          c5_b_out = true;
        } else {
          guard1 = true;
        }
      } else {
        c5_errorIfDataNotWrittenToFcn(chartInstance, 2U, 3U);
        guard1 = true;
      }

      if (guard1 == true) {
        CV_EML_MCDC(0, 0, 0, false);
        CV_EML_IF(0, 0, 0, false);
        c5_b_out = false;
      }

      _SFD_SYMBOL_SCOPE_POP();
      if (c5_b_out) {
        _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 0U, chartInstance->c5_sfEvent);
        chartInstance->c5_tp_Standby = 0U;
        _SFD_CS_CALL(STATE_INACTIVE_TAG, 4U, chartInstance->c5_sfEvent);
        chartInstance->c5_is_c5_SimulationModel = c5_IN_Running;
        _SFD_CS_CALL(STATE_ACTIVE_TAG, 1U, chartInstance->c5_sfEvent);
        chartInstance->c5_tp_Running = 1U;
        _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c5_c_debug_family_names,
          c5_b_debug_family_var_map);
        _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_c_nargin, 0U, c5_sf_marshallOut,
          c5_sf_marshallIn);
        _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_c_nargout, 1U,
          c5_sf_marshallOut, c5_sf_marshallIn);
        c5_startTiming(chartInstance);
        _SFD_SYMBOL_SCOPE_POP();
        _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 4U, chartInstance->c5_sfEvent);
        chartInstance->c5_is_Running = c5_IN_Roaming;
        _SFD_CS_CALL(STATE_ACTIVE_TAG, 3U, chartInstance->c5_sfEvent);
        chartInstance->c5_tp_Roaming = 1U;
      } else {
        _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG, 4U,
                     chartInstance->c5_sfEvent);
      }

      _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG, 4U, chartInstance->c5_sfEvent);
      break;

     default:
      CV_CHART_EVAL(1, 0, 0);
      chartInstance->c5_is_c5_SimulationModel = c5_IN_NO_ACTIVE_CHILD;
      _SFD_CS_CALL(STATE_INACTIVE_TAG, 0U, chartInstance->c5_sfEvent);
      break;
    }
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 1U, chartInstance->c5_sfEvent);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_SimulationModelMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void initSimStructsc5_SimulationModel(SFc5_SimulationModelInstanceStruct *
  chartInstance)
{
  (void)chartInstance;
}

static void c5_enter_atomic_Standby(SFc5_SimulationModelInstanceStruct
  *chartInstance)
{
  uint32_T c5_debug_family_var_map[2];
  real_T c5_nargin = 0.0;
  real_T c5_nargout = 0.0;
  int32_T c5_i12;
  int32_T c5_i13;
  real_T *c5_timeLastTargetFound;
  real_T *c5_elapsedTime;
  boolean_T (*c5_targetsFound)[8];
  c5_targetsFound = (boolean_T (*)[8])ssGetOutputPortSignal(chartInstance->S, 3);
  c5_elapsedTime = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c5_timeLastTargetFound = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c5_b_debug_family_names,
    c5_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_nargin, 0U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_nargout, 1U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  chartInstance->c5_numTargets = 8.0;
  c5_updateDataWrittenToVector(chartInstance, 8U);
  _SFD_DATA_RANGE_CHECK(chartInstance->c5_numTargets, 12U);
  for (c5_i12 = 0; c5_i12 < 8; c5_i12++) {
    (*c5_targetsFound)[c5_i12] = false;
  }

  c5_updateDataWrittenToVector(chartInstance, 10U);
  for (c5_i13 = 0; c5_i13 < 8; c5_i13++) {
    _SFD_DATA_RANGE_CHECK((real_T)(*c5_targetsFound)[c5_i13], 14U);
  }

  *c5_timeLastTargetFound = 0.0;
  c5_updateDataWrittenToVector(chartInstance, 4U);
  _SFD_DATA_RANGE_CHECK(*c5_timeLastTargetFound, 8U);
  *c5_elapsedTime = 0.0;
  c5_updateDataWrittenToVector(chartInstance, 7U);
  _SFD_DATA_RANGE_CHECK(*c5_elapsedTime, 11U);
  chartInstance->c5_numTargetsFound = 0.0;
  c5_updateDataWrittenToVector(chartInstance, 6U);
  _SFD_DATA_RANGE_CHECK(chartInstance->c5_numTargetsFound, 10U);
  chartInstance->c5_allowedRunTime = 180.0;
  c5_updateDataWrittenToVector(chartInstance, 0U);
  _SFD_DATA_RANGE_CHECK(chartInstance->c5_allowedRunTime, 1U);
  chartInstance->c5_startingRadius = 10.0;
  c5_updateDataWrittenToVector(chartInstance, 2U);
  _SFD_DATA_RANGE_CHECK(chartInstance->c5_startingRadius, 3U);
  chartInstance->c5_targetBoundaryRadius = 7.0;
  c5_updateDataWrittenToVector(chartInstance, 3U);
  _SFD_DATA_RANGE_CHECK(chartInstance->c5_targetBoundaryRadius, 5U);
  chartInstance->c5_statTimeReq = 2.5;
  c5_updateDataWrittenToVector(chartInstance, 1U);
  _SFD_DATA_RANGE_CHECK(chartInstance->c5_statTimeReq, 2U);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c5_Running(SFc5_SimulationModelInstanceStruct *chartInstance)
{
  uint32_T c5_debug_family_var_map[3];
  real_T c5_nargin = 0.0;
  real_T c5_nargout = 1.0;
  boolean_T c5_out;
  real_T c5_hoistedGlobal;
  real_T c5_b_hoistedGlobal;
  real_T c5_b_nargin = 0.0;
  real_T c5_b_nargout = 1.0;
  boolean_T c5_b_out;
  uint32_T c5_b_debug_family_var_map[2];
  real_T c5_c_nargin = 0.0;
  real_T c5_c_nargout = 0.0;
  int32_T c5_i14;
  real_T c5_dv3[2];
  int32_T c5_i15;
  real32_T c5_fv0[16];
  boolean_T c5_b_currentTarget[8];
  boolean_T c5_b_nearLocation;
  int32_T c5_i16;
  int32_T c5_i17;
  real_T c5_d_nargin = 0.0;
  real_T c5_d_nargout = 1.0;
  boolean_T c5_c_out;
  real_T c5_e_nargin = 0.0;
  real_T c5_e_nargout = 0.0;
  int32_T c5_i18;
  boolean_T c5_bv2[8];
  int32_T c5_i19;
  boolean_T c5_bv3[8];
  real_T c5_timeLastTargetFound;
  boolean_T c5_bv4[8];
  int32_T c5_i20;
  int32_T c5_i21;
  real_T c5_y;
  int32_T c5_k;
  int32_T c5_b_k;
  real_T c5_f_nargin = 0.0;
  real_T c5_f_nargout = 1.0;
  boolean_T c5_d_out;
  real_T c5_g_nargin = 0.0;
  real_T c5_g_nargout = 1.0;
  boolean_T c5_e_out;
  real_T c5_h_nargin = 0.0;
  real_T c5_h_nargout = 0.0;
  real_T *c5_elapsedTime;
  real_T *c5_b_timeLastTargetFound;
  boolean_T *c5_StartFlag;
  boolean_T (*c5_targetsFound)[8];
  real32_T (*c5_targetPositions)[16];
  real_T (*c5_robotPosition)[2];
  boolean_T guard1 = false;
  c5_targetsFound = (boolean_T (*)[8])ssGetOutputPortSignal(chartInstance->S, 3);
  c5_elapsedTime = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c5_b_timeLastTargetFound = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c5_targetPositions = (real32_T (*)[16])ssGetInputPortSignal(chartInstance->S,
    2);
  c5_StartFlag = (boolean_T *)ssGetInputPortSignal(chartInstance->S, 1);
  c5_robotPosition = (real_T (*)[2])ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG, 8U, chartInstance->c5_sfEvent);
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 3U, 3U, c5_q_debug_family_names,
    c5_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_nargin, 0U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_nargout, 1U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_out, 2U, c5_c_sf_marshallOut,
    c5_c_sf_marshallIn);
  c5_errorIfDataNotWrittenToFcn(chartInstance, 6U, 10U);
  c5_errorIfDataNotWrittenToFcn(chartInstance, 8U, 12U);
  guard1 = false;
  if (CV_EML_COND(8, 0, 0, chartInstance->c5_numTargetsFound ==
                  chartInstance->c5_numTargets)) {
    guard1 = true;
  } else {
    c5_errorIfDataNotWrittenToFcn(chartInstance, 7U, 11U);
    c5_errorIfDataNotWrittenToFcn(chartInstance, 0U, 1U);
    c5_hoistedGlobal = *c5_elapsedTime;
    c5_b_hoistedGlobal = chartInstance->c5_allowedRunTime;
    if (CV_EML_COND(8, 0, 1, c5_hoistedGlobal >= c5_b_hoistedGlobal -
                    1.4901161193847656E-8)) {
      guard1 = true;
    } else {
      CV_EML_MCDC(8, 0, 0, false);
      CV_EML_IF(8, 0, 0, false);
      c5_out = false;
    }
  }

  if (guard1 == true) {
    CV_EML_MCDC(8, 0, 0, true);
    CV_EML_IF(8, 0, 0, true);
    c5_out = true;
  }

  _SFD_SYMBOL_SCOPE_POP();
  if (c5_out) {
    _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 8U, chartInstance->c5_sfEvent);
    c5_exit_internal_Running(chartInstance);
    chartInstance->c5_tp_Running = 0U;
    _SFD_CS_CALL(STATE_INACTIVE_TAG, 1U, chartInstance->c5_sfEvent);
    chartInstance->c5_is_c5_SimulationModel = c5_IN_Finished;
    _SFD_CS_CALL(STATE_ACTIVE_TAG, 0U, chartInstance->c5_sfEvent);
    chartInstance->c5_tp_Finished = 1U;
  } else {
    _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG, 9U, chartInstance->c5_sfEvent);
    _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 3U, 3U, c5_p_debug_family_names,
      c5_debug_family_var_map);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_b_nargin, 0U, c5_sf_marshallOut,
      c5_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_b_nargout, 1U, c5_sf_marshallOut,
      c5_sf_marshallIn);
    _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_b_out, 2U, c5_c_sf_marshallOut,
      c5_c_sf_marshallIn);
    c5_b_out = CV_EML_IF(9, 0, 0, !*c5_StartFlag);
    _SFD_SYMBOL_SCOPE_POP();
    if (c5_b_out) {
      _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 9U, chartInstance->c5_sfEvent);
      c5_exit_internal_Running(chartInstance);
      chartInstance->c5_tp_Running = 0U;
      _SFD_CS_CALL(STATE_INACTIVE_TAG, 1U, chartInstance->c5_sfEvent);
      chartInstance->c5_is_c5_SimulationModel = c5_IN_Standby;
      _SFD_CS_CALL(STATE_ACTIVE_TAG, 4U, chartInstance->c5_sfEvent);
      chartInstance->c5_tp_Standby = 1U;
      c5_enter_atomic_Standby(chartInstance);
    } else {
      _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG, 1U,
                   chartInstance->c5_sfEvent);
      _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c5_d_debug_family_names,
        c5_b_debug_family_var_map);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_c_nargin, 0U, c5_sf_marshallOut,
        c5_sf_marshallIn);
      _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_c_nargout, 1U, c5_sf_marshallOut,
        c5_sf_marshallIn);
      *c5_elapsedTime = c5_getTiming(chartInstance);
      c5_updateDataWrittenToVector(chartInstance, 7U);
      _SFD_DATA_RANGE_CHECK(*c5_elapsedTime, 11U);
      c5_errorIfDataNotWrittenToFcn(chartInstance, 3U, 5U);
      for (c5_i14 = 0; c5_i14 < 2; c5_i14++) {
        c5_dv3[c5_i14] = (*c5_robotPosition)[c5_i14];
      }

      for (c5_i15 = 0; c5_i15 < 16; c5_i15++) {
        c5_fv0[c5_i15] = (*c5_targetPositions)[c5_i15];
      }

      c5_testIfNearLocation(chartInstance, c5_dv3, c5_fv0,
                            chartInstance->c5_targetBoundaryRadius,
                            &c5_b_nearLocation, c5_b_currentTarget);
      chartInstance->c5_nearLocation = c5_b_nearLocation;
      c5_updateDataWrittenToVector(chartInstance, 5U);
      _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c5_nearLocation, 9U);
      for (c5_i16 = 0; c5_i16 < 8; c5_i16++) {
        chartInstance->c5_currentTarget[c5_i16] = c5_b_currentTarget[c5_i16];
      }

      c5_updateDataWrittenToVector(chartInstance, 9U);
      for (c5_i17 = 0; c5_i17 < 8; c5_i17++) {
        _SFD_DATA_RANGE_CHECK((real_T)chartInstance->c5_currentTarget[c5_i17],
                              13U);
      }

      _SFD_SYMBOL_SCOPE_POP();
      switch (chartInstance->c5_is_Running) {
       case c5_IN_NearLocation:
        CV_STATE_EVAL(1, 0, 1);
        _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG, 2U,
                     chartInstance->c5_sfEvent);
        _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 3U, 3U, c5_j_debug_family_names,
          c5_debug_family_var_map);
        _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_d_nargin, 0U, c5_sf_marshallOut,
          c5_sf_marshallIn);
        _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_d_nargout, 1U,
          c5_sf_marshallOut, c5_sf_marshallIn);
        _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_c_out, 2U, c5_c_sf_marshallOut,
          c5_c_sf_marshallIn);
        c5_errorIfDataNotWrittenToFcn(chartInstance, 7U, 11U);
        c5_errorIfDataNotWrittenToFcn(chartInstance, 11U, 15U);
        c5_errorIfDataNotWrittenToFcn(chartInstance, 1U, 2U);
        c5_c_out = CV_EML_IF(2, 0, 0, *c5_elapsedTime -
                             chartInstance->c5_snapshotTime >
                             chartInstance->c5_statTimeReq);
        _SFD_SYMBOL_SCOPE_POP();
        if (c5_c_out) {
          _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 2U, chartInstance->c5_sfEvent);
          _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 7U, chartInstance->c5_sfEvent);
          _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c5_k_debug_family_names,
            c5_b_debug_family_var_map);
          _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_e_nargin, 0U,
            c5_sf_marshallOut, c5_sf_marshallIn);
          _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_e_nargout, 1U,
            c5_sf_marshallOut, c5_sf_marshallIn);
          c5_errorIfDataNotWrittenToFcn(chartInstance, 10U, 14U);
          c5_errorIfDataNotWrittenToFcn(chartInstance, 9U, 13U);
          c5_errorIfDataNotWrittenToFcn(chartInstance, 4U, 8U);
          c5_errorIfDataNotWrittenToFcn(chartInstance, 7U, 11U);
          for (c5_i18 = 0; c5_i18 < 8; c5_i18++) {
            c5_bv2[c5_i18] = (*c5_targetsFound)[c5_i18];
          }

          for (c5_i19 = 0; c5_i19 < 8; c5_i19++) {
            c5_bv3[c5_i19] = chartInstance->c5_currentTarget[c5_i19];
          }

          c5_updateTargetsFound(chartInstance, c5_bv2, c5_bv3,
                                *c5_b_timeLastTargetFound, *c5_elapsedTime,
                                c5_bv4, &c5_timeLastTargetFound);
          for (c5_i20 = 0; c5_i20 < 8; c5_i20++) {
            (*c5_targetsFound)[c5_i20] = c5_bv4[c5_i20];
          }

          c5_updateDataWrittenToVector(chartInstance, 10U);
          for (c5_i21 = 0; c5_i21 < 8; c5_i21++) {
            _SFD_DATA_RANGE_CHECK((real_T)(*c5_targetsFound)[c5_i21], 14U);
          }

          *c5_b_timeLastTargetFound = c5_timeLastTargetFound;
          c5_updateDataWrittenToVector(chartInstance, 4U);
          _SFD_DATA_RANGE_CHECK(*c5_b_timeLastTargetFound, 8U);
          c5_errorIfDataNotWrittenToFcn(chartInstance, 10U, 14U);
          c5_isequal(chartInstance);
          c5_y = (real_T)(*c5_targetsFound)[0];
          for (c5_k = 2; c5_k < 9; c5_k++) {
            c5_b_k = c5_k;
            c5_y += (real_T)(*c5_targetsFound)[_SFD_EML_ARRAY_BOUNDS_CHECK("",
              (int32_T)_SFD_INTEGER_CHECK("", (real_T)c5_b_k), 1, 8, 1, 0) - 1];
          }

          chartInstance->c5_numTargetsFound = c5_y;
          c5_updateDataWrittenToVector(chartInstance, 6U);
          _SFD_DATA_RANGE_CHECK(chartInstance->c5_numTargetsFound, 10U);
          _SFD_SYMBOL_SCOPE_POP();
          _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 5U, chartInstance->c5_sfEvent);
          chartInstance->c5_tp_NearLocation = 0U;
          _SFD_CS_CALL(STATE_INACTIVE_TAG, 2U, chartInstance->c5_sfEvent);
          chartInstance->c5_is_Running = c5_IN_Roaming;
          _SFD_CS_CALL(STATE_ACTIVE_TAG, 3U, chartInstance->c5_sfEvent);
          chartInstance->c5_tp_Roaming = 1U;
        } else {
          _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG, 3U,
                       chartInstance->c5_sfEvent);
          _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 3U, 3U, c5_i_debug_family_names,
            c5_debug_family_var_map);
          _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_f_nargin, 0U,
            c5_sf_marshallOut, c5_sf_marshallIn);
          _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_f_nargout, 1U,
            c5_sf_marshallOut, c5_sf_marshallIn);
          _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_d_out, 2U,
            c5_c_sf_marshallOut, c5_c_sf_marshallIn);
          c5_errorIfDataNotWrittenToFcn(chartInstance, 5U, 9U);
          c5_d_out = CV_EML_IF(3, 0, 0, !chartInstance->c5_nearLocation);
          _SFD_SYMBOL_SCOPE_POP();
          if (c5_d_out) {
            _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 3U, chartInstance->c5_sfEvent);
            chartInstance->c5_tp_NearLocation = 0U;
            _SFD_CS_CALL(STATE_INACTIVE_TAG, 2U, chartInstance->c5_sfEvent);
            chartInstance->c5_is_Running = c5_IN_Roaming;
            _SFD_CS_CALL(STATE_ACTIVE_TAG, 3U, chartInstance->c5_sfEvent);
            chartInstance->c5_tp_Roaming = 1U;
          } else {
            _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG, 2U,
                         chartInstance->c5_sfEvent);
          }
        }

        _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG, 2U, chartInstance->c5_sfEvent);
        break;

       case c5_IN_Roaming:
        CV_STATE_EVAL(1, 0, 2);
        _SFD_CT_CALL(TRANSITION_BEFORE_PROCESSING_TAG, 1U,
                     chartInstance->c5_sfEvent);
        _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 3U, 3U, c5_h_debug_family_names,
          c5_debug_family_var_map);
        _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_g_nargin, 0U, c5_sf_marshallOut,
          c5_sf_marshallIn);
        _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_g_nargout, 1U,
          c5_sf_marshallOut, c5_sf_marshallIn);
        _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_e_out, 2U, c5_c_sf_marshallOut,
          c5_c_sf_marshallIn);
        c5_errorIfDataNotWrittenToFcn(chartInstance, 5U, 9U);
        c5_e_out = CV_EML_IF(1, 0, 0, chartInstance->c5_nearLocation);
        _SFD_SYMBOL_SCOPE_POP();
        if (c5_e_out) {
          _SFD_CT_CALL(TRANSITION_ACTIVE_TAG, 1U, chartInstance->c5_sfEvent);
          chartInstance->c5_tp_Roaming = 0U;
          _SFD_CS_CALL(STATE_INACTIVE_TAG, 3U, chartInstance->c5_sfEvent);
          chartInstance->c5_is_Running = c5_IN_NearLocation;
          _SFD_CS_CALL(STATE_ACTIVE_TAG, 2U, chartInstance->c5_sfEvent);
          chartInstance->c5_tp_NearLocation = 1U;
          _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c5_g_debug_family_names,
            c5_b_debug_family_var_map);
          _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_h_nargin, 0U,
            c5_sf_marshallOut, c5_sf_marshallIn);
          _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_h_nargout, 1U,
            c5_sf_marshallOut, c5_sf_marshallIn);
          c5_errorIfDataNotWrittenToFcn(chartInstance, 7U, 11U);
          chartInstance->c5_snapshotTime = *c5_elapsedTime;
          c5_updateDataWrittenToVector(chartInstance, 11U);
          _SFD_DATA_RANGE_CHECK(chartInstance->c5_snapshotTime, 15U);
          _SFD_SYMBOL_SCOPE_POP();
        } else {
          _SFD_CS_CALL(STATE_ENTER_DURING_FUNCTION_TAG, 3U,
                       chartInstance->c5_sfEvent);
        }

        _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG, 3U, chartInstance->c5_sfEvent);
        break;

       default:
        CV_STATE_EVAL(1, 0, 0);
        chartInstance->c5_is_Running = c5_IN_NO_ACTIVE_CHILD;
        _SFD_CS_CALL(STATE_INACTIVE_TAG, 2U, chartInstance->c5_sfEvent);
        break;
      }
    }
  }

  _SFD_CS_CALL(EXIT_OUT_OF_FUNCTION_TAG, 1U, chartInstance->c5_sfEvent);
}

static void c5_exit_internal_Running(SFc5_SimulationModelInstanceStruct
  *chartInstance)
{
  switch (chartInstance->c5_is_Running) {
   case c5_IN_NearLocation:
    CV_STATE_EVAL(1, 1, 1);
    chartInstance->c5_tp_NearLocation = 0U;
    chartInstance->c5_is_Running = c5_IN_NO_ACTIVE_CHILD;
    _SFD_CS_CALL(STATE_INACTIVE_TAG, 2U, chartInstance->c5_sfEvent);
    break;

   case c5_IN_Roaming:
    CV_STATE_EVAL(1, 1, 2);
    chartInstance->c5_tp_Roaming = 0U;
    chartInstance->c5_is_Running = c5_IN_NO_ACTIVE_CHILD;
    _SFD_CS_CALL(STATE_INACTIVE_TAG, 3U, chartInstance->c5_sfEvent);
    break;

   default:
    CV_STATE_EVAL(1, 1, 0);
    chartInstance->c5_is_Running = c5_IN_NO_ACTIVE_CHILD;
    _SFD_CS_CALL(STATE_INACTIVE_TAG, 2U, chartInstance->c5_sfEvent);
    break;
  }
}

static void c5_bsxfun(SFc5_SimulationModelInstanceStruct *chartInstance,
                      real32_T c5_a[16], real_T c5_b[2], real32_T c5_c[16])
{
  int32_T c5_ak;
  int32_T c5_bk;
  int32_T c5_ck;
  int32_T c5_b_ck;
  int32_T c5_k;
  int32_T c5_b_k;
  int32_T c5_b_a;
  int32_T c5_b_b;
  int32_T c5_c_a;
  int32_T c5_c_b;
  int32_T c5_b_c;
  real32_T c5_av[8];
  int32_T c5_d_a;
  int32_T c5_e_a;
  int32_T c5_c_c;
  real_T c5_bv;
  int32_T c5_i22;
  real32_T c5_cv[8];
  int32_T c5_c_k;
  int32_T c5_f_a;
  int32_T c5_d_b;
  int32_T c5_g_a;
  int32_T c5_e_b;
  int32_T c5_d_c;
  int32_T c5_h_a;
  int32_T c5_i_a;
  int32_T c5_j_a;
  int32_T c5_k_a;
  (void)chartInstance;
  c5_ak = 0;
  c5_bk = 0;
  for (c5_ck = 0; c5_ck < 9; c5_ck += 8) {
    c5_b_ck = c5_ck;
    for (c5_k = 1; c5_k < 9; c5_k++) {
      c5_b_k = c5_k;
      c5_b_a = c5_ak;
      c5_b_b = c5_b_k;
      c5_c_a = c5_b_a;
      c5_c_b = c5_b_b;
      c5_b_c = c5_c_a + c5_c_b;
      c5_av[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c5_b_k), 1, 8, 1, 0) - 1] = c5_a[_SFD_EML_ARRAY_BOUNDS_CHECK("",
        (int32_T)_SFD_INTEGER_CHECK("", (real_T)c5_b_c), 1, 16, 1, 0) - 1];
    }

    c5_d_a = c5_bk;
    c5_e_a = c5_d_a;
    c5_c_c = c5_e_a;
    c5_bv = c5_b[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)(c5_c_c + 1)), 1, 2, 1, 0) - 1];
    for (c5_i22 = 0; c5_i22 < 8; c5_i22++) {
      c5_cv[c5_i22] = c5_av[c5_i22] - (real32_T)c5_bv;
    }

    for (c5_c_k = 1; c5_c_k < 9; c5_c_k++) {
      c5_b_k = c5_c_k;
      c5_f_a = c5_b_ck;
      c5_d_b = c5_b_k;
      c5_g_a = c5_f_a;
      c5_e_b = c5_d_b;
      c5_d_c = c5_g_a + c5_e_b;
      c5_c[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c5_d_c), 1, 16, 1, 0) - 1] = c5_cv[_SFD_EML_ARRAY_BOUNDS_CHECK(
        "", (int32_T)_SFD_INTEGER_CHECK("", (real_T)c5_b_k), 1, 8, 1, 0) - 1];
    }

    c5_h_a = c5_ak;
    c5_i_a = c5_h_a + 8;
    c5_ak = c5_i_a;
    c5_j_a = c5_bk;
    c5_k_a = c5_j_a + 1;
    c5_bk = c5_k_a;
  }
}

static void c5_eml_scalar_eg(SFc5_SimulationModelInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static boolean_T c5_any(SFc5_SimulationModelInstanceStruct *chartInstance,
  boolean_T c5_x[8])
{
  boolean_T c5_y;
  int32_T c5_k;
  real_T c5_b_k;
  boolean_T c5_b0;
  boolean_T exitg1;
  c5_isequal(chartInstance);
  c5_y = false;
  c5_k = 0;
  exitg1 = false;
  while ((exitg1 == false) && (c5_k < 8)) {
    c5_b_k = 1.0 + (real_T)c5_k;
    if ((real_T)c5_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK
         ("", c5_b_k), 1, 8, 1, 0) - 1] == 0.0) {
      c5_b0 = true;
    } else {
      (real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        c5_b_k), 1, 8, 1, 0);
      c5_b0 = false;
    }

    if (!c5_b0) {
      c5_y = true;
      exitg1 = true;
    } else {
      c5_k++;
    }
  }

  return c5_y;
}

static void c5_isequal(SFc5_SimulationModelInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c5_eml_error(SFc5_SimulationModelInstanceStruct *chartInstance)
{
  int32_T c5_i23;
  static char_T c5_cv0[30] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'E', 'l', 'F', 'u', 'n', 'D', 'o', 'm', 'a', 'i', 'n',
    'E', 'r', 'r', 'o', 'r' };

  char_T c5_u[30];
  const mxArray *c5_y = NULL;
  int32_T c5_i24;
  static char_T c5_cv1[4] = { 's', 'q', 'r', 't' };

  char_T c5_b_u[4];
  const mxArray *c5_b_y = NULL;
  (void)chartInstance;
  for (c5_i23 = 0; c5_i23 < 30; c5_i23++) {
    c5_u[c5_i23] = c5_cv0[c5_i23];
  }

  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", c5_u, 10, 0U, 1U, 0U, 2, 1, 30), false);
  for (c5_i24 = 0; c5_i24 < 4; c5_i24++) {
    c5_b_u[c5_i24] = c5_cv1[c5_i24];
  }

  c5_b_y = NULL;
  sf_mex_assign(&c5_b_y, sf_mex_create("y", c5_b_u, 10, 0U, 1U, 0U, 2, 1, 4),
                false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    2U, 14, c5_y, 14, c5_b_y));
}

static void init_script_number_translation(uint32_T c5_machineNumber, uint32_T
  c5_chartNumber, uint32_T c5_instanceNumber)
{
  (void)c5_machineNumber;
  (void)c5_chartNumber;
  (void)c5_instanceNumber;
}

static const mxArray *c5_sf_marshallOut(void *chartInstanceVoid, void *c5_inData)
{
  const mxArray *c5_mxArrayOutData = NULL;
  real_T c5_u;
  const mxArray *c5_y = NULL;
  SFc5_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc5_SimulationModelInstanceStruct *)chartInstanceVoid;
  c5_mxArrayOutData = NULL;
  c5_u = *(real_T *)c5_inData;
  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", &c5_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c5_mxArrayOutData, c5_y, false);
  return c5_mxArrayOutData;
}

static real_T c5_emlrt_marshallIn(SFc5_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c5_nargout, const char_T *c5_identifier)
{
  real_T c5_y;
  emlrtMsgIdentifier c5_thisId;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_y = c5_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_nargout), &c5_thisId);
  sf_mex_destroy(&c5_nargout);
  return c5_y;
}

static real_T c5_b_emlrt_marshallIn(SFc5_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId)
{
  real_T c5_y;
  real_T c5_d0;
  (void)chartInstance;
  sf_mex_import(c5_parentId, sf_mex_dup(c5_u), &c5_d0, 1, 0, 0U, 0, 0U, 0);
  c5_y = c5_d0;
  sf_mex_destroy(&c5_u);
  return c5_y;
}

static void c5_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData)
{
  const mxArray *c5_nargout;
  const char_T *c5_identifier;
  emlrtMsgIdentifier c5_thisId;
  real_T c5_y;
  SFc5_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc5_SimulationModelInstanceStruct *)chartInstanceVoid;
  c5_nargout = sf_mex_dup(c5_mxArrayInData);
  c5_identifier = c5_varName;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_y = c5_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_nargout), &c5_thisId);
  sf_mex_destroy(&c5_nargout);
  *(real_T *)c5_outData = c5_y;
  sf_mex_destroy(&c5_mxArrayInData);
}

static const mxArray *c5_b_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData)
{
  const mxArray *c5_mxArrayOutData = NULL;
  int32_T c5_i25;
  boolean_T c5_b_inData[8];
  int32_T c5_i26;
  boolean_T c5_u[8];
  const mxArray *c5_y = NULL;
  SFc5_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc5_SimulationModelInstanceStruct *)chartInstanceVoid;
  c5_mxArrayOutData = NULL;
  for (c5_i25 = 0; c5_i25 < 8; c5_i25++) {
    c5_b_inData[c5_i25] = (*(boolean_T (*)[8])c5_inData)[c5_i25];
  }

  for (c5_i26 = 0; c5_i26 < 8; c5_i26++) {
    c5_u[c5_i26] = c5_b_inData[c5_i26];
  }

  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", c5_u, 11, 0U, 1U, 0U, 1, 8), false);
  sf_mex_assign(&c5_mxArrayOutData, c5_y, false);
  return c5_mxArrayOutData;
}

static void c5_c_emlrt_marshallIn(SFc5_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c5_currentLoc, const char_T *c5_identifier,
  boolean_T c5_y[8])
{
  emlrtMsgIdentifier c5_thisId;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_currentLoc), &c5_thisId,
                        c5_y);
  sf_mex_destroy(&c5_currentLoc);
}

static void c5_d_emlrt_marshallIn(SFc5_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId,
  boolean_T c5_y[8])
{
  boolean_T c5_bv5[8];
  int32_T c5_i27;
  (void)chartInstance;
  sf_mex_import(c5_parentId, sf_mex_dup(c5_u), c5_bv5, 1, 11, 0U, 1, 0U, 1, 8);
  for (c5_i27 = 0; c5_i27 < 8; c5_i27++) {
    c5_y[c5_i27] = c5_bv5[c5_i27];
  }

  sf_mex_destroy(&c5_u);
}

static void c5_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData)
{
  const mxArray *c5_currentLoc;
  const char_T *c5_identifier;
  emlrtMsgIdentifier c5_thisId;
  boolean_T c5_y[8];
  int32_T c5_i28;
  SFc5_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc5_SimulationModelInstanceStruct *)chartInstanceVoid;
  c5_currentLoc = sf_mex_dup(c5_mxArrayInData);
  c5_identifier = c5_varName;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_currentLoc), &c5_thisId,
                        c5_y);
  sf_mex_destroy(&c5_currentLoc);
  for (c5_i28 = 0; c5_i28 < 8; c5_i28++) {
    (*(boolean_T (*)[8])c5_outData)[c5_i28] = c5_y[c5_i28];
  }

  sf_mex_destroy(&c5_mxArrayInData);
}

static const mxArray *c5_c_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData)
{
  const mxArray *c5_mxArrayOutData = NULL;
  boolean_T c5_u;
  const mxArray *c5_y = NULL;
  SFc5_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc5_SimulationModelInstanceStruct *)chartInstanceVoid;
  c5_mxArrayOutData = NULL;
  c5_u = *(boolean_T *)c5_inData;
  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", &c5_u, 11, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c5_mxArrayOutData, c5_y, false);
  return c5_mxArrayOutData;
}

static boolean_T c5_e_emlrt_marshallIn(SFc5_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c5_nearLoc, const char_T *c5_identifier)
{
  boolean_T c5_y;
  emlrtMsgIdentifier c5_thisId;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_y = c5_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_nearLoc), &c5_thisId);
  sf_mex_destroy(&c5_nearLoc);
  return c5_y;
}

static boolean_T c5_f_emlrt_marshallIn(SFc5_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId)
{
  boolean_T c5_y;
  boolean_T c5_b1;
  (void)chartInstance;
  sf_mex_import(c5_parentId, sf_mex_dup(c5_u), &c5_b1, 1, 11, 0U, 0, 0U, 0);
  c5_y = c5_b1;
  sf_mex_destroy(&c5_u);
  return c5_y;
}

static void c5_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData)
{
  const mxArray *c5_nearLoc;
  const char_T *c5_identifier;
  emlrtMsgIdentifier c5_thisId;
  boolean_T c5_y;
  SFc5_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc5_SimulationModelInstanceStruct *)chartInstanceVoid;
  c5_nearLoc = sf_mex_dup(c5_mxArrayInData);
  c5_identifier = c5_varName;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_y = c5_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_nearLoc), &c5_thisId);
  sf_mex_destroy(&c5_nearLoc);
  *(boolean_T *)c5_outData = c5_y;
  sf_mex_destroy(&c5_mxArrayInData);
}

static const mxArray *c5_d_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData)
{
  const mxArray *c5_mxArrayOutData = NULL;
  int32_T c5_i29;
  int32_T c5_i30;
  int32_T c5_i31;
  real32_T c5_b_inData[16];
  int32_T c5_i32;
  int32_T c5_i33;
  int32_T c5_i34;
  real32_T c5_u[16];
  const mxArray *c5_y = NULL;
  SFc5_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc5_SimulationModelInstanceStruct *)chartInstanceVoid;
  c5_mxArrayOutData = NULL;
  c5_i29 = 0;
  for (c5_i30 = 0; c5_i30 < 2; c5_i30++) {
    for (c5_i31 = 0; c5_i31 < 8; c5_i31++) {
      c5_b_inData[c5_i31 + c5_i29] = (*(real32_T (*)[16])c5_inData)[c5_i31 +
        c5_i29];
    }

    c5_i29 += 8;
  }

  c5_i32 = 0;
  for (c5_i33 = 0; c5_i33 < 2; c5_i33++) {
    for (c5_i34 = 0; c5_i34 < 8; c5_i34++) {
      c5_u[c5_i34 + c5_i32] = c5_b_inData[c5_i34 + c5_i32];
    }

    c5_i32 += 8;
  }

  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", c5_u, 1, 0U, 1U, 0U, 2, 8, 2), false);
  sf_mex_assign(&c5_mxArrayOutData, c5_y, false);
  return c5_mxArrayOutData;
}

static void c5_g_emlrt_marshallIn(SFc5_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId,
  real32_T c5_y[16])
{
  real32_T c5_fv1[16];
  int32_T c5_i35;
  (void)chartInstance;
  sf_mex_import(c5_parentId, sf_mex_dup(c5_u), c5_fv1, 1, 1, 0U, 1, 0U, 2, 8, 2);
  for (c5_i35 = 0; c5_i35 < 16; c5_i35++) {
    c5_y[c5_i35] = c5_fv1[c5_i35];
  }

  sf_mex_destroy(&c5_u);
}

static void c5_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData)
{
  const mxArray *c5_targetPos;
  const char_T *c5_identifier;
  emlrtMsgIdentifier c5_thisId;
  real32_T c5_y[16];
  int32_T c5_i36;
  int32_T c5_i37;
  int32_T c5_i38;
  SFc5_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc5_SimulationModelInstanceStruct *)chartInstanceVoid;
  c5_targetPos = sf_mex_dup(c5_mxArrayInData);
  c5_identifier = c5_varName;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_g_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_targetPos), &c5_thisId,
                        c5_y);
  sf_mex_destroy(&c5_targetPos);
  c5_i36 = 0;
  for (c5_i37 = 0; c5_i37 < 2; c5_i37++) {
    for (c5_i38 = 0; c5_i38 < 8; c5_i38++) {
      (*(real32_T (*)[16])c5_outData)[c5_i38 + c5_i36] = c5_y[c5_i38 + c5_i36];
    }

    c5_i36 += 8;
  }

  sf_mex_destroy(&c5_mxArrayInData);
}

static const mxArray *c5_e_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData)
{
  const mxArray *c5_mxArrayOutData = NULL;
  int32_T c5_i39;
  real_T c5_b_inData[2];
  int32_T c5_i40;
  real_T c5_u[2];
  const mxArray *c5_y = NULL;
  SFc5_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc5_SimulationModelInstanceStruct *)chartInstanceVoid;
  c5_mxArrayOutData = NULL;
  for (c5_i39 = 0; c5_i39 < 2; c5_i39++) {
    c5_b_inData[c5_i39] = (*(real_T (*)[2])c5_inData)[c5_i39];
  }

  for (c5_i40 = 0; c5_i40 < 2; c5_i40++) {
    c5_u[c5_i40] = c5_b_inData[c5_i40];
  }

  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", c5_u, 0, 0U, 1U, 0U, 2, 1, 2), false);
  sf_mex_assign(&c5_mxArrayOutData, c5_y, false);
  return c5_mxArrayOutData;
}

static void c5_h_emlrt_marshallIn(SFc5_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId,
  real_T c5_y[2])
{
  real_T c5_dv4[2];
  int32_T c5_i41;
  (void)chartInstance;
  sf_mex_import(c5_parentId, sf_mex_dup(c5_u), c5_dv4, 1, 0, 0U, 1, 0U, 2, 1, 2);
  for (c5_i41 = 0; c5_i41 < 2; c5_i41++) {
    c5_y[c5_i41] = c5_dv4[c5_i41];
  }

  sf_mex_destroy(&c5_u);
}

static void c5_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData)
{
  const mxArray *c5_robotPos;
  const char_T *c5_identifier;
  emlrtMsgIdentifier c5_thisId;
  real_T c5_y[2];
  int32_T c5_i42;
  SFc5_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc5_SimulationModelInstanceStruct *)chartInstanceVoid;
  c5_robotPos = sf_mex_dup(c5_mxArrayInData);
  c5_identifier = c5_varName;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_robotPos), &c5_thisId, c5_y);
  sf_mex_destroy(&c5_robotPos);
  for (c5_i42 = 0; c5_i42 < 2; c5_i42++) {
    (*(real_T (*)[2])c5_outData)[c5_i42] = c5_y[c5_i42];
  }

  sf_mex_destroy(&c5_mxArrayInData);
}

static const mxArray *c5_f_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData)
{
  const mxArray *c5_mxArrayOutData = NULL;
  int32_T c5_i43;
  real32_T c5_b_inData[8];
  int32_T c5_i44;
  real32_T c5_u[8];
  const mxArray *c5_y = NULL;
  SFc5_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc5_SimulationModelInstanceStruct *)chartInstanceVoid;
  c5_mxArrayOutData = NULL;
  for (c5_i43 = 0; c5_i43 < 8; c5_i43++) {
    c5_b_inData[c5_i43] = (*(real32_T (*)[8])c5_inData)[c5_i43];
  }

  for (c5_i44 = 0; c5_i44 < 8; c5_i44++) {
    c5_u[c5_i44] = c5_b_inData[c5_i44];
  }

  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", c5_u, 1, 0U, 1U, 0U, 1, 8), false);
  sf_mex_assign(&c5_mxArrayOutData, c5_y, false);
  return c5_mxArrayOutData;
}

static void c5_i_emlrt_marshallIn(SFc5_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId,
  real32_T c5_y[8])
{
  real32_T c5_fv2[8];
  int32_T c5_i45;
  (void)chartInstance;
  sf_mex_import(c5_parentId, sf_mex_dup(c5_u), c5_fv2, 1, 1, 0U, 1, 0U, 1, 8);
  for (c5_i45 = 0; c5_i45 < 8; c5_i45++) {
    c5_y[c5_i45] = c5_fv2[c5_i45];
  }

  sf_mex_destroy(&c5_u);
}

static void c5_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData)
{
  const mxArray *c5_dist;
  const char_T *c5_identifier;
  emlrtMsgIdentifier c5_thisId;
  real32_T c5_y[8];
  int32_T c5_i46;
  SFc5_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc5_SimulationModelInstanceStruct *)chartInstanceVoid;
  c5_dist = sf_mex_dup(c5_mxArrayInData);
  c5_identifier = c5_varName;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_i_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_dist), &c5_thisId, c5_y);
  sf_mex_destroy(&c5_dist);
  for (c5_i46 = 0; c5_i46 < 8; c5_i46++) {
    (*(real32_T (*)[8])c5_outData)[c5_i46] = c5_y[c5_i46];
  }

  sf_mex_destroy(&c5_mxArrayInData);
}

const mxArray *sf_c5_SimulationModel_get_eml_resolved_functions_info(void)
{
  const mxArray *c5_nameCaptureInfo = NULL;
  c5_nameCaptureInfo = NULL;
  sf_mex_assign(&c5_nameCaptureInfo, sf_mex_createstruct("structure", 2, 86, 1),
                false);
  c5_info_helper(&c5_nameCaptureInfo);
  c5_b_info_helper(&c5_nameCaptureInfo);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c5_nameCaptureInfo);
  return c5_nameCaptureInfo;
}

static void c5_info_helper(const mxArray **c5_info)
{
  const mxArray *c5_rhs0 = NULL;
  const mxArray *c5_lhs0 = NULL;
  const mxArray *c5_rhs1 = NULL;
  const mxArray *c5_lhs1 = NULL;
  const mxArray *c5_rhs2 = NULL;
  const mxArray *c5_lhs2 = NULL;
  const mxArray *c5_rhs3 = NULL;
  const mxArray *c5_lhs3 = NULL;
  const mxArray *c5_rhs4 = NULL;
  const mxArray *c5_lhs4 = NULL;
  const mxArray *c5_rhs5 = NULL;
  const mxArray *c5_lhs5 = NULL;
  const mxArray *c5_rhs6 = NULL;
  const mxArray *c5_lhs6 = NULL;
  const mxArray *c5_rhs7 = NULL;
  const mxArray *c5_lhs7 = NULL;
  const mxArray *c5_rhs8 = NULL;
  const mxArray *c5_lhs8 = NULL;
  const mxArray *c5_rhs9 = NULL;
  const mxArray *c5_lhs9 = NULL;
  const mxArray *c5_rhs10 = NULL;
  const mxArray *c5_lhs10 = NULL;
  const mxArray *c5_rhs11 = NULL;
  const mxArray *c5_lhs11 = NULL;
  const mxArray *c5_rhs12 = NULL;
  const mxArray *c5_lhs12 = NULL;
  const mxArray *c5_rhs13 = NULL;
  const mxArray *c5_lhs13 = NULL;
  const mxArray *c5_rhs14 = NULL;
  const mxArray *c5_lhs14 = NULL;
  const mxArray *c5_rhs15 = NULL;
  const mxArray *c5_lhs15 = NULL;
  const mxArray *c5_rhs16 = NULL;
  const mxArray *c5_lhs16 = NULL;
  const mxArray *c5_rhs17 = NULL;
  const mxArray *c5_lhs17 = NULL;
  const mxArray *c5_rhs18 = NULL;
  const mxArray *c5_lhs18 = NULL;
  const mxArray *c5_rhs19 = NULL;
  const mxArray *c5_lhs19 = NULL;
  const mxArray *c5_rhs20 = NULL;
  const mxArray *c5_lhs20 = NULL;
  const mxArray *c5_rhs21 = NULL;
  const mxArray *c5_lhs21 = NULL;
  const mxArray *c5_rhs22 = NULL;
  const mxArray *c5_lhs22 = NULL;
  const mxArray *c5_rhs23 = NULL;
  const mxArray *c5_lhs23 = NULL;
  const mxArray *c5_rhs24 = NULL;
  const mxArray *c5_lhs24 = NULL;
  const mxArray *c5_rhs25 = NULL;
  const mxArray *c5_lhs25 = NULL;
  const mxArray *c5_rhs26 = NULL;
  const mxArray *c5_lhs26 = NULL;
  const mxArray *c5_rhs27 = NULL;
  const mxArray *c5_lhs27 = NULL;
  const mxArray *c5_rhs28 = NULL;
  const mxArray *c5_lhs28 = NULL;
  const mxArray *c5_rhs29 = NULL;
  const mxArray *c5_lhs29 = NULL;
  const mxArray *c5_rhs30 = NULL;
  const mxArray *c5_lhs30 = NULL;
  const mxArray *c5_rhs31 = NULL;
  const mxArray *c5_lhs31 = NULL;
  const mxArray *c5_rhs32 = NULL;
  const mxArray *c5_lhs32 = NULL;
  const mxArray *c5_rhs33 = NULL;
  const mxArray *c5_lhs33 = NULL;
  const mxArray *c5_rhs34 = NULL;
  const mxArray *c5_lhs34 = NULL;
  const mxArray *c5_rhs35 = NULL;
  const mxArray *c5_lhs35 = NULL;
  const mxArray *c5_rhs36 = NULL;
  const mxArray *c5_lhs36 = NULL;
  const mxArray *c5_rhs37 = NULL;
  const mxArray *c5_lhs37 = NULL;
  const mxArray *c5_rhs38 = NULL;
  const mxArray *c5_lhs38 = NULL;
  const mxArray *c5_rhs39 = NULL;
  const mxArray *c5_lhs39 = NULL;
  const mxArray *c5_rhs40 = NULL;
  const mxArray *c5_lhs40 = NULL;
  const mxArray *c5_rhs41 = NULL;
  const mxArray *c5_lhs41 = NULL;
  const mxArray *c5_rhs42 = NULL;
  const mxArray *c5_lhs42 = NULL;
  const mxArray *c5_rhs43 = NULL;
  const mxArray *c5_lhs43 = NULL;
  const mxArray *c5_rhs44 = NULL;
  const mxArray *c5_lhs44 = NULL;
  const mxArray *c5_rhs45 = NULL;
  const mxArray *c5_lhs45 = NULL;
  const mxArray *c5_rhs46 = NULL;
  const mxArray *c5_lhs46 = NULL;
  const mxArray *c5_rhs47 = NULL;
  const mxArray *c5_lhs47 = NULL;
  const mxArray *c5_rhs48 = NULL;
  const mxArray *c5_lhs48 = NULL;
  const mxArray *c5_rhs49 = NULL;
  const mxArray *c5_lhs49 = NULL;
  const mxArray *c5_rhs50 = NULL;
  const mxArray *c5_lhs50 = NULL;
  const mxArray *c5_rhs51 = NULL;
  const mxArray *c5_lhs51 = NULL;
  const mxArray *c5_rhs52 = NULL;
  const mxArray *c5_lhs52 = NULL;
  const mxArray *c5_rhs53 = NULL;
  const mxArray *c5_lhs53 = NULL;
  const mxArray *c5_rhs54 = NULL;
  const mxArray *c5_lhs54 = NULL;
  const mxArray *c5_rhs55 = NULL;
  const mxArray *c5_lhs55 = NULL;
  const mxArray *c5_rhs56 = NULL;
  const mxArray *c5_lhs56 = NULL;
  const mxArray *c5_rhs57 = NULL;
  const mxArray *c5_lhs57 = NULL;
  const mxArray *c5_rhs58 = NULL;
  const mxArray *c5_lhs58 = NULL;
  const mxArray *c5_rhs59 = NULL;
  const mxArray *c5_lhs59 = NULL;
  const mxArray *c5_rhs60 = NULL;
  const mxArray *c5_lhs60 = NULL;
  const mxArray *c5_rhs61 = NULL;
  const mxArray *c5_lhs61 = NULL;
  const mxArray *c5_rhs62 = NULL;
  const mxArray *c5_lhs62 = NULL;
  const mxArray *c5_rhs63 = NULL;
  const mxArray *c5_lhs63 = NULL;
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "context", "context", 0);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("bsxfun"), "name", "name", 0);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("function_handle"),
                  "dominantType", "dominantType", 0);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/bsxfun.m"), "resolved",
                  "resolved", 0);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1356541494U), "fileTimeLo",
                  "fileTimeLo", 0);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 0);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 0);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 0);
  sf_mex_assign(&c5_rhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs0), "rhs", "rhs", 0);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs0), "lhs", "lhs", 0);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/bsxfun.m"), "context",
                  "context", 1);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 1);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 1);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 1);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1323170578U), "fileTimeLo",
                  "fileTimeLo", 1);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 1);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 1);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 1);
  sf_mex_assign(&c5_rhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs1), "rhs", "rhs", 1);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs1), "lhs", "lhs", 1);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/bsxfun.m!bsxfun_compatible"),
                  "context", "context", 2);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("min"), "name", "name", 2);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 2);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m"), "resolved",
                  "resolved", 2);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1311255318U), "fileTimeLo",
                  "fileTimeLo", 2);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 2);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 2);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 2);
  sf_mex_assign(&c5_rhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs2), "rhs", "rhs", 2);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs2), "lhs", "lhs", 2);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m"), "context",
                  "context", 3);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_min_or_max"), "name",
                  "name", 3);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 3);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"),
                  "resolved", "resolved", 3);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1378295984U), "fileTimeLo",
                  "fileTimeLo", 3);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 3);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 3);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 3);
  sf_mex_assign(&c5_rhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs3), "rhs", "rhs", 3);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs3), "lhs", "lhs", 3);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum"),
                  "context", "context", 4);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 4);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 4);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 4);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 4);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 4);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 4);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 4);
  sf_mex_assign(&c5_rhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs4), "rhs", "rhs", 4);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs4), "lhs", "lhs", 4);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "context",
                  "context", 5);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 5);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 5);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 5);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307920U), "fileTimeLo",
                  "fileTimeLo", 5);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 5);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 5);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 5);
  sf_mex_assign(&c5_rhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs5), "rhs", "rhs", 5);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs5), "lhs", "lhs", 5);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum"),
                  "context", "context", 6);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_scalexp_alloc"), "name",
                  "name", 6);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 6);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "resolved", "resolved", 6);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 6);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 6);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 6);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 6);
  sf_mex_assign(&c5_rhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs6), "rhs", "rhs", 6);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs6), "lhs", "lhs", 6);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "context", "context", 7);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.scalexpAlloc"),
                  "name", "name", 7);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 7);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalexpAlloc.p"),
                  "resolved", "resolved", 7);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307920U), "fileTimeLo",
                  "fileTimeLo", 7);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 7);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 7);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 7);
  sf_mex_assign(&c5_rhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs7), "rhs", "rhs", 7);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs7), "lhs", "lhs", 7);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum"),
                  "context", "context", 8);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 8);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 8);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 8);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1323170578U), "fileTimeLo",
                  "fileTimeLo", 8);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 8);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 8);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 8);
  sf_mex_assign(&c5_rhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs8), "rhs", "rhs", 8);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs8), "lhs", "lhs", 8);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum"),
                  "context", "context", 9);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 9);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 9);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 9);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 9);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 9);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 9);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 9);
  sf_mex_assign(&c5_rhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs9), "rhs", "rhs", 9);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs9), "lhs", "lhs", 9);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum"),
                  "context", "context", 10);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 10);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 10);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 10);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 10);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 10);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 10);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 10);
  sf_mex_assign(&c5_rhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs10), "rhs", "rhs",
                  10);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs10), "lhs", "lhs",
                  10);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/bsxfun.m"), "context",
                  "context", 11);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("max"), "name", "name", 11);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 11);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/max.m"), "resolved",
                  "resolved", 11);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1311255316U), "fileTimeLo",
                  "fileTimeLo", 11);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 11);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 11);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 11);
  sf_mex_assign(&c5_rhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs11), "rhs", "rhs",
                  11);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs11), "lhs", "lhs",
                  11);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/max.m"), "context",
                  "context", 12);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_min_or_max"), "name",
                  "name", 12);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 12);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"),
                  "resolved", "resolved", 12);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1378295984U), "fileTimeLo",
                  "fileTimeLo", 12);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 12);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 12);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 12);
  sf_mex_assign(&c5_rhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs12), "rhs", "rhs",
                  12);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs12), "lhs", "lhs",
                  12);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/bsxfun.m"), "context",
                  "context", 13);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_index_minus"), "name",
                  "name", 13);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 13);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m"),
                  "resolved", "resolved", 13);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1372582416U), "fileTimeLo",
                  "fileTimeLo", 13);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 13);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 13);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 13);
  sf_mex_assign(&c5_rhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs13), "rhs", "rhs",
                  13);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs13), "lhs", "lhs",
                  13);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m"),
                  "context", "context", 14);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexMinus"),
                  "name", "name", 14);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 14);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexMinus.m"),
                  "resolved", "resolved", 14);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 14);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 14);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 14);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 14);
  sf_mex_assign(&c5_rhs14, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs14, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs14), "rhs", "rhs",
                  14);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs14), "lhs", "lhs",
                  14);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/bsxfun.m"), "context",
                  "context", 15);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 15);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 15);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 15);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 15);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 15);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 15);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 15);
  sf_mex_assign(&c5_rhs15, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs15, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs15), "rhs", "rhs",
                  15);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs15), "lhs", "lhs",
                  15);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "context",
                  "context", 16);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 16);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 16);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 16);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307920U), "fileTimeLo",
                  "fileTimeLo", 16);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 16);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 16);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 16);
  sf_mex_assign(&c5_rhs16, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs16, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs16), "rhs", "rhs",
                  16);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs16), "lhs", "lhs",
                  16);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/bsxfun.m"), "context",
                  "context", 17);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 17);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 17);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 17);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 17);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 17);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 17);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 17);
  sf_mex_assign(&c5_rhs17, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs17, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs17), "rhs", "rhs",
                  17);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs17), "lhs", "lhs",
                  17);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/bsxfun.m!calc_page_back_vector"),
                  "context", "context", 18);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 18);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 18);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 18);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1323170578U), "fileTimeLo",
                  "fileTimeLo", 18);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 18);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 18);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 18);
  sf_mex_assign(&c5_rhs18, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs18, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs18), "rhs", "rhs",
                  18);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs18), "lhs", "lhs",
                  18);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/bsxfun.m"), "context",
                  "context", 19);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 19);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 19);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 19);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 19);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 19);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 19);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 19);
  sf_mex_assign(&c5_rhs19, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs19, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs19), "rhs", "rhs",
                  19);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs19), "lhs", "lhs",
                  19);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper"),
                  "context", "context", 20);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("intmax"), "name", "name", 20);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 20);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "resolved",
                  "resolved", 20);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1362261882U), "fileTimeLo",
                  "fileTimeLo", 20);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 20);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 20);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 20);
  sf_mex_assign(&c5_rhs20, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs20, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs20), "rhs", "rhs",
                  20);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs20), "lhs", "lhs",
                  20);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "context",
                  "context", 21);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 21);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 21);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 21);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1381850300U), "fileTimeLo",
                  "fileTimeLo", 21);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 21);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 21);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 21);
  sf_mex_assign(&c5_rhs21, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs21, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs21), "rhs", "rhs",
                  21);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs21), "lhs", "lhs",
                  21);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/bsxfun.m"), "context",
                  "context", 22);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 22);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 22);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 22);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1372582416U), "fileTimeLo",
                  "fileTimeLo", 22);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 22);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 22);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 22);
  sf_mex_assign(&c5_rhs22, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs22, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs22), "rhs", "rhs",
                  22);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs22), "lhs", "lhs",
                  22);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"), "context",
                  "context", 23);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 23);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 23);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 23);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 23);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 23);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 23);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 23);
  sf_mex_assign(&c5_rhs23, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs23, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs23), "rhs", "rhs",
                  23);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs23), "lhs", "lhs",
                  23);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/bsxfun.m"), "context",
                  "context", 24);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 24);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 24);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 24);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1372582416U), "fileTimeLo",
                  "fileTimeLo", 24);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 24);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 24);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 24);
  sf_mex_assign(&c5_rhs24, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs24, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs24), "rhs", "rhs",
                  24);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs24), "lhs", "lhs",
                  24);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"), "context",
                  "context", 25);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 25);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 25);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 25);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 25);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 25);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 25);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 25);
  sf_mex_assign(&c5_rhs25, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs25, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs25), "rhs", "rhs",
                  25);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs25), "lhs", "lhs",
                  25);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "context", "context", 26);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("power"), "name", "name", 26);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 26);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m"), "resolved",
                  "resolved", 26);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363713880U), "fileTimeLo",
                  "fileTimeLo", 26);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 26);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 26);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 26);
  sf_mex_assign(&c5_rhs26, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs26, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs26), "rhs", "rhs",
                  26);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs26), "lhs", "lhs",
                  26);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m"), "context",
                  "context", 27);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 27);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 27);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 27);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 27);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 27);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 27);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 27);
  sf_mex_assign(&c5_rhs27, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs27, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs27), "rhs", "rhs",
                  27);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs27), "lhs", "lhs",
                  27);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m"), "context",
                  "context", 28);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 28);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 28);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 28);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 28);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 28);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 28);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 28);
  sf_mex_assign(&c5_rhs28, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs28, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs28), "rhs", "rhs",
                  28);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs28), "lhs", "lhs",
                  28);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower"), "context",
                  "context", 29);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 29);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 29);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 29);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 29);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 29);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 29);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 29);
  sf_mex_assign(&c5_rhs29, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs29, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs29), "rhs", "rhs",
                  29);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs29), "lhs", "lhs",
                  29);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower"), "context",
                  "context", 30);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_scalexp_alloc"), "name",
                  "name", 30);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 30);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "resolved", "resolved", 30);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 30);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 30);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 30);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 30);
  sf_mex_assign(&c5_rhs30, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs30, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs30), "rhs", "rhs",
                  30);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs30), "lhs", "lhs",
                  30);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "context", "context", 31);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.scalexpAlloc"),
                  "name", "name", 31);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 31);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalexpAlloc.p"),
                  "resolved", "resolved", 31);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389307920U), "fileTimeLo",
                  "fileTimeLo", 31);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 31);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 31);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 31);
  sf_mex_assign(&c5_rhs31, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs31, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs31), "rhs", "rhs",
                  31);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs31), "lhs", "lhs",
                  31);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower"), "context",
                  "context", 32);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("floor"), "name", "name", 32);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 32);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "resolved",
                  "resolved", 32);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363713854U), "fileTimeLo",
                  "fileTimeLo", 32);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 32);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 32);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 32);
  sf_mex_assign(&c5_rhs32, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs32, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs32), "rhs", "rhs",
                  32);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs32), "lhs", "lhs",
                  32);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "context",
                  "context", 33);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 33);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 33);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 33);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 33);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 33);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 33);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 33);
  sf_mex_assign(&c5_rhs33, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs33, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs33), "rhs", "rhs",
                  33);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs33), "lhs", "lhs",
                  33);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "context",
                  "context", 34);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_scalar_floor"), "name",
                  "name", 34);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 34);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m"),
                  "resolved", "resolved", 34);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1286818726U), "fileTimeLo",
                  "fileTimeLo", 34);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 34);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 34);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 34);
  sf_mex_assign(&c5_rhs34, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs34, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs34), "rhs", "rhs",
                  34);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs34), "lhs", "lhs",
                  34);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!scalar_float_power"),
                  "context", "context", 35);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 35);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 35);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 35);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 35);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 35);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 35);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 35);
  sf_mex_assign(&c5_rhs35, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs35, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs35), "rhs", "rhs",
                  35);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs35), "lhs", "lhs",
                  35);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "context", "context", 36);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("sum"), "name", "name", 36);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 36);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m"), "resolved",
                  "resolved", 36);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363713858U), "fileTimeLo",
                  "fileTimeLo", 36);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 36);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 36);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 36);
  sf_mex_assign(&c5_rhs36, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs36, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs36), "rhs", "rhs",
                  36);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs36), "lhs", "lhs",
                  36);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m"), "context",
                  "context", 37);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 37);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 37);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 37);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 37);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 37);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 37);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 37);
  sf_mex_assign(&c5_rhs37, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs37, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs37), "rhs", "rhs",
                  37);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs37), "lhs", "lhs",
                  37);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m"), "context",
                  "context", 38);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_assert_valid_dim"), "name",
                  "name", 38);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 38);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_dim.m"),
                  "resolved", "resolved", 38);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1372582416U), "fileTimeLo",
                  "fileTimeLo", 38);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 38);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 38);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 38);
  sf_mex_assign(&c5_rhs38, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs38, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs38), "rhs", "rhs",
                  38);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs38), "lhs", "lhs",
                  38);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_dim.m"),
                  "context", "context", 39);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.assertValidDim"),
                  "name", "name", 39);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 39);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assertValidDim.m"),
                  "resolved", "resolved", 39);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 39);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 39);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 39);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 39);
  sf_mex_assign(&c5_rhs39, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs39, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs39), "rhs", "rhs",
                  39);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs39), "lhs", "lhs",
                  39);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assertValidDim.m"),
                  "context", "context", 40);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 40);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 40);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 40);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 40);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 40);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 40);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 40);
  sf_mex_assign(&c5_rhs40, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs40, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs40), "rhs", "rhs",
                  40);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs40), "lhs", "lhs",
                  40);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assertValidDim.m"),
                  "context", "context", 41);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("floor"), "name", "name", 41);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 41);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "resolved",
                  "resolved", 41);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363713854U), "fileTimeLo",
                  "fileTimeLo", 41);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 41);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 41);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 41);
  sf_mex_assign(&c5_rhs41, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs41, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs41), "rhs", "rhs",
                  41);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs41), "lhs", "lhs",
                  41);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assertValidDim.m"),
                  "context", "context", 42);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("intmax"), "name", "name", 42);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 42);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "resolved",
                  "resolved", 42);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1362261882U), "fileTimeLo",
                  "fileTimeLo", 42);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 42);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 42);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 42);
  sf_mex_assign(&c5_rhs42, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs42, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs42), "rhs", "rhs",
                  42);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs42), "lhs", "lhs",
                  42);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m"), "context",
                  "context", 43);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 43);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 43);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 43);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1323170578U), "fileTimeLo",
                  "fileTimeLo", 43);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 43);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 43);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 43);
  sf_mex_assign(&c5_rhs43, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs43, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs43), "rhs", "rhs",
                  43);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs43), "lhs", "lhs",
                  43);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m"), "context",
                  "context", 44);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 44);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 44);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 44);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 44);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 44);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 44);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 44);
  sf_mex_assign(&c5_rhs44, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs44, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs44), "rhs", "rhs",
                  44);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs44), "lhs", "lhs",
                  44);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m"), "context",
                  "context", 45);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_matrix_vstride"), "name",
                  "name", 45);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 45);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_matrix_vstride.m"),
                  "resolved", "resolved", 45);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1360282350U), "fileTimeLo",
                  "fileTimeLo", 45);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 45);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 45);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 45);
  sf_mex_assign(&c5_rhs45, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs45, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs45), "rhs", "rhs",
                  45);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs45), "lhs", "lhs",
                  45);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_matrix_vstride.m"),
                  "context", "context", 46);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.prodsize"),
                  "name", "name", 46);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 46);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/prodsize.m"),
                  "resolved", "resolved", 46);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1360282588U), "fileTimeLo",
                  "fileTimeLo", 46);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 46);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 46);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 46);
  sf_mex_assign(&c5_rhs46, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs46, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs46), "rhs", "rhs",
                  46);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs46), "lhs", "lhs",
                  46);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m"), "context",
                  "context", 47);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_matrix_npages"), "name",
                  "name", 47);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 47);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_matrix_npages.m"),
                  "resolved", "resolved", 47);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1360282350U), "fileTimeLo",
                  "fileTimeLo", 47);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 47);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 47);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 47);
  sf_mex_assign(&c5_rhs47, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs47, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs47), "rhs", "rhs",
                  47);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs47), "lhs", "lhs",
                  47);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_matrix_npages.m"),
                  "context", "context", 48);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.prodsize"),
                  "name", "name", 48);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 48);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/prodsize.m"),
                  "resolved", "resolved", 48);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1360282588U), "fileTimeLo",
                  "fileTimeLo", 48);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 48);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 48);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 48);
  sf_mex_assign(&c5_rhs48, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs48, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs48), "rhs", "rhs",
                  48);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs48), "lhs", "lhs",
                  48);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m"), "context",
                  "context", 49);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 49);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 49);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 49);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 49);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 49);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 49);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 49);
  sf_mex_assign(&c5_rhs49, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs49, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs49), "rhs", "rhs",
                  49);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs49), "lhs", "lhs",
                  49);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m"), "context",
                  "context", 50);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 50);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 50);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 50);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1372582416U), "fileTimeLo",
                  "fileTimeLo", 50);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 50);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 50);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 50);
  sf_mex_assign(&c5_rhs50, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs50, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs50), "rhs", "rhs",
                  50);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs50), "lhs", "lhs",
                  50);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m"), "context",
                  "context", 51);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 51);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 51);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 51);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1372582416U), "fileTimeLo",
                  "fileTimeLo", 51);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 51);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 51);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 51);
  sf_mex_assign(&c5_rhs51, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs51, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs51), "rhs", "rhs",
                  51);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs51), "lhs", "lhs",
                  51);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "context", "context", 52);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("mpower"), "name", "name", 52);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 52);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "resolved",
                  "resolved", 52);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363713878U), "fileTimeLo",
                  "fileTimeLo", 52);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 52);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 52);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 52);
  sf_mex_assign(&c5_rhs52, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs52, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs52), "rhs", "rhs",
                  52);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs52), "lhs", "lhs",
                  52);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "context",
                  "context", 53);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 53);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 53);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 53);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 53);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 53);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 53);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 53);
  sf_mex_assign(&c5_rhs53, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs53, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs53), "rhs", "rhs",
                  53);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs53), "lhs", "lhs",
                  53);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "context",
                  "context", 54);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("ismatrix"), "name", "name", 54);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 54);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/ismatrix.m"), "resolved",
                  "resolved", 54);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1331304858U), "fileTimeLo",
                  "fileTimeLo", 54);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 54);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 54);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 54);
  sf_mex_assign(&c5_rhs54, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs54, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs54), "rhs", "rhs",
                  54);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs54), "lhs", "lhs",
                  54);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "context",
                  "context", 55);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("power"), "name", "name", 55);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 55);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m"), "resolved",
                  "resolved", 55);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363713880U), "fileTimeLo",
                  "fileTimeLo", 55);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 55);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 55);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 55);
  sf_mex_assign(&c5_rhs55, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs55, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs55), "rhs", "rhs",
                  55);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs55), "lhs", "lhs",
                  55);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower"), "context",
                  "context", 56);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 56);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 56);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 56);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 56);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 56);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 56);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 56);
  sf_mex_assign(&c5_rhs56, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs56, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs56), "rhs", "rhs",
                  56);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs56), "lhs", "lhs",
                  56);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower"), "context",
                  "context", 57);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_scalexp_alloc"), "name",
                  "name", 57);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 57);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "resolved", "resolved", 57);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 57);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 57);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 57);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 57);
  sf_mex_assign(&c5_rhs57, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs57, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs57), "rhs", "rhs",
                  57);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs57), "lhs", "lhs",
                  57);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!scalar_float_power"),
                  "context", "context", 58);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 58);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 58);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 58);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 58);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 58);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 58);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 58);
  sf_mex_assign(&c5_rhs58, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs58, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs58), "rhs", "rhs",
                  58);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs58), "lhs", "lhs",
                  58);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "context", "context", 59);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("any"), "name", "name", 59);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("logical"), "dominantType",
                  "dominantType", 59);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/any.m"), "resolved",
                  "resolved", 59);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1372582416U), "fileTimeLo",
                  "fileTimeLo", 59);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 59);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 59);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 59);
  sf_mex_assign(&c5_rhs59, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs59, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs59), "rhs", "rhs",
                  59);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs59), "lhs", "lhs",
                  59);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/any.m"), "context", "context",
                  60);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.assert"),
                  "name", "name", 60);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 60);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assert.m"),
                  "resolved", "resolved", 60);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 60);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 60);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 60);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 60);
  sf_mex_assign(&c5_rhs60, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs60, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs60), "rhs", "rhs",
                  60);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs60), "lhs", "lhs",
                  60);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/any.m"), "context", "context",
                  61);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 61);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("logical"), "dominantType",
                  "dominantType", 61);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 61);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 61);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 61);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 61);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 61);
  sf_mex_assign(&c5_rhs61, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs61, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs61), "rhs", "rhs",
                  61);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs61), "lhs", "lhs",
                  61);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/any.m"), "context", "context",
                  62);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.allOrAny"),
                  "name", "name", 62);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 62);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/allOrAny.m"),
                  "resolved", "resolved", 62);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1372583158U), "fileTimeLo",
                  "fileTimeLo", 62);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 62);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 62);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 62);
  sf_mex_assign(&c5_rhs62, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs62, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs62), "rhs", "rhs",
                  62);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs62), "lhs", "lhs",
                  62);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/allOrAny.m"),
                  "context", "context", 63);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.assert"),
                  "name", "name", 63);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 63);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assert.m"),
                  "resolved", "resolved", 63);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 63);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 63);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 63);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 63);
  sf_mex_assign(&c5_rhs63, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs63, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs63), "rhs", "rhs",
                  63);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs63), "lhs", "lhs",
                  63);
  sf_mex_destroy(&c5_rhs0);
  sf_mex_destroy(&c5_lhs0);
  sf_mex_destroy(&c5_rhs1);
  sf_mex_destroy(&c5_lhs1);
  sf_mex_destroy(&c5_rhs2);
  sf_mex_destroy(&c5_lhs2);
  sf_mex_destroy(&c5_rhs3);
  sf_mex_destroy(&c5_lhs3);
  sf_mex_destroy(&c5_rhs4);
  sf_mex_destroy(&c5_lhs4);
  sf_mex_destroy(&c5_rhs5);
  sf_mex_destroy(&c5_lhs5);
  sf_mex_destroy(&c5_rhs6);
  sf_mex_destroy(&c5_lhs6);
  sf_mex_destroy(&c5_rhs7);
  sf_mex_destroy(&c5_lhs7);
  sf_mex_destroy(&c5_rhs8);
  sf_mex_destroy(&c5_lhs8);
  sf_mex_destroy(&c5_rhs9);
  sf_mex_destroy(&c5_lhs9);
  sf_mex_destroy(&c5_rhs10);
  sf_mex_destroy(&c5_lhs10);
  sf_mex_destroy(&c5_rhs11);
  sf_mex_destroy(&c5_lhs11);
  sf_mex_destroy(&c5_rhs12);
  sf_mex_destroy(&c5_lhs12);
  sf_mex_destroy(&c5_rhs13);
  sf_mex_destroy(&c5_lhs13);
  sf_mex_destroy(&c5_rhs14);
  sf_mex_destroy(&c5_lhs14);
  sf_mex_destroy(&c5_rhs15);
  sf_mex_destroy(&c5_lhs15);
  sf_mex_destroy(&c5_rhs16);
  sf_mex_destroy(&c5_lhs16);
  sf_mex_destroy(&c5_rhs17);
  sf_mex_destroy(&c5_lhs17);
  sf_mex_destroy(&c5_rhs18);
  sf_mex_destroy(&c5_lhs18);
  sf_mex_destroy(&c5_rhs19);
  sf_mex_destroy(&c5_lhs19);
  sf_mex_destroy(&c5_rhs20);
  sf_mex_destroy(&c5_lhs20);
  sf_mex_destroy(&c5_rhs21);
  sf_mex_destroy(&c5_lhs21);
  sf_mex_destroy(&c5_rhs22);
  sf_mex_destroy(&c5_lhs22);
  sf_mex_destroy(&c5_rhs23);
  sf_mex_destroy(&c5_lhs23);
  sf_mex_destroy(&c5_rhs24);
  sf_mex_destroy(&c5_lhs24);
  sf_mex_destroy(&c5_rhs25);
  sf_mex_destroy(&c5_lhs25);
  sf_mex_destroy(&c5_rhs26);
  sf_mex_destroy(&c5_lhs26);
  sf_mex_destroy(&c5_rhs27);
  sf_mex_destroy(&c5_lhs27);
  sf_mex_destroy(&c5_rhs28);
  sf_mex_destroy(&c5_lhs28);
  sf_mex_destroy(&c5_rhs29);
  sf_mex_destroy(&c5_lhs29);
  sf_mex_destroy(&c5_rhs30);
  sf_mex_destroy(&c5_lhs30);
  sf_mex_destroy(&c5_rhs31);
  sf_mex_destroy(&c5_lhs31);
  sf_mex_destroy(&c5_rhs32);
  sf_mex_destroy(&c5_lhs32);
  sf_mex_destroy(&c5_rhs33);
  sf_mex_destroy(&c5_lhs33);
  sf_mex_destroy(&c5_rhs34);
  sf_mex_destroy(&c5_lhs34);
  sf_mex_destroy(&c5_rhs35);
  sf_mex_destroy(&c5_lhs35);
  sf_mex_destroy(&c5_rhs36);
  sf_mex_destroy(&c5_lhs36);
  sf_mex_destroy(&c5_rhs37);
  sf_mex_destroy(&c5_lhs37);
  sf_mex_destroy(&c5_rhs38);
  sf_mex_destroy(&c5_lhs38);
  sf_mex_destroy(&c5_rhs39);
  sf_mex_destroy(&c5_lhs39);
  sf_mex_destroy(&c5_rhs40);
  sf_mex_destroy(&c5_lhs40);
  sf_mex_destroy(&c5_rhs41);
  sf_mex_destroy(&c5_lhs41);
  sf_mex_destroy(&c5_rhs42);
  sf_mex_destroy(&c5_lhs42);
  sf_mex_destroy(&c5_rhs43);
  sf_mex_destroy(&c5_lhs43);
  sf_mex_destroy(&c5_rhs44);
  sf_mex_destroy(&c5_lhs44);
  sf_mex_destroy(&c5_rhs45);
  sf_mex_destroy(&c5_lhs45);
  sf_mex_destroy(&c5_rhs46);
  sf_mex_destroy(&c5_lhs46);
  sf_mex_destroy(&c5_rhs47);
  sf_mex_destroy(&c5_lhs47);
  sf_mex_destroy(&c5_rhs48);
  sf_mex_destroy(&c5_lhs48);
  sf_mex_destroy(&c5_rhs49);
  sf_mex_destroy(&c5_lhs49);
  sf_mex_destroy(&c5_rhs50);
  sf_mex_destroy(&c5_lhs50);
  sf_mex_destroy(&c5_rhs51);
  sf_mex_destroy(&c5_lhs51);
  sf_mex_destroy(&c5_rhs52);
  sf_mex_destroy(&c5_lhs52);
  sf_mex_destroy(&c5_rhs53);
  sf_mex_destroy(&c5_lhs53);
  sf_mex_destroy(&c5_rhs54);
  sf_mex_destroy(&c5_lhs54);
  sf_mex_destroy(&c5_rhs55);
  sf_mex_destroy(&c5_lhs55);
  sf_mex_destroy(&c5_rhs56);
  sf_mex_destroy(&c5_lhs56);
  sf_mex_destroy(&c5_rhs57);
  sf_mex_destroy(&c5_lhs57);
  sf_mex_destroy(&c5_rhs58);
  sf_mex_destroy(&c5_lhs58);
  sf_mex_destroy(&c5_rhs59);
  sf_mex_destroy(&c5_lhs59);
  sf_mex_destroy(&c5_rhs60);
  sf_mex_destroy(&c5_lhs60);
  sf_mex_destroy(&c5_rhs61);
  sf_mex_destroy(&c5_lhs61);
  sf_mex_destroy(&c5_rhs62);
  sf_mex_destroy(&c5_lhs62);
  sf_mex_destroy(&c5_rhs63);
  sf_mex_destroy(&c5_lhs63);
}

static const mxArray *c5_emlrt_marshallOut(const char * c5_u)
{
  const mxArray *c5_y = NULL;
  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", c5_u, 15, 0U, 0U, 0U, 2, 1, strlen
    (c5_u)), false);
  return c5_y;
}

static const mxArray *c5_b_emlrt_marshallOut(const uint32_T c5_u)
{
  const mxArray *c5_y = NULL;
  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", &c5_u, 7, 0U, 0U, 0U, 0), false);
  return c5_y;
}

static void c5_b_info_helper(const mxArray **c5_info)
{
  const mxArray *c5_rhs64 = NULL;
  const mxArray *c5_lhs64 = NULL;
  const mxArray *c5_rhs65 = NULL;
  const mxArray *c5_lhs65 = NULL;
  const mxArray *c5_rhs66 = NULL;
  const mxArray *c5_lhs66 = NULL;
  const mxArray *c5_rhs67 = NULL;
  const mxArray *c5_lhs67 = NULL;
  const mxArray *c5_rhs68 = NULL;
  const mxArray *c5_lhs68 = NULL;
  const mxArray *c5_rhs69 = NULL;
  const mxArray *c5_lhs69 = NULL;
  const mxArray *c5_rhs70 = NULL;
  const mxArray *c5_lhs70 = NULL;
  const mxArray *c5_rhs71 = NULL;
  const mxArray *c5_lhs71 = NULL;
  const mxArray *c5_rhs72 = NULL;
  const mxArray *c5_lhs72 = NULL;
  const mxArray *c5_rhs73 = NULL;
  const mxArray *c5_lhs73 = NULL;
  const mxArray *c5_rhs74 = NULL;
  const mxArray *c5_lhs74 = NULL;
  const mxArray *c5_rhs75 = NULL;
  const mxArray *c5_lhs75 = NULL;
  const mxArray *c5_rhs76 = NULL;
  const mxArray *c5_lhs76 = NULL;
  const mxArray *c5_rhs77 = NULL;
  const mxArray *c5_lhs77 = NULL;
  const mxArray *c5_rhs78 = NULL;
  const mxArray *c5_lhs78 = NULL;
  const mxArray *c5_rhs79 = NULL;
  const mxArray *c5_lhs79 = NULL;
  const mxArray *c5_rhs80 = NULL;
  const mxArray *c5_lhs80 = NULL;
  const mxArray *c5_rhs81 = NULL;
  const mxArray *c5_lhs81 = NULL;
  const mxArray *c5_rhs82 = NULL;
  const mxArray *c5_lhs82 = NULL;
  const mxArray *c5_rhs83 = NULL;
  const mxArray *c5_lhs83 = NULL;
  const mxArray *c5_rhs84 = NULL;
  const mxArray *c5_lhs84 = NULL;
  const mxArray *c5_rhs85 = NULL;
  const mxArray *c5_lhs85 = NULL;
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/allOrAny.m"),
                  "context", "context", 64);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("isequal"), "name", "name", 64);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 64);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isequal.m"), "resolved",
                  "resolved", 64);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1286818758U), "fileTimeLo",
                  "fileTimeLo", 64);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 64);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 64);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 64);
  sf_mex_assign(&c5_rhs64, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs64, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs64), "rhs", "rhs",
                  64);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs64), "lhs", "lhs",
                  64);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isequal.m"), "context",
                  "context", 65);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_isequal_core"), "name",
                  "name", 65);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 65);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isequal_core.m"),
                  "resolved", "resolved", 65);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1286818786U), "fileTimeLo",
                  "fileTimeLo", 65);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 65);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 65);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 65);
  sf_mex_assign(&c5_rhs65, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs65, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs65), "rhs", "rhs",
                  65);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs65), "lhs", "lhs",
                  65);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/allOrAny.m"),
                  "context", "context", 66);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "coder.internal.constNonSingletonDim"), "name", "name", 66);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("logical"), "dominantType",
                  "dominantType", 66);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/constNonSingletonDim.m"),
                  "resolved", "resolved", 66);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 66);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 66);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 66);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 66);
  sf_mex_assign(&c5_rhs66, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs66, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs66), "rhs", "rhs",
                  66);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs66), "lhs", "lhs",
                  66);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/allOrAny.m"),
                  "context", "context", 67);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("isnan"), "name", "name", 67);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("logical"), "dominantType",
                  "dominantType", 67);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m"), "resolved",
                  "resolved", 67);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363713858U), "fileTimeLo",
                  "fileTimeLo", 67);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 67);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 67);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 67);
  sf_mex_assign(&c5_rhs67, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs67, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs67), "rhs", "rhs",
                  67);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs67), "lhs", "lhs",
                  67);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m"), "context",
                  "context", 68);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 68);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("logical"), "dominantType",
                  "dominantType", 68);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 68);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 68);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 68);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 68);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 68);
  sf_mex_assign(&c5_rhs68, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs68, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs68), "rhs", "rhs",
                  68);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs68), "lhs", "lhs",
                  68);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "context", "context", 69);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("sum"), "name", "name", 69);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("logical"), "dominantType",
                  "dominantType", 69);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m"), "resolved",
                  "resolved", 69);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363713858U), "fileTimeLo",
                  "fileTimeLo", 69);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 69);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 69);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 69);
  sf_mex_assign(&c5_rhs69, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs69, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs69), "rhs", "rhs",
                  69);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs69), "lhs", "lhs",
                  69);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m"), "context",
                  "context", 70);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 70);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("logical"), "dominantType",
                  "dominantType", 70);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 70);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 70);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 70);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 70);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 70);
  sf_mex_assign(&c5_rhs70, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs70, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs70), "rhs", "rhs",
                  70);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs70), "lhs", "lhs",
                  70);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m"), "context",
                  "context", 71);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("isequal"), "name", "name", 71);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 71);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isequal.m"), "resolved",
                  "resolved", 71);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1286818758U), "fileTimeLo",
                  "fileTimeLo", 71);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 71);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 71);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 71);
  sf_mex_assign(&c5_rhs71, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs71, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs71), "rhs", "rhs",
                  71);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs71), "lhs", "lhs",
                  71);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m"), "context",
                  "context", 72);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_const_nonsingleton_dim"),
                  "name", "name", 72);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("logical"), "dominantType",
                  "dominantType", 72);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_const_nonsingleton_dim.m"),
                  "resolved", "resolved", 72);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1372582416U), "fileTimeLo",
                  "fileTimeLo", 72);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 72);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 72);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 72);
  sf_mex_assign(&c5_rhs72, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs72, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs72), "rhs", "rhs",
                  72);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs72), "lhs", "lhs",
                  72);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_const_nonsingleton_dim.m"),
                  "context", "context", 73);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "coder.internal.constNonSingletonDim"), "name", "name", 73);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("logical"), "dominantType",
                  "dominantType", 73);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/constNonSingletonDim.m"),
                  "resolved", "resolved", 73);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 73);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 73);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 73);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 73);
  sf_mex_assign(&c5_rhs73, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs73, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs73), "rhs", "rhs",
                  73);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs73), "lhs", "lhs",
                  73);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m"), "context",
                  "context", 74);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 74);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 74);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 74);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 74);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 74);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 74);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 74);
  sf_mex_assign(&c5_rhs74, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs74, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs74), "rhs", "rhs",
                  74);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs74), "lhs", "lhs",
                  74);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "context", "context", 75);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("power"), "name", "name", 75);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 75);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m"), "resolved",
                  "resolved", 75);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363713880U), "fileTimeLo",
                  "fileTimeLo", 75);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 75);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 75);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 75);
  sf_mex_assign(&c5_rhs75, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs75, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs75), "rhs", "rhs",
                  75);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs75), "lhs", "lhs",
                  75);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "context", "context", 76);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("sum"), "name", "name", 76);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 76);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m"), "resolved",
                  "resolved", 76);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363713858U), "fileTimeLo",
                  "fileTimeLo", 76);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 76);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 76);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 76);
  sf_mex_assign(&c5_rhs76, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs76, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs76), "rhs", "rhs",
                  76);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs76), "lhs", "lhs",
                  76);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m"), "context",
                  "context", 77);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 77);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 77);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 77);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 77);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 77);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 77);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 77);
  sf_mex_assign(&c5_rhs77, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs77, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs77), "rhs", "rhs",
                  77);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs77), "lhs", "lhs",
                  77);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m"), "context",
                  "context", 78);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_const_nonsingleton_dim"),
                  "name", "name", 78);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 78);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_const_nonsingleton_dim.m"),
                  "resolved", "resolved", 78);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1372582416U), "fileTimeLo",
                  "fileTimeLo", 78);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 78);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 78);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 78);
  sf_mex_assign(&c5_rhs78, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs78, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs78), "rhs", "rhs",
                  78);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs78), "lhs", "lhs",
                  78);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_const_nonsingleton_dim.m"),
                  "context", "context", 79);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "coder.internal.constNonSingletonDim"), "name", "name", 79);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 79);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/constNonSingletonDim.m"),
                  "resolved", "resolved", 79);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 79);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 79);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 79);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 79);
  sf_mex_assign(&c5_rhs79, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs79, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs79), "rhs", "rhs",
                  79);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs79), "lhs", "lhs",
                  79);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "context", "context", 80);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("sqrt"), "name", "name", 80);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 80);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "resolved",
                  "resolved", 80);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1343830386U), "fileTimeLo",
                  "fileTimeLo", 80);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 80);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 80);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 80);
  sf_mex_assign(&c5_rhs80, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs80, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs80), "rhs", "rhs",
                  80);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs80), "lhs", "lhs",
                  80);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "context",
                  "context", 81);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_error"), "name", "name",
                  81);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 81);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m"), "resolved",
                  "resolved", 81);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1343830358U), "fileTimeLo",
                  "fileTimeLo", 81);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 81);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 81);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 81);
  sf_mex_assign(&c5_rhs81, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs81, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs81), "rhs", "rhs",
                  81);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs81), "lhs", "lhs",
                  81);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "context",
                  "context", 82);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_scalar_sqrt"), "name",
                  "name", 82);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 82);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sqrt.m"),
                  "resolved", "resolved", 82);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1286818738U), "fileTimeLo",
                  "fileTimeLo", 82);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 82);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 82);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 82);
  sf_mex_assign(&c5_rhs82, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs82, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs82), "rhs", "rhs",
                  82);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs82), "lhs", "lhs",
                  82);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "context", "context", 83);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eps"), "name", "name", 83);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 83);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m"), "resolved",
                  "resolved", 83);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1326727996U), "fileTimeLo",
                  "fileTimeLo", 83);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 83);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 83);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 83);
  sf_mex_assign(&c5_rhs83, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs83, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs83), "rhs", "rhs",
                  83);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs83), "lhs", "lhs",
                  83);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m"), "context",
                  "context", 84);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_eps"), "name", "name", 84);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 84);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m"), "resolved",
                  "resolved", 84);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1326727996U), "fileTimeLo",
                  "fileTimeLo", 84);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 84);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 84);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 84);
  sf_mex_assign(&c5_rhs84, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs84, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs84), "rhs", "rhs",
                  84);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs84), "lhs", "lhs",
                  84);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m"), "context",
                  "context", 85);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_float_model"), "name",
                  "name", 85);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 85);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m"),
                  "resolved", "resolved", 85);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1326727996U), "fileTimeLo",
                  "fileTimeLo", 85);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 85);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 85);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 85);
  sf_mex_assign(&c5_rhs85, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs85, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs85), "rhs", "rhs",
                  85);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs85), "lhs", "lhs",
                  85);
  sf_mex_destroy(&c5_rhs64);
  sf_mex_destroy(&c5_lhs64);
  sf_mex_destroy(&c5_rhs65);
  sf_mex_destroy(&c5_lhs65);
  sf_mex_destroy(&c5_rhs66);
  sf_mex_destroy(&c5_lhs66);
  sf_mex_destroy(&c5_rhs67);
  sf_mex_destroy(&c5_lhs67);
  sf_mex_destroy(&c5_rhs68);
  sf_mex_destroy(&c5_lhs68);
  sf_mex_destroy(&c5_rhs69);
  sf_mex_destroy(&c5_lhs69);
  sf_mex_destroy(&c5_rhs70);
  sf_mex_destroy(&c5_lhs70);
  sf_mex_destroy(&c5_rhs71);
  sf_mex_destroy(&c5_lhs71);
  sf_mex_destroy(&c5_rhs72);
  sf_mex_destroy(&c5_lhs72);
  sf_mex_destroy(&c5_rhs73);
  sf_mex_destroy(&c5_lhs73);
  sf_mex_destroy(&c5_rhs74);
  sf_mex_destroy(&c5_lhs74);
  sf_mex_destroy(&c5_rhs75);
  sf_mex_destroy(&c5_lhs75);
  sf_mex_destroy(&c5_rhs76);
  sf_mex_destroy(&c5_lhs76);
  sf_mex_destroy(&c5_rhs77);
  sf_mex_destroy(&c5_lhs77);
  sf_mex_destroy(&c5_rhs78);
  sf_mex_destroy(&c5_lhs78);
  sf_mex_destroy(&c5_rhs79);
  sf_mex_destroy(&c5_lhs79);
  sf_mex_destroy(&c5_rhs80);
  sf_mex_destroy(&c5_lhs80);
  sf_mex_destroy(&c5_rhs81);
  sf_mex_destroy(&c5_lhs81);
  sf_mex_destroy(&c5_rhs82);
  sf_mex_destroy(&c5_lhs82);
  sf_mex_destroy(&c5_rhs83);
  sf_mex_destroy(&c5_lhs83);
  sf_mex_destroy(&c5_rhs84);
  sf_mex_destroy(&c5_lhs84);
  sf_mex_destroy(&c5_rhs85);
  sf_mex_destroy(&c5_lhs85);
}

static void c5_startTiming(SFc5_SimulationModelInstanceStruct *chartInstance)
{
  uint32_T c5_debug_family_var_map[2];
  real_T c5_nargin = 0.0;
  real_T c5_nargout = 0.0;
  real_T *c5_sim_time;
  c5_sim_time = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c5_debug_family_names,
    c5_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_nargin, 0U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_nargout, 1U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  CV_EML_FCN(7, 0);
  _SFD_EML_CALL(7U, chartInstance->c5_sfEvent, 2);
  _SFD_EML_CALL(7U, chartInstance->c5_sfEvent, 4);
  CV_EML_IF(7, 1, 0, false);
  _SFD_EML_CALL(7U, chartInstance->c5_sfEvent, 7);
  chartInstance->local_t = *c5_sim_time;
  c5_updateDataWrittenToVector(chartInstance, 12U);
  _SFD_EML_CALL(7U, chartInstance->c5_sfEvent, -7);
  _SFD_SYMBOL_SCOPE_POP();
}

static real_T c5_getTiming(SFc5_SimulationModelInstanceStruct *chartInstance)
{
  real_T c5_et;
  uint32_T c5_debug_family_var_map[4];
  real_T c5_lt;
  real_T c5_nargin = 0.0;
  real_T c5_nargout = 1.0;
  real_T *c5_sim_time;
  c5_sim_time = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 4U, 4U, c5_e_debug_family_names,
    c5_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_lt, 0U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_nargin, 1U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_nargout, 2U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_et, 3U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  CV_EML_FCN(6, 0);
  _SFD_EML_CALL(6U, chartInstance->c5_sfEvent, 2);
  _SFD_EML_CALL(6U, chartInstance->c5_sfEvent, 4);
  c5_lt = 0.0;
  _SFD_EML_CALL(6U, chartInstance->c5_sfEvent, 6);
  CV_EML_IF(6, 1, 0, false);
  _SFD_EML_CALL(6U, chartInstance->c5_sfEvent, 9);
  c5_errorIfDataNotWrittenToFcn(chartInstance, 12U, 16U);
  c5_lt = *c5_sim_time - chartInstance->local_t;
  _SFD_EML_CALL(6U, chartInstance->c5_sfEvent, 12);
  c5_et = c5_lt;
  _SFD_EML_CALL(6U, chartInstance->c5_sfEvent, -12);
  _SFD_SYMBOL_SCOPE_POP();
  return c5_et;
}

static void c5_testIfNearLocation(SFc5_SimulationModelInstanceStruct
  *chartInstance, real_T c5_robotPos[2], real32_T c5_targetPos[16], real_T
  c5_proximityVal, boolean_T *c5_nearLoc, boolean_T c5_currentLoc[8])
{
  uint32_T c5_debug_family_var_map[9];
  real32_T c5_delta[16];
  real32_T c5_dist[8];
  real_T c5_nargin = 3.0;
  real_T c5_nargout = 2.0;
  int32_T c5_i47;
  real32_T c5_b_targetPos[16];
  int32_T c5_i48;
  real_T c5_b_robotPos[2];
  real32_T c5_fv3[16];
  int32_T c5_i49;
  int32_T c5_i50;
  real32_T c5_a[16];
  int32_T c5_k;
  real_T c5_b_k;
  real32_T c5_ak;
  real32_T c5_b_a;
  real32_T c5_y;
  real32_T c5_b_y[16];
  int32_T c5_iy;
  int32_T c5_ixstart;
  int32_T c5_j;
  int32_T c5_c_a;
  int32_T c5_d_a;
  int32_T c5_ix;
  real32_T c5_s;
  int32_T c5_e_a;
  int32_T c5_f_a;
  int32_T c5_g_a;
  int32_T c5_h_a;
  real_T c5_i_a;
  real_T c5_j_a;
  real_T c5_k_a;
  real_T c5_b_ak;
  real_T c5_l_a;
  real_T c5_c;
  int32_T c5_i51;
  int32_T c5_i52;
  boolean_T c5_b_currentLoc[8];
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 9U, 9U, c5_f_debug_family_names,
    c5_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c5_delta, 0U, c5_d_sf_marshallOut,
    c5_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c5_dist, 1U, c5_f_sf_marshallOut,
    c5_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_nargin, 2U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_nargout, 3U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c5_robotPos, 4U, c5_e_sf_marshallOut,
    c5_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c5_targetPos, 5U, c5_d_sf_marshallOut,
    c5_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_proximityVal, 6U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c5_nearLoc, 7U, c5_c_sf_marshallOut,
    c5_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c5_currentLoc, 8U, c5_b_sf_marshallOut,
    c5_b_sf_marshallIn);
  CV_EML_FCN(8, 0);
  _SFD_EML_CALL(8U, chartInstance->c5_sfEvent, 3);
  for (c5_i47 = 0; c5_i47 < 16; c5_i47++) {
    c5_b_targetPos[c5_i47] = c5_targetPos[c5_i47];
  }

  for (c5_i48 = 0; c5_i48 < 2; c5_i48++) {
    c5_b_robotPos[c5_i48] = c5_robotPos[c5_i48];
  }

  c5_bsxfun(chartInstance, c5_b_targetPos, c5_b_robotPos, c5_fv3);
  for (c5_i49 = 0; c5_i49 < 16; c5_i49++) {
    c5_delta[c5_i49] = c5_fv3[c5_i49];
  }

  _SFD_EML_CALL(8U, chartInstance->c5_sfEvent, 4);
  for (c5_i50 = 0; c5_i50 < 16; c5_i50++) {
    c5_a[c5_i50] = c5_delta[c5_i50];
  }

  for (c5_k = 0; c5_k < 16; c5_k++) {
    c5_b_k = 1.0 + (real_T)c5_k;
    c5_ak = c5_a[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      c5_b_k), 1, 16, 1, 0) - 1];
    c5_b_a = c5_ak;
    c5_y = c5_b_a * c5_b_a;
    c5_b_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      c5_b_k), 1, 16, 1, 0) - 1] = c5_y;
  }

  c5_iy = 0;
  c5_ixstart = 0;
  for (c5_j = 1; c5_j < 9; c5_j++) {
    c5_c_a = c5_ixstart;
    c5_d_a = c5_c_a + 1;
    c5_ixstart = c5_d_a;
    c5_ix = c5_ixstart;
    c5_s = c5_b_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c5_ix), 1, 16, 1, 0) - 1];
    c5_e_a = c5_ix;
    c5_f_a = c5_e_a;
    c5_ix = c5_f_a;
    c5_s += c5_b_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK(
      "", (real_T)(c5_ix + 8)), 1, 16, 1, 0) - 1];
    c5_g_a = c5_iy;
    c5_h_a = c5_g_a + 1;
    c5_iy = c5_h_a;
    c5_dist[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      (real_T)c5_iy), 1, 8, 1, 0) - 1] = c5_s;
  }

  _SFD_EML_CALL(8U, chartInstance->c5_sfEvent, 6);
  c5_i_a = c5_proximityVal;
  c5_j_a = c5_i_a;
  c5_k_a = c5_j_a;
  c5_eml_scalar_eg(chartInstance);
  c5_b_ak = c5_k_a;
  c5_l_a = c5_b_ak;
  c5_eml_scalar_eg(chartInstance);
  c5_c = c5_l_a * c5_l_a;
  for (c5_i51 = 0; c5_i51 < 8; c5_i51++) {
    c5_currentLoc[c5_i51] = ((real_T)c5_dist[c5_i51] <= c5_c);
  }

  _SFD_EML_CALL(8U, chartInstance->c5_sfEvent, 7);
  for (c5_i52 = 0; c5_i52 < 8; c5_i52++) {
    c5_b_currentLoc[c5_i52] = c5_currentLoc[c5_i52];
  }

  *c5_nearLoc = c5_any(chartInstance, c5_b_currentLoc);
  _SFD_EML_CALL(8U, chartInstance->c5_sfEvent, -7);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c5_updateTargetsFound(SFc5_SimulationModelInstanceStruct
  *chartInstance, boolean_T c5_tfound_in[8], boolean_T c5_currentT[8], real_T
  c5_lastTime_in, real_T c5_time, boolean_T c5_tfound_out[8], real_T
  *c5_lastTime_out)
{
  uint32_T c5_debug_family_var_map[8];
  real_T c5_nargin = 4.0;
  real_T c5_nargout = 2.0;
  int32_T c5_i53;
  boolean_T c5_b_tfound_in[8];
  int32_T c5_i54;
  boolean_T c5_b_currentT[8];
  int32_T c5_i55;
  boolean_T c5_c_tfound_in[8];
  int32_T c5_i56;
  int32_T c5_i57;
  int32_T c5_i58;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 8U, 8U, c5_l_debug_family_names,
    c5_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_nargin, 0U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_nargout, 1U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c5_tfound_in, 2U, c5_b_sf_marshallOut,
    c5_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c5_currentT, 3U, c5_b_sf_marshallOut,
    c5_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_lastTime_in, 4U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_time, 5U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c5_tfound_out, 6U, c5_b_sf_marshallOut,
    c5_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c5_lastTime_out, 7U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  CV_EML_FCN(9, 0);
  _SFD_EML_CALL(9U, chartInstance->c5_sfEvent, 3);
  for (c5_i53 = 0; c5_i53 < 8; c5_i53++) {
    c5_b_tfound_in[c5_i53] = c5_tfound_in[c5_i53];
  }

  for (c5_i54 = 0; c5_i54 < 8; c5_i54++) {
    c5_b_currentT[c5_i54] = c5_currentT[c5_i54];
  }

  for (c5_i55 = 0; c5_i55 < 8; c5_i55++) {
    c5_c_tfound_in[c5_i55] = (c5_b_tfound_in[c5_i55] && c5_b_currentT[c5_i55]);
  }

  if (CV_EML_IF(9, 1, 0, c5_any(chartInstance, c5_c_tfound_in))) {
    _SFD_EML_CALL(9U, chartInstance->c5_sfEvent, 4);
    *c5_lastTime_out = c5_lastTime_in;
  } else {
    _SFD_EML_CALL(9U, chartInstance->c5_sfEvent, 6);
    *c5_lastTime_out = c5_time;
  }

  _SFD_EML_CALL(9U, chartInstance->c5_sfEvent, 9);
  for (c5_i56 = 0; c5_i56 < 8; c5_i56++) {
    c5_b_tfound_in[c5_i56] = c5_tfound_in[c5_i56];
  }

  for (c5_i57 = 0; c5_i57 < 8; c5_i57++) {
    c5_b_currentT[c5_i57] = c5_currentT[c5_i57];
  }

  for (c5_i58 = 0; c5_i58 < 8; c5_i58++) {
    c5_tfound_out[c5_i58] = (c5_b_tfound_in[c5_i58] || c5_b_currentT[c5_i58]);
  }

  _SFD_EML_CALL(9U, chartInstance->c5_sfEvent, -9);
  _SFD_SYMBOL_SCOPE_POP();
}

static real_T c5_distancefrom(SFc5_SimulationModelInstanceStruct *chartInstance,
  real_T c5_pos[2], real_T c5_x0[2])
{
  real_T c5_d;
  uint32_T c5_debug_family_var_map[5];
  real_T c5_nargin = 2.0;
  real_T c5_nargout = 1.0;
  int32_T c5_i59;
  real_T c5_a[2];
  int32_T c5_k;
  real_T c5_b_k;
  real_T c5_ak;
  real_T c5_b_a;
  real_T c5_y;
  real_T c5_b_y[2];
  real_T c5_c_y;
  real_T c5_x;
  real_T c5_b_x;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 5U, 5U, c5_o_debug_family_names,
    c5_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_nargin, 0U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_nargout, 1U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c5_pos, 2U, c5_e_sf_marshallOut,
    c5_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c5_x0, 3U, c5_e_sf_marshallOut,
    c5_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_d, 4U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  CV_EML_FCN(5, 0);
  _SFD_EML_CALL(5U, chartInstance->c5_sfEvent, 2);
  for (c5_i59 = 0; c5_i59 < 2; c5_i59++) {
    c5_a[c5_i59] = c5_pos[c5_i59] - c5_x0[c5_i59];
  }

  for (c5_k = 0; c5_k < 2; c5_k++) {
    c5_b_k = 1.0 + (real_T)c5_k;
    c5_ak = c5_a[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      c5_b_k), 1, 2, 1, 0) - 1];
    c5_b_a = c5_ak;
    c5_eml_scalar_eg(chartInstance);
    c5_y = c5_b_a * c5_b_a;
    c5_b_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
      c5_b_k), 1, 2, 1, 0) - 1] = c5_y;
  }

  c5_c_y = c5_b_y[0];
  c5_c_y += c5_b_y[1];
  c5_x = c5_c_y;
  c5_d = c5_x;
  if (c5_d < 0.0) {
    c5_eml_error(chartInstance);
  }

  c5_b_x = c5_d;
  c5_d = c5_b_x;
  c5_d = muDoubleScalarSqrt(c5_d);
  _SFD_EML_CALL(5U, chartInstance->c5_sfEvent, -2);
  _SFD_SYMBOL_SCOPE_POP();
  return c5_d;
}

static const mxArray *c5_g_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData)
{
  const mxArray *c5_mxArrayOutData = NULL;
  int32_T c5_u;
  const mxArray *c5_y = NULL;
  SFc5_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc5_SimulationModelInstanceStruct *)chartInstanceVoid;
  c5_mxArrayOutData = NULL;
  c5_u = *(int32_T *)c5_inData;
  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", &c5_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c5_mxArrayOutData, c5_y, false);
  return c5_mxArrayOutData;
}

static int32_T c5_j_emlrt_marshallIn(SFc5_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId)
{
  int32_T c5_y;
  int32_T c5_i60;
  (void)chartInstance;
  sf_mex_import(c5_parentId, sf_mex_dup(c5_u), &c5_i60, 1, 6, 0U, 0, 0U, 0);
  c5_y = c5_i60;
  sf_mex_destroy(&c5_u);
  return c5_y;
}

static void c5_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData)
{
  const mxArray *c5_b_sfEvent;
  const char_T *c5_identifier;
  emlrtMsgIdentifier c5_thisId;
  int32_T c5_y;
  SFc5_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc5_SimulationModelInstanceStruct *)chartInstanceVoid;
  c5_b_sfEvent = sf_mex_dup(c5_mxArrayInData);
  c5_identifier = c5_varName;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_y = c5_j_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_b_sfEvent),
    &c5_thisId);
  sf_mex_destroy(&c5_b_sfEvent);
  *(int32_T *)c5_outData = c5_y;
  sf_mex_destroy(&c5_mxArrayInData);
}

static const mxArray *c5_h_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData)
{
  const mxArray *c5_mxArrayOutData = NULL;
  uint8_T c5_u;
  const mxArray *c5_y = NULL;
  SFc5_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc5_SimulationModelInstanceStruct *)chartInstanceVoid;
  c5_mxArrayOutData = NULL;
  c5_u = *(uint8_T *)c5_inData;
  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", &c5_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c5_mxArrayOutData, c5_y, false);
  return c5_mxArrayOutData;
}

static uint8_T c5_k_emlrt_marshallIn(SFc5_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c5_b_tp_Standby, const char_T *c5_identifier)
{
  uint8_T c5_y;
  emlrtMsgIdentifier c5_thisId;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_y = c5_l_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_b_tp_Standby),
    &c5_thisId);
  sf_mex_destroy(&c5_b_tp_Standby);
  return c5_y;
}

static uint8_T c5_l_emlrt_marshallIn(SFc5_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId)
{
  uint8_T c5_y;
  uint8_T c5_u0;
  (void)chartInstance;
  sf_mex_import(c5_parentId, sf_mex_dup(c5_u), &c5_u0, 1, 3, 0U, 0, 0U, 0);
  c5_y = c5_u0;
  sf_mex_destroy(&c5_u);
  return c5_y;
}

static void c5_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData)
{
  const mxArray *c5_b_tp_Standby;
  const char_T *c5_identifier;
  emlrtMsgIdentifier c5_thisId;
  uint8_T c5_y;
  SFc5_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc5_SimulationModelInstanceStruct *)chartInstanceVoid;
  c5_b_tp_Standby = sf_mex_dup(c5_mxArrayInData);
  c5_identifier = c5_varName;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_y = c5_l_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_b_tp_Standby),
    &c5_thisId);
  sf_mex_destroy(&c5_b_tp_Standby);
  *(uint8_T *)c5_outData = c5_y;
  sf_mex_destroy(&c5_mxArrayInData);
}

static const mxArray *c5_m_emlrt_marshallIn(SFc5_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c5_b_setSimStateSideEffectsInfo, const char_T
  *c5_identifier)
{
  const mxArray *c5_y = NULL;
  emlrtMsgIdentifier c5_thisId;
  c5_y = NULL;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  sf_mex_assign(&c5_y, c5_n_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c5_b_setSimStateSideEffectsInfo), &c5_thisId), false);
  sf_mex_destroy(&c5_b_setSimStateSideEffectsInfo);
  return c5_y;
}

static const mxArray *c5_n_emlrt_marshallIn(SFc5_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId)
{
  const mxArray *c5_y = NULL;
  (void)chartInstance;
  (void)c5_parentId;
  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_duplicatearraysafe(&c5_u), false);
  sf_mex_destroy(&c5_u);
  return c5_y;
}

static void c5_updateDataWrittenToVector(SFc5_SimulationModelInstanceStruct
  *chartInstance, uint32_T c5_vectorIndex)
{
  (void)chartInstance;
  c5_dataWrittenToVector[(uint32_T)_SFD_EML_ARRAY_BOUNDS_CHECK(0, (int32_T)
    c5_vectorIndex, 0, 18, 1, 0)] = true;
}

static void c5_errorIfDataNotWrittenToFcn(SFc5_SimulationModelInstanceStruct
  *chartInstance, uint32_T c5_vectorIndex, uint32_T c5_dataNumber)
{
  (void)chartInstance;
  _SFD_DATA_READ_BEFORE_WRITE_CHECK(c5_dataNumber, c5_dataWrittenToVector
    [(uint32_T)_SFD_EML_ARRAY_BOUNDS_CHECK(0, (int32_T)c5_vectorIndex, 0, 18, 1,
    0)]);
}

static void init_dsm_address_info(SFc5_SimulationModelInstanceStruct
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

void sf_c5_SimulationModel_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3850779428U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3313183234U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(117362803U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3573318319U);
}

mxArray *sf_c5_SimulationModel_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("cOM7uErdwUZ6euP9O7hMVH");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,4,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(2);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
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
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(1));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(8);
      pr[1] = (double)(2);
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
      pr[1] = (double)(1);
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
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,1,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(2);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxData);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,3,3,dataFields);

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
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
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
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(8);
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
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,10,3,dataFields);

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
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
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
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
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
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
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
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
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
      pr[0] = (double)(1);
      pr[1] = (double)(1);
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
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,6,"type",mxType);
    }

    mxSetField(mxData,6,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(8);
      pr[1] = (double)(1);
      mxSetField(mxData,7,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(1));
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
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
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
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,9,"type",mxType);
    }

    mxSetField(mxData,9,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxData);
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c5_SimulationModel_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c5_SimulationModel_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

static const mxArray *sf_get_sim_state_info_c5_SimulationModel(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x10'type','srcId','name','auxInfo'{{M[1],M[80],T\"elapsedTime\",},{M[1],M[102],T\"targetsFound\",},{M[1],M[74],T\"timeLastTargetFound\",},{M[3],M[67],T\"allowedRunTime\",},{M[3],M[101],T\"currentTarget\",},{M[3],M[107],T\"local_t\",},{M[3],M[75],T\"nearLocation\",},{M[3],M[100],T\"numTargets\",},{M[3],M[79],T\"numTargetsFound\",},{M[3],M[105],T\"snapshotTime\",}}",
    "100 S1x6'type','srcId','name','auxInfo'{{M[3],M[69],T\"startingRadius\",},{M[3],M[68],T\"statTimeReq\",},{M[3],M[71],T\"targetBoundaryRadius\",},{M[8],M[0],T\"is_active_c5_SimulationModel\",},{M[9],M[0],T\"is_c5_SimulationModel\",},{M[9],M[3],T\"is_Running\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 16, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c5_SimulationModel_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc5_SimulationModelInstanceStruct *chartInstance;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    chartInstance = (SFc5_SimulationModelInstanceStruct *)
      chartInfo->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _SimulationModelMachineNumber_,
           5,
           10,
           11,
           0,
           33,
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
          _SFD_SET_DATA_PROPS(0,10,0,0,"startPos");
          _SFD_SET_DATA_PROPS(1,0,0,0,"allowedRunTime");
          _SFD_SET_DATA_PROPS(2,0,0,0,"statTimeReq");
          _SFD_SET_DATA_PROPS(3,0,0,0,"startingRadius");
          _SFD_SET_DATA_PROPS(4,1,1,0,"robotPosition");
          _SFD_SET_DATA_PROPS(5,0,0,0,"targetBoundaryRadius");
          _SFD_SET_DATA_PROPS(6,1,1,0,"StartFlag");
          _SFD_SET_DATA_PROPS(7,1,1,0,"targetPositions");
          _SFD_SET_DATA_PROPS(8,2,0,1,"timeLastTargetFound");
          _SFD_SET_DATA_PROPS(9,0,0,0,"nearLocation");
          _SFD_SET_DATA_PROPS(10,0,0,0,"numTargetsFound");
          _SFD_SET_DATA_PROPS(11,2,0,1,"elapsedTime");
          _SFD_SET_DATA_PROPS(12,0,0,0,"numTargets");
          _SFD_SET_DATA_PROPS(13,0,0,0,"currentTarget");
          _SFD_SET_DATA_PROPS(14,2,0,1,"targetsFound");
          _SFD_SET_DATA_PROPS(15,0,0,0,"snapshotTime");
          _SFD_SET_DATA_PROPS(16,0,0,0,"local_t");
          _SFD_SET_DATA_PROPS(17,1,1,0,"sim_time");
          _SFD_SET_DATA_PROPS(18,9,0,0,"");
          _SFD_SET_DATA_PROPS(19,8,0,0,"");
          _SFD_SET_DATA_PROPS(20,8,0,0,"");
          _SFD_SET_DATA_PROPS(21,9,0,0,"");
          _SFD_SET_DATA_PROPS(22,8,0,0,"");
          _SFD_SET_DATA_PROPS(23,9,0,0,"");
          _SFD_SET_DATA_PROPS(24,8,0,0,"");
          _SFD_SET_DATA_PROPS(25,8,0,0,"");
          _SFD_SET_DATA_PROPS(26,8,0,0,"");
          _SFD_SET_DATA_PROPS(27,8,0,0,"");
          _SFD_SET_DATA_PROPS(28,9,0,0,"");
          _SFD_SET_DATA_PROPS(29,9,0,0,"");
          _SFD_SET_DATA_PROPS(30,8,0,0,"");
          _SFD_SET_DATA_PROPS(31,8,0,0,"");
          _SFD_SET_DATA_PROPS(32,9,0,0,"");
          _SFD_STATE_INFO(0,0,0);
          _SFD_STATE_INFO(1,0,0);
          _SFD_STATE_INFO(2,0,0);
          _SFD_STATE_INFO(3,0,0);
          _SFD_STATE_INFO(4,0,0);
          _SFD_STATE_INFO(5,0,2);
          _SFD_STATE_INFO(6,0,2);
          _SFD_STATE_INFO(7,0,2);
          _SFD_STATE_INFO(8,0,2);
          _SFD_STATE_INFO(9,0,2);
          _SFD_CH_SUBSTATE_COUNT(3);
          _SFD_CH_SUBSTATE_DECOMP(0);
          _SFD_CH_SUBSTATE_INDEX(0,0);
          _SFD_CH_SUBSTATE_INDEX(1,1);
          _SFD_CH_SUBSTATE_INDEX(2,4);
          _SFD_ST_SUBSTATE_COUNT(0,0);
          _SFD_ST_SUBSTATE_COUNT(1,2);
          _SFD_ST_SUBSTATE_INDEX(1,0,2);
          _SFD_ST_SUBSTATE_INDEX(1,1,3);
          _SFD_ST_SUBSTATE_COUNT(2,0);
          _SFD_ST_SUBSTATE_COUNT(3,0);
          _SFD_ST_SUBSTATE_COUNT(4,0);
        }

        _SFD_CV_INIT_CHART(3,1,0,0);

        {
          _SFD_CV_INIT_STATE(0,0,0,0,0,0,NULL,NULL);
        }

        {
          _SFD_CV_INIT_STATE(1,2,1,1,0,0,NULL,NULL);
        }

        {
          _SFD_CV_INIT_STATE(2,0,0,0,0,0,NULL,NULL);
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

        {
          _SFD_CV_INIT_STATE(8,0,0,0,0,0,NULL,NULL);
        }

        {
          _SFD_CV_INIT_STATE(9,0,0,0,0,0,NULL,NULL);
        }

        _SFD_CV_INIT_TRANS(1,0,NULL,NULL,0,NULL);
        _SFD_CV_INIT_TRANS(4,0,NULL,NULL,0,NULL);
        _SFD_CV_INIT_TRANS(3,0,NULL,NULL,0,NULL);
        _SFD_CV_INIT_TRANS(2,0,NULL,NULL,0,NULL);
        _SFD_CV_INIT_TRANS(5,0,NULL,NULL,0,NULL);
        _SFD_CV_INIT_TRANS(7,0,NULL,NULL,0,NULL);
        _SFD_CV_INIT_TRANS(10,0,NULL,NULL,0,NULL);
        _SFD_CV_INIT_TRANS(6,0,NULL,NULL,0,NULL);
        _SFD_CV_INIT_TRANS(0,0,NULL,NULL,0,NULL);
        _SFD_CV_INIT_TRANS(9,0,NULL,NULL,0,NULL);
        _SFD_CV_INIT_TRANS(8,0,NULL,NULL,0,NULL);

        /* Initialization of MATLAB Function Model Coverage */
        _SFD_CV_INIT_EML(7,1,1,1,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_FCN(7,0,"startTiming",0,-1,142);
        _SFD_CV_INIT_EML_IF(7,1,0,46,75,109,141);
        _SFD_CV_INIT_EML(6,1,1,1,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_FCN(6,0,"getTiming",0,-1,183);
        _SFD_CV_INIT_EML_IF(6,1,0,58,87,136,173);
        _SFD_CV_INIT_EML(9,1,1,1,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_FCN(9,0,"updateTargetsFound",0,-1,256);
        _SFD_CV_INIT_EML_IF(9,1,0,98,126,187,220);
        _SFD_CV_INIT_EML(8,1,1,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_FCN(8,0,"testIfNearLocation",0,-1,230);
        _SFD_CV_INIT_EML(5,1,1,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_FCN(5,0,"distancefrom",0,-1,62);
        _SFD_CV_INIT_EML(4,1,0,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML(1,1,0,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML(2,1,0,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML(1,0,0,1,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_IF(1,0,0,1,21,1,21);
        _SFD_CV_INIT_EML(3,0,0,1,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_IF(3,0,0,1,22,1,22);
        _SFD_CV_INIT_EML(2,0,0,1,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_IF(2,0,0,1,43,1,43);
        _SFD_CV_INIT_EML(7,0,0,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML(10,0,0,1,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_IF(10,0,0,1,19,1,19);
        _SFD_CV_INIT_EML(0,0,0,1,0,0,0,0,0,2,1);
        _SFD_CV_INIT_EML_IF(0,0,0,1,79,1,48);

        {
          static int condStart[] = { 1, 25 };

          static int condEnd[] = { 18, 79 };

          static int pfixExpr[] = { 0, 1, -3 };

          _SFD_CV_INIT_EML_MCDC(0,0,0,1,79,2,0,&(condStart[0]),&(condEnd[0]),3,
                                &(pfixExpr[0]));
        }

        _SFD_CV_INIT_EML(9,0,0,1,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_IF(9,0,0,1,19,1,19);
        _SFD_CV_INIT_EML(8,0,0,1,0,0,0,0,0,2,1);
        _SFD_CV_INIT_EML_IF(8,0,0,1,77,1,48);

        {
          static int condStart[] = { 1, 38 };

          static int condEnd[] = { 30, 77 };

          static int pfixExpr[] = { 0, 1, -2 };

          _SFD_CV_INIT_EML_MCDC(8,0,0,1,77,2,0,&(condStart[0]),&(condEnd[0]),3,
                                &(pfixExpr[0]));
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 1;
          dimVector[1]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c5_e_sf_marshallOut,(MexInFcnForType)
            c5_e_sf_marshallIn);
        }

        _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c5_sf_marshallOut,(MexInFcnForType)c5_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c5_sf_marshallOut,(MexInFcnForType)c5_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c5_sf_marshallOut,(MexInFcnForType)c5_sf_marshallIn);

        {
          unsigned int dimVector[2];
          dimVector[0]= 1;
          dimVector[1]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c5_e_sf_marshallOut,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c5_sf_marshallOut,(MexInFcnForType)c5_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(6,SF_UINT8,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c5_c_sf_marshallOut,(MexInFcnForType)NULL);

        {
          unsigned int dimVector[2];
          dimVector[0]= 8;
          dimVector[1]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(7,SF_SINGLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c5_d_sf_marshallOut,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(8,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c5_sf_marshallOut,(MexInFcnForType)c5_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(9,SF_UINT8,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c5_c_sf_marshallOut,(MexInFcnForType)c5_c_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(10,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c5_sf_marshallOut,(MexInFcnForType)c5_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(11,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c5_sf_marshallOut,(MexInFcnForType)c5_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(12,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c5_sf_marshallOut,(MexInFcnForType)c5_sf_marshallIn);

        {
          unsigned int dimVector[1];
          dimVector[0]= 8;
          _SFD_SET_DATA_COMPILED_PROPS(13,SF_UINT8,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c5_b_sf_marshallOut,(MexInFcnForType)
            c5_b_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 8;
          _SFD_SET_DATA_COMPILED_PROPS(14,SF_UINT8,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c5_b_sf_marshallOut,(MexInFcnForType)
            c5_b_sf_marshallIn);
        }

        _SFD_SET_DATA_COMPILED_PROPS(15,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c5_sf_marshallOut,(MexInFcnForType)c5_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(16,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c5_sf_marshallOut,(MexInFcnForType)c5_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(17,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c5_sf_marshallOut,(MexInFcnForType)NULL);

        {
          unsigned int dimVector[1];
          dimVector[0]= 4294967295;
          _SFD_SET_DATA_COMPILED_PROPS(18,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)NULL,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 4294967295;
          _SFD_SET_DATA_COMPILED_PROPS(19,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)NULL,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 4294967295;
          _SFD_SET_DATA_COMPILED_PROPS(20,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)NULL,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 4294967295;
          _SFD_SET_DATA_COMPILED_PROPS(21,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)NULL,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 4294967295;
          _SFD_SET_DATA_COMPILED_PROPS(22,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)NULL,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 4294967295;
          _SFD_SET_DATA_COMPILED_PROPS(23,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)NULL,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 4294967295;
          _SFD_SET_DATA_COMPILED_PROPS(24,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)NULL,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 4294967295;
          _SFD_SET_DATA_COMPILED_PROPS(25,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)NULL,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 4294967295;
          _SFD_SET_DATA_COMPILED_PROPS(26,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)NULL,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 4294967295;
          _SFD_SET_DATA_COMPILED_PROPS(27,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)NULL,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 4294967295;
          _SFD_SET_DATA_COMPILED_PROPS(28,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)NULL,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 4294967295;
          _SFD_SET_DATA_COMPILED_PROPS(29,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)NULL,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 4294967295;
          _SFD_SET_DATA_COMPILED_PROPS(30,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)NULL,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 4294967295;
          _SFD_SET_DATA_COMPILED_PROPS(31,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)NULL,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 4294967295;
          _SFD_SET_DATA_COMPILED_PROPS(32,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)NULL,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_VALUE_PTR(18,(void *)(NULL));
        _SFD_SET_DATA_VALUE_PTR(19,(void *)(NULL));
        _SFD_SET_DATA_VALUE_PTR(20,(void *)(NULL));
        _SFD_SET_DATA_VALUE_PTR(21,(void *)(NULL));
        _SFD_SET_DATA_VALUE_PTR(22,(void *)(NULL));
        _SFD_SET_DATA_VALUE_PTR(23,(void *)(NULL));
        _SFD_SET_DATA_VALUE_PTR(24,(void *)(NULL));
        _SFD_SET_DATA_VALUE_PTR(25,(void *)(NULL));
        _SFD_SET_DATA_VALUE_PTR(26,(void *)(NULL));
        _SFD_SET_DATA_VALUE_PTR(27,(void *)(NULL));
        _SFD_SET_DATA_VALUE_PTR(28,(void *)(NULL));
        _SFD_SET_DATA_VALUE_PTR(29,(void *)(NULL));
        _SFD_SET_DATA_VALUE_PTR(30,(void *)(NULL));
        _SFD_SET_DATA_VALUE_PTR(31,(void *)(NULL));
        _SFD_SET_DATA_VALUE_PTR(32,(void *)(NULL));

        {
          boolean_T *c5_StartFlag;
          real_T *c5_timeLastTargetFound;
          real_T *c5_elapsedTime;
          real_T *c5_sim_time;
          real_T (*c5_robotPosition)[2];
          real32_T (*c5_targetPositions)[16];
          boolean_T (*c5_targetsFound)[8];
          c5_sim_time = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
          c5_targetsFound = (boolean_T (*)[8])ssGetOutputPortSignal
            (chartInstance->S, 3);
          c5_elapsedTime = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
          c5_timeLastTargetFound = (real_T *)ssGetOutputPortSignal
            (chartInstance->S, 1);
          c5_targetPositions = (real32_T (*)[16])ssGetInputPortSignal
            (chartInstance->S, 2);
          c5_StartFlag = (boolean_T *)ssGetInputPortSignal(chartInstance->S, 1);
          c5_robotPosition = (real_T (*)[2])ssGetInputPortSignal
            (chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, chartInstance->c5_startPos);
          _SFD_SET_DATA_VALUE_PTR(1U, &chartInstance->c5_allowedRunTime);
          _SFD_SET_DATA_VALUE_PTR(2U, &chartInstance->c5_statTimeReq);
          _SFD_SET_DATA_VALUE_PTR(3U, &chartInstance->c5_startingRadius);
          _SFD_SET_DATA_VALUE_PTR(4U, *c5_robotPosition);
          _SFD_SET_DATA_VALUE_PTR(5U, &chartInstance->c5_targetBoundaryRadius);
          _SFD_SET_DATA_VALUE_PTR(6U, c5_StartFlag);
          _SFD_SET_DATA_VALUE_PTR(7U, *c5_targetPositions);
          _SFD_SET_DATA_VALUE_PTR(8U, c5_timeLastTargetFound);
          _SFD_SET_DATA_VALUE_PTR(9U, &chartInstance->c5_nearLocation);
          _SFD_SET_DATA_VALUE_PTR(10U, &chartInstance->c5_numTargetsFound);
          _SFD_SET_DATA_VALUE_PTR(11U, c5_elapsedTime);
          _SFD_SET_DATA_VALUE_PTR(12U, &chartInstance->c5_numTargets);
          _SFD_SET_DATA_VALUE_PTR(13U, chartInstance->c5_currentTarget);
          _SFD_SET_DATA_VALUE_PTR(14U, *c5_targetsFound);
          _SFD_SET_DATA_VALUE_PTR(15U, &chartInstance->c5_snapshotTime);
          _SFD_SET_DATA_VALUE_PTR(16U, &chartInstance->local_t);
          _SFD_SET_DATA_VALUE_PTR(17U, c5_sim_time);
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
  return "eRakhMnlKT9xJviVBhRoNH";
}

static void sf_opaque_initialize_c5_SimulationModel(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc5_SimulationModelInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c5_SimulationModel((SFc5_SimulationModelInstanceStruct*)
    chartInstanceVar);
  initialize_c5_SimulationModel((SFc5_SimulationModelInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c5_SimulationModel(void *chartInstanceVar)
{
  enable_c5_SimulationModel((SFc5_SimulationModelInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c5_SimulationModel(void *chartInstanceVar)
{
  disable_c5_SimulationModel((SFc5_SimulationModelInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c5_SimulationModel(void *chartInstanceVar)
{
  sf_gateway_c5_SimulationModel((SFc5_SimulationModelInstanceStruct*)
    chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c5_SimulationModel(SimStruct* S)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c5_SimulationModel
    ((SFc5_SimulationModelInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c5_SimulationModel();/* state var info */
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

extern void sf_internal_set_sim_state_c5_SimulationModel(SimStruct* S, const
  mxArray *st)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[3];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxDuplicateArray(st);      /* high level simctx */
  prhs[2] = (mxArray*) sf_get_sim_state_info_c5_SimulationModel();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 3, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c5_SimulationModel((SFc5_SimulationModelInstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c5_SimulationModel(SimStruct* S)
{
  return sf_internal_get_sim_state_c5_SimulationModel(S);
}

static void sf_opaque_set_sim_state_c5_SimulationModel(SimStruct* S, const
  mxArray *st)
{
  sf_internal_set_sim_state_c5_SimulationModel(S, st);
}

static void sf_opaque_terminate_c5_SimulationModel(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc5_SimulationModelInstanceStruct*) chartInstanceVar)->S;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_SimulationModel_optimization_info();
    }

    finalize_c5_SimulationModel((SFc5_SimulationModelInstanceStruct*)
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
  initSimStructsc5_SimulationModel((SFc5_SimulationModelInstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c5_SimulationModel(SimStruct *S)
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
    initialize_params_c5_SimulationModel((SFc5_SimulationModelInstanceStruct*)
      (chartInfo->chartInstance));
  }
}

static void mdlSetWorkWidths_c5_SimulationModel(SimStruct *S)
{
  /* Actual parameters from chart:
     startPos
   */
  const char_T *rtParamNames[] = { "startPos" };

  ssSetNumRunTimeParams(S,ssGetSFcnParamsCount(S));

  /* registration for startPos*/
  ssRegDlgParamAsRunTimeParam(S, 0, 0, rtParamNames[0], SS_DOUBLE);
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_SimulationModel_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,5);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(sf_get_instance_specialization(),
                infoStruct,5,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,5,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(sf_get_instance_specialization(),infoStruct,5);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,5,4);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,5,3);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=3; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 4; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,5);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(213307585U));
  ssSetChecksum1(S,(86911738U));
  ssSetChecksum2(S,(373557120U));
  ssSetChecksum3(S,(2813753260U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c5_SimulationModel(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Stateflow");
  }
}

static void mdlStart_c5_SimulationModel(SimStruct *S)
{
  SFc5_SimulationModelInstanceStruct *chartInstance;
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)utMalloc(sizeof
    (ChartRunTimeInfo));
  chartInstance = (SFc5_SimulationModelInstanceStruct *)utMalloc(sizeof
    (SFc5_SimulationModelInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc5_SimulationModelInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 0;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c5_SimulationModel;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c5_SimulationModel;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c5_SimulationModel;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c5_SimulationModel;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c5_SimulationModel;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c5_SimulationModel;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c5_SimulationModel;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c5_SimulationModel;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c5_SimulationModel;
  chartInstance->chartInfo.mdlStart = mdlStart_c5_SimulationModel;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c5_SimulationModel;
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

void c5_SimulationModel_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c5_SimulationModel(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c5_SimulationModel(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c5_SimulationModel(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c5_SimulationModel_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
