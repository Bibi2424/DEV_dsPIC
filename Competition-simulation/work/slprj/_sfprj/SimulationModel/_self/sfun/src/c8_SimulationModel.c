/* Include files */

#include <stddef.h>
#include "blas.h"
#include "SimulationModel_sfun.h"
#include "c8_SimulationModel.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "SimulationModel_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(sfGlobalDebugInstanceStruct,S);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static real_T _sfTime_;
static const char * c8_debug_family_names[11] = { "loc", "locatedTargetsX",
  "locatedTargetsY", "targetTheta", "nargin", "nargout", "robotPos", "dist",
  "bearing", "robotTheta", "locatedTargets" };

/* Function Declarations */
static void initialize_c8_SimulationModel(SFc8_SimulationModelInstanceStruct
  *chartInstance);
static void initialize_params_c8_SimulationModel
  (SFc8_SimulationModelInstanceStruct *chartInstance);
static void enable_c8_SimulationModel(SFc8_SimulationModelInstanceStruct
  *chartInstance);
static void disable_c8_SimulationModel(SFc8_SimulationModelInstanceStruct
  *chartInstance);
static void c8_update_debugger_state_c8_SimulationModel
  (SFc8_SimulationModelInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c8_SimulationModel
  (SFc8_SimulationModelInstanceStruct *chartInstance);
static void set_sim_state_c8_SimulationModel(SFc8_SimulationModelInstanceStruct *
  chartInstance, const mxArray *c8_st);
static void finalize_c8_SimulationModel(SFc8_SimulationModelInstanceStruct
  *chartInstance);
static void sf_gateway_c8_SimulationModel(SFc8_SimulationModelInstanceStruct
  *chartInstance);
static void initSimStructsc8_SimulationModel(SFc8_SimulationModelInstanceStruct *
  chartInstance);
static void init_script_number_translation(uint32_T c8_machineNumber, uint32_T
  c8_chartNumber, uint32_T c8_instanceNumber);
static const mxArray *c8_sf_marshallOut(void *chartInstanceVoid, void *c8_inData);
static void c8_emlrt_marshallIn(SFc8_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c8_locatedTargets, const char_T *c8_identifier,
  real_T c8_y[12]);
static void c8_b_emlrt_marshallIn(SFc8_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId,
  real_T c8_y[12]);
static void c8_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData);
static const mxArray *c8_b_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData);
static const mxArray *c8_c_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData);
static const mxArray *c8_d_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData);
static real_T c8_c_emlrt_marshallIn(SFc8_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId);
static void c8_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData);
static const mxArray *c8_e_sf_marshallOut(void *chartInstanceVoid, real32_T
  c8_inData_data[], int32_T *c8_inData_sizes);
static void c8_d_emlrt_marshallIn(SFc8_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId,
  real32_T c8_y_data[], int32_T *c8_y_sizes);
static void c8_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, real32_T c8_outData_data[],
  int32_T *c8_outData_sizes);
static const mxArray *c8_f_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData);
static void c8_e_emlrt_marshallIn(SFc8_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId,
  real_T c8_y[6]);
static void c8_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData);
static const mxArray *c8_g_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData);
static void c8_f_emlrt_marshallIn(SFc8_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId,
  boolean_T c8_y[6]);
static void c8_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData);
static void c8_info_helper(const mxArray **c8_info);
static const mxArray *c8_emlrt_marshallOut(const char * c8_u);
static const mxArray *c8_b_emlrt_marshallOut(const uint32_T c8_u);
static void c8_eml_li_find(SFc8_SimulationModelInstanceStruct *chartInstance,
  boolean_T c8_x[6], int32_T c8_y_data[], int32_T *c8_y_sizes);
static void c8_cosd(SFc8_SimulationModelInstanceStruct *chartInstance, real32_T
                    c8_x_data[], int32_T c8_x_sizes, real32_T c8_b_x_data[],
                    int32_T *c8_b_x_sizes);
static boolean_T c8_isfinite(SFc8_SimulationModelInstanceStruct *chartInstance,
  real32_T c8_x);
static void c8_sind(SFc8_SimulationModelInstanceStruct *chartInstance, real32_T
                    c8_x_data[], int32_T c8_x_sizes, real32_T c8_b_x_data[],
                    int32_T *c8_b_x_sizes);
static const mxArray *c8_h_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData);
static int32_T c8_g_emlrt_marshallIn(SFc8_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId);
static void c8_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData);
static uint8_T c8_h_emlrt_marshallIn(SFc8_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c8_b_is_active_c8_SimulationModel, const char_T
  *c8_identifier);
static uint8_T c8_i_emlrt_marshallIn(SFc8_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId);
static void c8_b_cosd(SFc8_SimulationModelInstanceStruct *chartInstance,
                      real32_T c8_x_data[], int32_T *c8_x_sizes);
static void c8_b_sind(SFc8_SimulationModelInstanceStruct *chartInstance,
                      real32_T c8_x_data[], int32_T *c8_x_sizes);
static void init_dsm_address_info(SFc8_SimulationModelInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c8_SimulationModel(SFc8_SimulationModelInstanceStruct
  *chartInstance)
{
  chartInstance->c8_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c8_is_active_c8_SimulationModel = 0U;
}

static void initialize_params_c8_SimulationModel
  (SFc8_SimulationModelInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void enable_c8_SimulationModel(SFc8_SimulationModelInstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c8_SimulationModel(SFc8_SimulationModelInstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c8_update_debugger_state_c8_SimulationModel
  (SFc8_SimulationModelInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c8_SimulationModel
  (SFc8_SimulationModelInstanceStruct *chartInstance)
{
  const mxArray *c8_st;
  const mxArray *c8_y = NULL;
  int32_T c8_i0;
  real_T c8_u[12];
  const mxArray *c8_b_y = NULL;
  uint8_T c8_hoistedGlobal;
  uint8_T c8_b_u;
  const mxArray *c8_c_y = NULL;
  real_T (*c8_locatedTargets)[12];
  c8_locatedTargets = (real_T (*)[12])ssGetOutputPortSignal(chartInstance->S, 1);
  c8_st = NULL;
  c8_st = NULL;
  c8_y = NULL;
  sf_mex_assign(&c8_y, sf_mex_createcellmatrix(2, 1), false);
  for (c8_i0 = 0; c8_i0 < 12; c8_i0++) {
    c8_u[c8_i0] = (*c8_locatedTargets)[c8_i0];
  }

  c8_b_y = NULL;
  sf_mex_assign(&c8_b_y, sf_mex_create("y", c8_u, 0, 0U, 1U, 0U, 2, 6, 2), false);
  sf_mex_setcell(c8_y, 0, c8_b_y);
  c8_hoistedGlobal = chartInstance->c8_is_active_c8_SimulationModel;
  c8_b_u = c8_hoistedGlobal;
  c8_c_y = NULL;
  sf_mex_assign(&c8_c_y, sf_mex_create("y", &c8_b_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c8_y, 1, c8_c_y);
  sf_mex_assign(&c8_st, c8_y, false);
  return c8_st;
}

static void set_sim_state_c8_SimulationModel(SFc8_SimulationModelInstanceStruct *
  chartInstance, const mxArray *c8_st)
{
  const mxArray *c8_u;
  real_T c8_dv0[12];
  int32_T c8_i1;
  real_T (*c8_locatedTargets)[12];
  c8_locatedTargets = (real_T (*)[12])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c8_doneDoubleBufferReInit = true;
  c8_u = sf_mex_dup(c8_st);
  c8_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c8_u, 0)),
                      "locatedTargets", c8_dv0);
  for (c8_i1 = 0; c8_i1 < 12; c8_i1++) {
    (*c8_locatedTargets)[c8_i1] = c8_dv0[c8_i1];
  }

  chartInstance->c8_is_active_c8_SimulationModel = c8_h_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c8_u, 1)),
     "is_active_c8_SimulationModel");
  sf_mex_destroy(&c8_u);
  c8_update_debugger_state_c8_SimulationModel(chartInstance);
  sf_mex_destroy(&c8_st);
}

static void finalize_c8_SimulationModel(SFc8_SimulationModelInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c8_SimulationModel(SFc8_SimulationModelInstanceStruct
  *chartInstance)
{
  int32_T c8_i2;
  int32_T c8_i3;
  real_T c8_hoistedGlobal;
  int32_T c8_i4;
  real_T c8_robotPos[2];
  int32_T c8_i5;
  real32_T c8_dist[6];
  int32_T c8_i6;
  real32_T c8_bearing[6];
  real_T c8_robotTheta;
  uint32_T c8_debug_family_var_map[11];
  boolean_T c8_loc[6];
  real_T c8_locatedTargetsX[6];
  real_T c8_locatedTargetsY[6];
  int32_T c8_targetTheta_sizes;
  real32_T c8_targetTheta_data[6];
  real_T c8_nargin = 4.0;
  real_T c8_nargout = 1.0;
  real_T c8_locatedTargets[12];
  int32_T c8_i7;
  int32_T c8_i8;
  int32_T c8_i9;
  int32_T c8_i10;
  boolean_T c8_b_loc[6];
  int32_T c8_tmp_sizes;
  int32_T c8_tmp_data[6];
  int32_T c8_loop_ub;
  int32_T c8_i11;
  int32_T c8_i12;
  boolean_T c8_c_loc[6];
  int32_T c8_b_tmp_sizes;
  int32_T c8_b_loop_ub;
  int32_T c8_i13;
  real32_T c8_b_tmp_data[6];
  int32_T c8_c_tmp_sizes;
  int32_T c8_c_loop_ub;
  int32_T c8_i14;
  real32_T c8_c_tmp_data[6];
  int32_T c8_i15;
  boolean_T c8_d_loc[6];
  real32_T c8_b_robotPos;
  int32_T c8_d_tmp_sizes;
  int32_T c8_d_loop_ub;
  int32_T c8_i16;
  real_T c8_d_tmp_data[6];
  int32_T c8_e_loop_ub;
  int32_T c8_i17;
  int32_T c8_i18;
  boolean_T c8_e_loc[6];
  int32_T c8_f_loop_ub;
  int32_T c8_i19;
  int32_T c8_g_loop_ub;
  int32_T c8_i20;
  int32_T c8_i21;
  boolean_T c8_f_loc[6];
  real32_T c8_c_robotPos;
  int32_T c8_h_loop_ub;
  int32_T c8_i22;
  int32_T c8_i_loop_ub;
  int32_T c8_i23;
  int32_T c8_i24;
  int32_T c8_i25;
  int32_T c8_i26;
  int32_T c8_i27;
  int32_T c8_i28;
  real_T *c8_b_robotTheta;
  real_T (*c8_b_locatedTargets)[12];
  real32_T (*c8_b_bearing)[6];
  real32_T (*c8_b_dist)[6];
  real_T (*c8_d_robotPos)[2];
  c8_b_robotTheta = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
  c8_b_bearing = (real32_T (*)[6])ssGetInputPortSignal(chartInstance->S, 2);
  c8_b_locatedTargets = (real_T (*)[12])ssGetOutputPortSignal(chartInstance->S,
    1);
  c8_b_dist = (real32_T (*)[6])ssGetInputPortSignal(chartInstance->S, 1);
  c8_d_robotPos = (real_T (*)[2])ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 4U, chartInstance->c8_sfEvent);
  for (c8_i2 = 0; c8_i2 < 2; c8_i2++) {
    _SFD_DATA_RANGE_CHECK((*c8_d_robotPos)[c8_i2], 0U);
  }

  for (c8_i3 = 0; c8_i3 < 6; c8_i3++) {
    _SFD_DATA_RANGE_CHECK((real_T)(*c8_b_dist)[c8_i3], 1U);
  }

  chartInstance->c8_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 4U, chartInstance->c8_sfEvent);
  c8_hoistedGlobal = *c8_b_robotTheta;
  for (c8_i4 = 0; c8_i4 < 2; c8_i4++) {
    c8_robotPos[c8_i4] = (*c8_d_robotPos)[c8_i4];
  }

  for (c8_i5 = 0; c8_i5 < 6; c8_i5++) {
    c8_dist[c8_i5] = (*c8_b_dist)[c8_i5];
  }

  for (c8_i6 = 0; c8_i6 < 6; c8_i6++) {
    c8_bearing[c8_i6] = (*c8_b_bearing)[c8_i6];
  }

  c8_robotTheta = c8_hoistedGlobal;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 11U, 11U, c8_debug_family_names,
    c8_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c8_loc, 0U, c8_g_sf_marshallOut,
    c8_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c8_locatedTargetsX, 1U,
    c8_f_sf_marshallOut, c8_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c8_locatedTargetsY, 2U,
    c8_f_sf_marshallOut, c8_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_DYN_IMPORTABLE(c8_targetTheta_data, (const int32_T *)
    &c8_targetTheta_sizes, NULL, 0, 3, (void *)c8_e_sf_marshallOut, (void *)
    c8_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c8_nargin, 4U, c8_b_sf_marshallOut,
    c8_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c8_nargout, 5U, c8_b_sf_marshallOut,
    c8_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c8_robotPos, 6U, c8_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c8_dist, 7U, c8_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c8_bearing, 8U, c8_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c8_robotTheta, 9U, c8_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c8_locatedTargets, 10U, c8_sf_marshallOut,
    c8_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 4);
  for (c8_i7 = 0; c8_i7 < 6; c8_i7++) {
    c8_loc[c8_i7] = (c8_dist[c8_i7] > 0.0F);
  }

  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 6);
  for (c8_i8 = 0; c8_i8 < 6; c8_i8++) {
    c8_locatedTargetsX[c8_i8] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 7);
  for (c8_i9 = 0; c8_i9 < 6; c8_i9++) {
    c8_locatedTargetsY[c8_i9] = c8_locatedTargetsX[c8_i9];
  }

  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 9);
  for (c8_i10 = 0; c8_i10 < 6; c8_i10++) {
    c8_b_loc[c8_i10] = c8_loc[c8_i10];
  }

  c8_eml_li_find(chartInstance, c8_b_loc, c8_tmp_data, &c8_tmp_sizes);
  c8_targetTheta_sizes = c8_tmp_sizes;
  c8_loop_ub = c8_tmp_sizes - 1;
  for (c8_i11 = 0; c8_i11 <= c8_loop_ub; c8_i11++) {
    c8_targetTheta_data[c8_i11] = (real32_T)c8_robotTheta +
      c8_bearing[c8_tmp_data[c8_i11] - 1];
  }

  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 11);
  for (c8_i12 = 0; c8_i12 < 6; c8_i12++) {
    c8_c_loc[c8_i12] = c8_loc[c8_i12];
  }

  c8_eml_li_find(chartInstance, c8_c_loc, c8_tmp_data, &c8_tmp_sizes);
  c8_b_tmp_sizes = c8_tmp_sizes;
  c8_b_loop_ub = c8_tmp_sizes - 1;
  for (c8_i13 = 0; c8_i13 <= c8_b_loop_ub; c8_i13++) {
    c8_b_tmp_data[c8_i13] = c8_dist[c8_tmp_data[c8_i13] - 1];
  }

  c8_c_tmp_sizes = c8_targetTheta_sizes;
  c8_c_loop_ub = c8_targetTheta_sizes - 1;
  for (c8_i14 = 0; c8_i14 <= c8_c_loop_ub; c8_i14++) {
    c8_c_tmp_data[c8_i14] = c8_targetTheta_data[c8_i14];
  }

  c8_b_cosd(chartInstance, c8_c_tmp_data, &c8_c_tmp_sizes);
  _SFD_SIZE_EQ_CHECK_1D(c8_b_tmp_sizes, c8_c_tmp_sizes);
  for (c8_i15 = 0; c8_i15 < 6; c8_i15++) {
    c8_d_loc[c8_i15] = c8_loc[c8_i15];
  }

  c8_eml_li_find(chartInstance, c8_d_loc, c8_tmp_data, &c8_tmp_sizes);
  c8_b_robotPos = (real32_T)c8_robotPos[0];
  c8_d_tmp_sizes = c8_b_tmp_sizes;
  c8_d_loop_ub = c8_b_tmp_sizes - 1;
  for (c8_i16 = 0; c8_i16 <= c8_d_loop_ub; c8_i16++) {
    c8_d_tmp_data[c8_i16] = c8_b_robotPos + c8_b_tmp_data[c8_i16] *
      c8_c_tmp_data[c8_i16];
  }

  _SFD_SIZE_EQ_CHECK_1D(c8_tmp_sizes, c8_d_tmp_sizes);
  c8_e_loop_ub = c8_d_tmp_sizes - 1;
  for (c8_i17 = 0; c8_i17 <= c8_e_loop_ub; c8_i17++) {
    c8_locatedTargetsX[c8_tmp_data[c8_i17] - 1] = c8_d_tmp_data[c8_i17];
  }

  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 12);
  for (c8_i18 = 0; c8_i18 < 6; c8_i18++) {
    c8_e_loc[c8_i18] = c8_loc[c8_i18];
  }

  c8_eml_li_find(chartInstance, c8_e_loc, c8_tmp_data, &c8_tmp_sizes);
  c8_b_tmp_sizes = c8_tmp_sizes;
  c8_f_loop_ub = c8_tmp_sizes - 1;
  for (c8_i19 = 0; c8_i19 <= c8_f_loop_ub; c8_i19++) {
    c8_b_tmp_data[c8_i19] = c8_dist[c8_tmp_data[c8_i19] - 1];
  }

  c8_c_tmp_sizes = c8_targetTheta_sizes;
  c8_g_loop_ub = c8_targetTheta_sizes - 1;
  for (c8_i20 = 0; c8_i20 <= c8_g_loop_ub; c8_i20++) {
    c8_c_tmp_data[c8_i20] = c8_targetTheta_data[c8_i20];
  }

  c8_b_sind(chartInstance, c8_c_tmp_data, &c8_c_tmp_sizes);
  _SFD_SIZE_EQ_CHECK_1D(c8_b_tmp_sizes, c8_c_tmp_sizes);
  for (c8_i21 = 0; c8_i21 < 6; c8_i21++) {
    c8_f_loc[c8_i21] = c8_loc[c8_i21];
  }

  c8_eml_li_find(chartInstance, c8_f_loc, c8_tmp_data, &c8_tmp_sizes);
  c8_c_robotPos = (real32_T)c8_robotPos[1];
  c8_d_tmp_sizes = c8_b_tmp_sizes;
  c8_h_loop_ub = c8_b_tmp_sizes - 1;
  for (c8_i22 = 0; c8_i22 <= c8_h_loop_ub; c8_i22++) {
    c8_d_tmp_data[c8_i22] = c8_c_robotPos + c8_b_tmp_data[c8_i22] *
      c8_c_tmp_data[c8_i22];
  }

  _SFD_SIZE_EQ_CHECK_1D(c8_tmp_sizes, c8_d_tmp_sizes);
  c8_i_loop_ub = c8_d_tmp_sizes - 1;
  for (c8_i23 = 0; c8_i23 <= c8_i_loop_ub; c8_i23++) {
    c8_locatedTargetsY[c8_tmp_data[c8_i23] - 1] = c8_d_tmp_data[c8_i23];
  }

  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 14);
  for (c8_i24 = 0; c8_i24 < 6; c8_i24++) {
    c8_locatedTargets[c8_i24] = c8_locatedTargetsX[c8_i24];
  }

  for (c8_i25 = 0; c8_i25 < 6; c8_i25++) {
    c8_locatedTargets[c8_i25 + 6] = c8_locatedTargetsY[c8_i25];
  }

  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, -14);
  _SFD_SYMBOL_SCOPE_POP();
  for (c8_i26 = 0; c8_i26 < 12; c8_i26++) {
    (*c8_b_locatedTargets)[c8_i26] = c8_locatedTargets[c8_i26];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 4U, chartInstance->c8_sfEvent);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_SimulationModelMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
  for (c8_i27 = 0; c8_i27 < 12; c8_i27++) {
    _SFD_DATA_RANGE_CHECK((*c8_b_locatedTargets)[c8_i27], 2U);
  }

  for (c8_i28 = 0; c8_i28 < 6; c8_i28++) {
    _SFD_DATA_RANGE_CHECK((real_T)(*c8_b_bearing)[c8_i28], 3U);
  }

  _SFD_DATA_RANGE_CHECK(*c8_b_robotTheta, 4U);
}

static void initSimStructsc8_SimulationModel(SFc8_SimulationModelInstanceStruct *
  chartInstance)
{
  (void)chartInstance;
}

static void init_script_number_translation(uint32_T c8_machineNumber, uint32_T
  c8_chartNumber, uint32_T c8_instanceNumber)
{
  (void)c8_machineNumber;
  (void)c8_chartNumber;
  (void)c8_instanceNumber;
}

static const mxArray *c8_sf_marshallOut(void *chartInstanceVoid, void *c8_inData)
{
  const mxArray *c8_mxArrayOutData = NULL;
  int32_T c8_i29;
  int32_T c8_i30;
  int32_T c8_i31;
  real_T c8_b_inData[12];
  int32_T c8_i32;
  int32_T c8_i33;
  int32_T c8_i34;
  real_T c8_u[12];
  const mxArray *c8_y = NULL;
  SFc8_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc8_SimulationModelInstanceStruct *)chartInstanceVoid;
  c8_mxArrayOutData = NULL;
  c8_i29 = 0;
  for (c8_i30 = 0; c8_i30 < 2; c8_i30++) {
    for (c8_i31 = 0; c8_i31 < 6; c8_i31++) {
      c8_b_inData[c8_i31 + c8_i29] = (*(real_T (*)[12])c8_inData)[c8_i31 +
        c8_i29];
    }

    c8_i29 += 6;
  }

  c8_i32 = 0;
  for (c8_i33 = 0; c8_i33 < 2; c8_i33++) {
    for (c8_i34 = 0; c8_i34 < 6; c8_i34++) {
      c8_u[c8_i34 + c8_i32] = c8_b_inData[c8_i34 + c8_i32];
    }

    c8_i32 += 6;
  }

  c8_y = NULL;
  sf_mex_assign(&c8_y, sf_mex_create("y", c8_u, 0, 0U, 1U, 0U, 2, 6, 2), false);
  sf_mex_assign(&c8_mxArrayOutData, c8_y, false);
  return c8_mxArrayOutData;
}

static void c8_emlrt_marshallIn(SFc8_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c8_locatedTargets, const char_T *c8_identifier,
  real_T c8_y[12])
{
  emlrtMsgIdentifier c8_thisId;
  c8_thisId.fIdentifier = c8_identifier;
  c8_thisId.fParent = NULL;
  c8_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c8_locatedTargets), &c8_thisId,
                        c8_y);
  sf_mex_destroy(&c8_locatedTargets);
}

static void c8_b_emlrt_marshallIn(SFc8_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId,
  real_T c8_y[12])
{
  real_T c8_dv1[12];
  int32_T c8_i35;
  (void)chartInstance;
  sf_mex_import(c8_parentId, sf_mex_dup(c8_u), c8_dv1, 1, 0, 0U, 1, 0U, 2, 6, 2);
  for (c8_i35 = 0; c8_i35 < 12; c8_i35++) {
    c8_y[c8_i35] = c8_dv1[c8_i35];
  }

  sf_mex_destroy(&c8_u);
}

static void c8_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData)
{
  const mxArray *c8_locatedTargets;
  const char_T *c8_identifier;
  emlrtMsgIdentifier c8_thisId;
  real_T c8_y[12];
  int32_T c8_i36;
  int32_T c8_i37;
  int32_T c8_i38;
  SFc8_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc8_SimulationModelInstanceStruct *)chartInstanceVoid;
  c8_locatedTargets = sf_mex_dup(c8_mxArrayInData);
  c8_identifier = c8_varName;
  c8_thisId.fIdentifier = c8_identifier;
  c8_thisId.fParent = NULL;
  c8_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c8_locatedTargets), &c8_thisId,
                        c8_y);
  sf_mex_destroy(&c8_locatedTargets);
  c8_i36 = 0;
  for (c8_i37 = 0; c8_i37 < 2; c8_i37++) {
    for (c8_i38 = 0; c8_i38 < 6; c8_i38++) {
      (*(real_T (*)[12])c8_outData)[c8_i38 + c8_i36] = c8_y[c8_i38 + c8_i36];
    }

    c8_i36 += 6;
  }

  sf_mex_destroy(&c8_mxArrayInData);
}

static const mxArray *c8_b_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData)
{
  const mxArray *c8_mxArrayOutData = NULL;
  real_T c8_u;
  const mxArray *c8_y = NULL;
  SFc8_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc8_SimulationModelInstanceStruct *)chartInstanceVoid;
  c8_mxArrayOutData = NULL;
  c8_u = *(real_T *)c8_inData;
  c8_y = NULL;
  sf_mex_assign(&c8_y, sf_mex_create("y", &c8_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c8_mxArrayOutData, c8_y, false);
  return c8_mxArrayOutData;
}

static const mxArray *c8_c_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData)
{
  const mxArray *c8_mxArrayOutData = NULL;
  int32_T c8_i39;
  real32_T c8_b_inData[6];
  int32_T c8_i40;
  real32_T c8_u[6];
  const mxArray *c8_y = NULL;
  SFc8_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc8_SimulationModelInstanceStruct *)chartInstanceVoid;
  c8_mxArrayOutData = NULL;
  for (c8_i39 = 0; c8_i39 < 6; c8_i39++) {
    c8_b_inData[c8_i39] = (*(real32_T (*)[6])c8_inData)[c8_i39];
  }

  for (c8_i40 = 0; c8_i40 < 6; c8_i40++) {
    c8_u[c8_i40] = c8_b_inData[c8_i40];
  }

  c8_y = NULL;
  sf_mex_assign(&c8_y, sf_mex_create("y", c8_u, 1, 0U, 1U, 0U, 1, 6), false);
  sf_mex_assign(&c8_mxArrayOutData, c8_y, false);
  return c8_mxArrayOutData;
}

static const mxArray *c8_d_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData)
{
  const mxArray *c8_mxArrayOutData = NULL;
  int32_T c8_i41;
  real_T c8_b_inData[2];
  int32_T c8_i42;
  real_T c8_u[2];
  const mxArray *c8_y = NULL;
  SFc8_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc8_SimulationModelInstanceStruct *)chartInstanceVoid;
  c8_mxArrayOutData = NULL;
  for (c8_i41 = 0; c8_i41 < 2; c8_i41++) {
    c8_b_inData[c8_i41] = (*(real_T (*)[2])c8_inData)[c8_i41];
  }

  for (c8_i42 = 0; c8_i42 < 2; c8_i42++) {
    c8_u[c8_i42] = c8_b_inData[c8_i42];
  }

  c8_y = NULL;
  sf_mex_assign(&c8_y, sf_mex_create("y", c8_u, 0, 0U, 1U, 0U, 1, 2), false);
  sf_mex_assign(&c8_mxArrayOutData, c8_y, false);
  return c8_mxArrayOutData;
}

static real_T c8_c_emlrt_marshallIn(SFc8_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId)
{
  real_T c8_y;
  real_T c8_d0;
  (void)chartInstance;
  sf_mex_import(c8_parentId, sf_mex_dup(c8_u), &c8_d0, 1, 0, 0U, 0, 0U, 0);
  c8_y = c8_d0;
  sf_mex_destroy(&c8_u);
  return c8_y;
}

static void c8_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData)
{
  const mxArray *c8_nargout;
  const char_T *c8_identifier;
  emlrtMsgIdentifier c8_thisId;
  real_T c8_y;
  SFc8_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc8_SimulationModelInstanceStruct *)chartInstanceVoid;
  c8_nargout = sf_mex_dup(c8_mxArrayInData);
  c8_identifier = c8_varName;
  c8_thisId.fIdentifier = c8_identifier;
  c8_thisId.fParent = NULL;
  c8_y = c8_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c8_nargout), &c8_thisId);
  sf_mex_destroy(&c8_nargout);
  *(real_T *)c8_outData = c8_y;
  sf_mex_destroy(&c8_mxArrayInData);
}

static const mxArray *c8_e_sf_marshallOut(void *chartInstanceVoid, real32_T
  c8_inData_data[], int32_T *c8_inData_sizes)
{
  const mxArray *c8_mxArrayOutData = NULL;
  int32_T c8_b_inData_sizes;
  int32_T c8_loop_ub;
  int32_T c8_i43;
  real32_T c8_b_inData_data[6];
  int32_T c8_u_sizes;
  int32_T c8_b_loop_ub;
  int32_T c8_i44;
  real32_T c8_u_data[6];
  const mxArray *c8_y = NULL;
  SFc8_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc8_SimulationModelInstanceStruct *)chartInstanceVoid;
  c8_mxArrayOutData = NULL;
  c8_b_inData_sizes = *c8_inData_sizes;
  c8_loop_ub = *c8_inData_sizes - 1;
  for (c8_i43 = 0; c8_i43 <= c8_loop_ub; c8_i43++) {
    c8_b_inData_data[c8_i43] = c8_inData_data[c8_i43];
  }

  c8_u_sizes = c8_b_inData_sizes;
  c8_b_loop_ub = c8_b_inData_sizes - 1;
  for (c8_i44 = 0; c8_i44 <= c8_b_loop_ub; c8_i44++) {
    c8_u_data[c8_i44] = c8_b_inData_data[c8_i44];
  }

  c8_y = NULL;
  sf_mex_assign(&c8_y, sf_mex_create("y", c8_u_data, 1, 0U, 1U, 0U, 1,
    c8_u_sizes), false);
  sf_mex_assign(&c8_mxArrayOutData, c8_y, false);
  return c8_mxArrayOutData;
}

static void c8_d_emlrt_marshallIn(SFc8_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId,
  real32_T c8_y_data[], int32_T *c8_y_sizes)
{
  static uint32_T c8_uv0[1] = { 6U };

  uint32_T c8_uv1[1];
  static boolean_T c8_bv0[1] = { true };

  boolean_T c8_bv1[1];
  int32_T c8_tmp_sizes;
  real32_T c8_tmp_data[6];
  int32_T c8_loop_ub;
  int32_T c8_i45;
  (void)chartInstance;
  c8_uv1[0] = c8_uv0[0];
  c8_bv1[0] = c8_bv0[0];
  sf_mex_import_vs(c8_parentId, sf_mex_dup(c8_u), c8_tmp_data, 1, 1, 0U, 1, 0U,
                   1, c8_bv1, c8_uv1, &c8_tmp_sizes);
  *c8_y_sizes = c8_tmp_sizes;
  c8_loop_ub = c8_tmp_sizes - 1;
  for (c8_i45 = 0; c8_i45 <= c8_loop_ub; c8_i45++) {
    c8_y_data[c8_i45] = c8_tmp_data[c8_i45];
  }

  sf_mex_destroy(&c8_u);
}

static void c8_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, real32_T c8_outData_data[],
  int32_T *c8_outData_sizes)
{
  const mxArray *c8_targetTheta;
  const char_T *c8_identifier;
  emlrtMsgIdentifier c8_thisId;
  int32_T c8_y_sizes;
  real32_T c8_y_data[6];
  int32_T c8_loop_ub;
  int32_T c8_i46;
  SFc8_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc8_SimulationModelInstanceStruct *)chartInstanceVoid;
  c8_targetTheta = sf_mex_dup(c8_mxArrayInData);
  c8_identifier = c8_varName;
  c8_thisId.fIdentifier = c8_identifier;
  c8_thisId.fParent = NULL;
  c8_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c8_targetTheta), &c8_thisId,
                        c8_y_data, &c8_y_sizes);
  sf_mex_destroy(&c8_targetTheta);
  *c8_outData_sizes = c8_y_sizes;
  c8_loop_ub = c8_y_sizes - 1;
  for (c8_i46 = 0; c8_i46 <= c8_loop_ub; c8_i46++) {
    c8_outData_data[c8_i46] = c8_y_data[c8_i46];
  }

  sf_mex_destroy(&c8_mxArrayInData);
}

static const mxArray *c8_f_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData)
{
  const mxArray *c8_mxArrayOutData = NULL;
  int32_T c8_i47;
  real_T c8_b_inData[6];
  int32_T c8_i48;
  real_T c8_u[6];
  const mxArray *c8_y = NULL;
  SFc8_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc8_SimulationModelInstanceStruct *)chartInstanceVoid;
  c8_mxArrayOutData = NULL;
  for (c8_i47 = 0; c8_i47 < 6; c8_i47++) {
    c8_b_inData[c8_i47] = (*(real_T (*)[6])c8_inData)[c8_i47];
  }

  for (c8_i48 = 0; c8_i48 < 6; c8_i48++) {
    c8_u[c8_i48] = c8_b_inData[c8_i48];
  }

  c8_y = NULL;
  sf_mex_assign(&c8_y, sf_mex_create("y", c8_u, 0, 0U, 1U, 0U, 1, 6), false);
  sf_mex_assign(&c8_mxArrayOutData, c8_y, false);
  return c8_mxArrayOutData;
}

static void c8_e_emlrt_marshallIn(SFc8_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId,
  real_T c8_y[6])
{
  real_T c8_dv2[6];
  int32_T c8_i49;
  (void)chartInstance;
  sf_mex_import(c8_parentId, sf_mex_dup(c8_u), c8_dv2, 1, 0, 0U, 1, 0U, 1, 6);
  for (c8_i49 = 0; c8_i49 < 6; c8_i49++) {
    c8_y[c8_i49] = c8_dv2[c8_i49];
  }

  sf_mex_destroy(&c8_u);
}

static void c8_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData)
{
  const mxArray *c8_locatedTargetsY;
  const char_T *c8_identifier;
  emlrtMsgIdentifier c8_thisId;
  real_T c8_y[6];
  int32_T c8_i50;
  SFc8_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc8_SimulationModelInstanceStruct *)chartInstanceVoid;
  c8_locatedTargetsY = sf_mex_dup(c8_mxArrayInData);
  c8_identifier = c8_varName;
  c8_thisId.fIdentifier = c8_identifier;
  c8_thisId.fParent = NULL;
  c8_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c8_locatedTargetsY),
                        &c8_thisId, c8_y);
  sf_mex_destroy(&c8_locatedTargetsY);
  for (c8_i50 = 0; c8_i50 < 6; c8_i50++) {
    (*(real_T (*)[6])c8_outData)[c8_i50] = c8_y[c8_i50];
  }

  sf_mex_destroy(&c8_mxArrayInData);
}

static const mxArray *c8_g_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData)
{
  const mxArray *c8_mxArrayOutData = NULL;
  int32_T c8_i51;
  boolean_T c8_b_inData[6];
  int32_T c8_i52;
  boolean_T c8_u[6];
  const mxArray *c8_y = NULL;
  SFc8_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc8_SimulationModelInstanceStruct *)chartInstanceVoid;
  c8_mxArrayOutData = NULL;
  for (c8_i51 = 0; c8_i51 < 6; c8_i51++) {
    c8_b_inData[c8_i51] = (*(boolean_T (*)[6])c8_inData)[c8_i51];
  }

  for (c8_i52 = 0; c8_i52 < 6; c8_i52++) {
    c8_u[c8_i52] = c8_b_inData[c8_i52];
  }

  c8_y = NULL;
  sf_mex_assign(&c8_y, sf_mex_create("y", c8_u, 11, 0U, 1U, 0U, 1, 6), false);
  sf_mex_assign(&c8_mxArrayOutData, c8_y, false);
  return c8_mxArrayOutData;
}

static void c8_f_emlrt_marshallIn(SFc8_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId,
  boolean_T c8_y[6])
{
  boolean_T c8_bv2[6];
  int32_T c8_i53;
  (void)chartInstance;
  sf_mex_import(c8_parentId, sf_mex_dup(c8_u), c8_bv2, 1, 11, 0U, 1, 0U, 1, 6);
  for (c8_i53 = 0; c8_i53 < 6; c8_i53++) {
    c8_y[c8_i53] = c8_bv2[c8_i53];
  }

  sf_mex_destroy(&c8_u);
}

static void c8_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData)
{
  const mxArray *c8_loc;
  const char_T *c8_identifier;
  emlrtMsgIdentifier c8_thisId;
  boolean_T c8_y[6];
  int32_T c8_i54;
  SFc8_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc8_SimulationModelInstanceStruct *)chartInstanceVoid;
  c8_loc = sf_mex_dup(c8_mxArrayInData);
  c8_identifier = c8_varName;
  c8_thisId.fIdentifier = c8_identifier;
  c8_thisId.fParent = NULL;
  c8_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c8_loc), &c8_thisId, c8_y);
  sf_mex_destroy(&c8_loc);
  for (c8_i54 = 0; c8_i54 < 6; c8_i54++) {
    (*(boolean_T (*)[6])c8_outData)[c8_i54] = c8_y[c8_i54];
  }

  sf_mex_destroy(&c8_mxArrayInData);
}

const mxArray *sf_c8_SimulationModel_get_eml_resolved_functions_info(void)
{
  const mxArray *c8_nameCaptureInfo = NULL;
  c8_nameCaptureInfo = NULL;
  sf_mex_assign(&c8_nameCaptureInfo, sf_mex_createstruct("structure", 2, 42, 1),
                false);
  c8_info_helper(&c8_nameCaptureInfo);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c8_nameCaptureInfo);
  return c8_nameCaptureInfo;
}

static void c8_info_helper(const mxArray **c8_info)
{
  const mxArray *c8_rhs0 = NULL;
  const mxArray *c8_lhs0 = NULL;
  const mxArray *c8_rhs1 = NULL;
  const mxArray *c8_lhs1 = NULL;
  const mxArray *c8_rhs2 = NULL;
  const mxArray *c8_lhs2 = NULL;
  const mxArray *c8_rhs3 = NULL;
  const mxArray *c8_lhs3 = NULL;
  const mxArray *c8_rhs4 = NULL;
  const mxArray *c8_lhs4 = NULL;
  const mxArray *c8_rhs5 = NULL;
  const mxArray *c8_lhs5 = NULL;
  const mxArray *c8_rhs6 = NULL;
  const mxArray *c8_lhs6 = NULL;
  const mxArray *c8_rhs7 = NULL;
  const mxArray *c8_lhs7 = NULL;
  const mxArray *c8_rhs8 = NULL;
  const mxArray *c8_lhs8 = NULL;
  const mxArray *c8_rhs9 = NULL;
  const mxArray *c8_lhs9 = NULL;
  const mxArray *c8_rhs10 = NULL;
  const mxArray *c8_lhs10 = NULL;
  const mxArray *c8_rhs11 = NULL;
  const mxArray *c8_lhs11 = NULL;
  const mxArray *c8_rhs12 = NULL;
  const mxArray *c8_lhs12 = NULL;
  const mxArray *c8_rhs13 = NULL;
  const mxArray *c8_lhs13 = NULL;
  const mxArray *c8_rhs14 = NULL;
  const mxArray *c8_lhs14 = NULL;
  const mxArray *c8_rhs15 = NULL;
  const mxArray *c8_lhs15 = NULL;
  const mxArray *c8_rhs16 = NULL;
  const mxArray *c8_lhs16 = NULL;
  const mxArray *c8_rhs17 = NULL;
  const mxArray *c8_lhs17 = NULL;
  const mxArray *c8_rhs18 = NULL;
  const mxArray *c8_lhs18 = NULL;
  const mxArray *c8_rhs19 = NULL;
  const mxArray *c8_lhs19 = NULL;
  const mxArray *c8_rhs20 = NULL;
  const mxArray *c8_lhs20 = NULL;
  const mxArray *c8_rhs21 = NULL;
  const mxArray *c8_lhs21 = NULL;
  const mxArray *c8_rhs22 = NULL;
  const mxArray *c8_lhs22 = NULL;
  const mxArray *c8_rhs23 = NULL;
  const mxArray *c8_lhs23 = NULL;
  const mxArray *c8_rhs24 = NULL;
  const mxArray *c8_lhs24 = NULL;
  const mxArray *c8_rhs25 = NULL;
  const mxArray *c8_lhs25 = NULL;
  const mxArray *c8_rhs26 = NULL;
  const mxArray *c8_lhs26 = NULL;
  const mxArray *c8_rhs27 = NULL;
  const mxArray *c8_lhs27 = NULL;
  const mxArray *c8_rhs28 = NULL;
  const mxArray *c8_lhs28 = NULL;
  const mxArray *c8_rhs29 = NULL;
  const mxArray *c8_lhs29 = NULL;
  const mxArray *c8_rhs30 = NULL;
  const mxArray *c8_lhs30 = NULL;
  const mxArray *c8_rhs31 = NULL;
  const mxArray *c8_lhs31 = NULL;
  const mxArray *c8_rhs32 = NULL;
  const mxArray *c8_lhs32 = NULL;
  const mxArray *c8_rhs33 = NULL;
  const mxArray *c8_lhs33 = NULL;
  const mxArray *c8_rhs34 = NULL;
  const mxArray *c8_lhs34 = NULL;
  const mxArray *c8_rhs35 = NULL;
  const mxArray *c8_lhs35 = NULL;
  const mxArray *c8_rhs36 = NULL;
  const mxArray *c8_lhs36 = NULL;
  const mxArray *c8_rhs37 = NULL;
  const mxArray *c8_lhs37 = NULL;
  const mxArray *c8_rhs38 = NULL;
  const mxArray *c8_lhs38 = NULL;
  const mxArray *c8_rhs39 = NULL;
  const mxArray *c8_lhs39 = NULL;
  const mxArray *c8_rhs40 = NULL;
  const mxArray *c8_lhs40 = NULL;
  const mxArray *c8_rhs41 = NULL;
  const mxArray *c8_lhs41 = NULL;
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(""), "context", "context", 0);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("eml_li_find"), "name", "name",
                  0);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 0);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_li_find.m"), "resolved",
                  "resolved", 0);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1286818786U), "fileTimeLo",
                  "fileTimeLo", 0);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 0);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 0);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 0);
  sf_mex_assign(&c8_rhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs0), "rhs", "rhs", 0);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs0), "lhs", "lhs", 0);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_li_find.m"), "context",
                  "context", 1);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 1);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 1);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 1);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1323170578U), "fileTimeLo",
                  "fileTimeLo", 1);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 1);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 1);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 1);
  sf_mex_assign(&c8_rhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs1), "rhs", "rhs", 1);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs1), "lhs", "lhs", 1);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_li_find.m!compute_nones"),
                  "context", "context", 2);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 2);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 2);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 2);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1323170578U), "fileTimeLo",
                  "fileTimeLo", 2);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 2);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 2);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 2);
  sf_mex_assign(&c8_rhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs2), "rhs", "rhs", 2);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs2), "lhs", "lhs", 2);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_li_find.m!compute_nones"),
                  "context", "context", 3);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 3);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 3);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 3);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 3);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 3);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 3);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 3);
  sf_mex_assign(&c8_rhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs3), "rhs", "rhs", 3);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs3), "lhs", "lhs", 3);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper"),
                  "context", "context", 4);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("intmax"), "name", "name", 4);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 4);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "resolved",
                  "resolved", 4);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1362261882U), "fileTimeLo",
                  "fileTimeLo", 4);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 4);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 4);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 4);
  sf_mex_assign(&c8_rhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs4), "rhs", "rhs", 4);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs4), "lhs", "lhs", 4);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "context",
                  "context", 5);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 5);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 5);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 5);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1381850300U), "fileTimeLo",
                  "fileTimeLo", 5);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 5);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 5);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 5);
  sf_mex_assign(&c8_rhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs5), "rhs", "rhs", 5);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs5), "lhs", "lhs", 5);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_li_find.m!compute_nones"),
                  "context", "context", 6);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 6);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 6);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 6);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1372582416U), "fileTimeLo",
                  "fileTimeLo", 6);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 6);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 6);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 6);
  sf_mex_assign(&c8_rhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs6), "rhs", "rhs", 6);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs6), "lhs", "lhs", 6);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"), "context",
                  "context", 7);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 7);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 7);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 7);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 7);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 7);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 7);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 7);
  sf_mex_assign(&c8_rhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs7), "rhs", "rhs", 7);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs7), "lhs", "lhs", 7);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_li_find.m"), "context",
                  "context", 8);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 8);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 8);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 8);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 8);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 8);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 8);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 8);
  sf_mex_assign(&c8_rhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs8), "rhs", "rhs", 8);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs8), "lhs", "lhs", 8);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_li_find.m"), "context",
                  "context", 9);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 9);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 9);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 9);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1372582416U), "fileTimeLo",
                  "fileTimeLo", 9);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 9);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 9);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 9);
  sf_mex_assign(&c8_rhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs9), "rhs", "rhs", 9);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs9), "lhs", "lhs", 9);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(""), "context", "context", 10);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("cosd"), "name", "name", 10);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 10);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cosd.m"), "resolved",
                  "resolved", 10);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1343830372U), "fileTimeLo",
                  "fileTimeLo", 10);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 10);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 10);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 10);
  sf_mex_assign(&c8_rhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs10), "rhs", "rhs",
                  10);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs10), "lhs", "lhs",
                  10);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cosd.m"), "context",
                  "context", 11);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("eml_scalar_cosd_and_sind"),
                  "name", "name", 11);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 11);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_cosd_and_sind.m"),
                  "resolved", "resolved", 11);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1343830378U), "fileTimeLo",
                  "fileTimeLo", 11);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 11);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 11);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 11);
  sf_mex_assign(&c8_rhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs11), "rhs", "rhs",
                  11);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs11), "lhs", "lhs",
                  11);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_cosd_and_sind.m"),
                  "context", "context", 12);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("isfinite"), "name", "name", 12);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 12);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m"), "resolved",
                  "resolved", 12);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1363713856U), "fileTimeLo",
                  "fileTimeLo", 12);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 12);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 12);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 12);
  sf_mex_assign(&c8_rhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs12), "rhs", "rhs",
                  12);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs12), "lhs", "lhs",
                  12);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m"), "context",
                  "context", 13);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 13);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 13);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 13);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 13);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 13);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 13);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 13);
  sf_mex_assign(&c8_rhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs13), "rhs", "rhs",
                  13);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs13), "lhs", "lhs",
                  13);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m"), "context",
                  "context", 14);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("isinf"), "name", "name", 14);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 14);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isinf.m"), "resolved",
                  "resolved", 14);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1363713856U), "fileTimeLo",
                  "fileTimeLo", 14);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 14);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 14);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 14);
  sf_mex_assign(&c8_rhs14, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs14, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs14), "rhs", "rhs",
                  14);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs14), "lhs", "lhs",
                  14);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isinf.m"), "context",
                  "context", 15);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 15);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 15);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 15);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 15);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 15);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 15);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 15);
  sf_mex_assign(&c8_rhs15, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs15, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs15), "rhs", "rhs",
                  15);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs15), "lhs", "lhs",
                  15);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m"), "context",
                  "context", 16);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("isnan"), "name", "name", 16);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 16);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m"), "resolved",
                  "resolved", 16);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1363713858U), "fileTimeLo",
                  "fileTimeLo", 16);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 16);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 16);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 16);
  sf_mex_assign(&c8_rhs16, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs16, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs16), "rhs", "rhs",
                  16);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs16), "lhs", "lhs",
                  16);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m"), "context",
                  "context", 17);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 17);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 17);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 17);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 17);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 17);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 17);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 17);
  sf_mex_assign(&c8_rhs17, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs17, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs17), "rhs", "rhs",
                  17);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs17), "lhs", "lhs",
                  17);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_cosd_and_sind.m"),
                  "context", "context", 18);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("eml_guarded_nan"), "name",
                  "name", 18);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 18);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_guarded_nan.m"),
                  "resolved", "resolved", 18);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1286818776U), "fileTimeLo",
                  "fileTimeLo", 18);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 18);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 18);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 18);
  sf_mex_assign(&c8_rhs18, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs18, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs18), "rhs", "rhs",
                  18);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs18), "lhs", "lhs",
                  18);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_guarded_nan.m"),
                  "context", "context", 19);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("eml_is_float_class"), "name",
                  "name", 19);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 19);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_float_class.m"),
                  "resolved", "resolved", 19);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1286818782U), "fileTimeLo",
                  "fileTimeLo", 19);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 19);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 19);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 19);
  sf_mex_assign(&c8_rhs19, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs19, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs19), "rhs", "rhs",
                  19);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs19), "lhs", "lhs",
                  19);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_cosd_and_sind.m"),
                  "context", "context", 20);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("eml_scalar_rem90"), "name",
                  "name", 20);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 20);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_rem90.m"),
                  "resolved", "resolved", 20);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1343830378U), "fileTimeLo",
                  "fileTimeLo", 20);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 20);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 20);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 20);
  sf_mex_assign(&c8_rhs20, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs20, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs20), "rhs", "rhs",
                  20);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs20), "lhs", "lhs",
                  20);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_rem90.m"),
                  "context", "context", 21);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("mrdivide"), "name", "name", 21);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 21);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "resolved",
                  "resolved", 21);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1388460096U), "fileTimeLo",
                  "fileTimeLo", 21);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 21);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1370009886U), "mFileTimeLo",
                  "mFileTimeLo", 21);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 21);
  sf_mex_assign(&c8_rhs21, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs21, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs21), "rhs", "rhs",
                  21);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs21), "lhs", "lhs",
                  21);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 22);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("coder.internal.assert"),
                  "name", "name", 22);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 22);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assert.m"),
                  "resolved", "resolved", 22);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 22);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 22);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 22);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 22);
  sf_mex_assign(&c8_rhs22, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs22, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs22), "rhs", "rhs",
                  22);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs22), "lhs", "lhs",
                  22);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 23);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("rdivide"), "name", "name", 23);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 23);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "resolved",
                  "resolved", 23);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1363713880U), "fileTimeLo",
                  "fileTimeLo", 23);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 23);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 23);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 23);
  sf_mex_assign(&c8_rhs23, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs23, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs23), "rhs", "rhs",
                  23);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs23), "lhs", "lhs",
                  23);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 24);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 24);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 24);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 24);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 24);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 24);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 24);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 24);
  sf_mex_assign(&c8_rhs24, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs24, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs24), "rhs", "rhs",
                  24);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs24), "lhs", "lhs",
                  24);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 25);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("eml_scalexp_compatible"),
                  "name", "name", 25);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 25);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m"),
                  "resolved", "resolved", 25);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1286818796U), "fileTimeLo",
                  "fileTimeLo", 25);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 25);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 25);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 25);
  sf_mex_assign(&c8_rhs25, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs25, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs25), "rhs", "rhs",
                  25);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs25), "lhs", "lhs",
                  25);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 26);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("eml_div"), "name", "name", 26);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 26);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "resolved",
                  "resolved", 26);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 26);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 26);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 26);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 26);
  sf_mex_assign(&c8_rhs26, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs26, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs26), "rhs", "rhs",
                  26);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs26), "lhs", "lhs",
                  26);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "context",
                  "context", 27);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("coder.internal.div"), "name",
                  "name", 27);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 27);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/div.p"), "resolved",
                  "resolved", 27);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1389307920U), "fileTimeLo",
                  "fileTimeLo", 27);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 27);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 27);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 27);
  sf_mex_assign(&c8_rhs27, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs27, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs27), "rhs", "rhs",
                  27);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs27), "lhs", "lhs",
                  27);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_rem90.m"),
                  "context", "context", 28);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("rem"), "name", "name", 28);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 28);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/rem.m"), "resolved",
                  "resolved", 28);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1363713854U), "fileTimeLo",
                  "fileTimeLo", 28);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 28);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 28);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 28);
  sf_mex_assign(&c8_rhs28, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs28, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs28), "rhs", "rhs",
                  28);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs28), "lhs", "lhs",
                  28);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/rem.m"), "context",
                  "context", 29);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 29);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 29);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 29);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 29);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 29);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 29);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 29);
  sf_mex_assign(&c8_rhs29, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs29, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs29), "rhs", "rhs",
                  29);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs29), "lhs", "lhs",
                  29);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/rem.m"), "context",
                  "context", 30);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 30);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 30);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 30);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 30);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 30);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 30);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 30);
  sf_mex_assign(&c8_rhs30, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs30, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs30), "rhs", "rhs",
                  30);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs30), "lhs", "lhs",
                  30);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "context",
                  "context", 31);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 31);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 31);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 31);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1389307920U), "fileTimeLo",
                  "fileTimeLo", 31);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 31);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 31);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 31);
  sf_mex_assign(&c8_rhs31, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs31, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs31), "rhs", "rhs",
                  31);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs31), "lhs", "lhs",
                  31);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/rem.m"), "context",
                  "context", 32);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("eml_scalexp_alloc"), "name",
                  "name", 32);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 32);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "resolved", "resolved", 32);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 32);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 32);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 32);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 32);
  sf_mex_assign(&c8_rhs32, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs32, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs32), "rhs", "rhs",
                  32);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs32), "lhs", "lhs",
                  32);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "context", "context", 33);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("coder.internal.scalexpAlloc"),
                  "name", "name", 33);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 33);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalexpAlloc.p"),
                  "resolved", "resolved", 33);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1389307920U), "fileTimeLo",
                  "fileTimeLo", 33);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 33);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 33);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 33);
  sf_mex_assign(&c8_rhs33, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs33, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs33), "rhs", "rhs",
                  33);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs33), "lhs", "lhs",
                  33);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_rem90.m"),
                  "context", "context", 34);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("abs"), "name", "name", 34);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 34);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "resolved",
                  "resolved", 34);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1363713852U), "fileTimeLo",
                  "fileTimeLo", 34);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 34);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 34);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 34);
  sf_mex_assign(&c8_rhs34, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs34, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs34), "rhs", "rhs",
                  34);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs34), "lhs", "lhs",
                  34);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 35);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 35);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 35);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 35);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 35);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 35);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 35);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 35);
  sf_mex_assign(&c8_rhs35, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs35, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs35), "rhs", "rhs",
                  35);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs35), "lhs", "lhs",
                  35);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 36);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("eml_scalar_abs"), "name",
                  "name", 36);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 36);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m"),
                  "resolved", "resolved", 36);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1286818712U), "fileTimeLo",
                  "fileTimeLo", 36);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 36);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 36);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 36);
  sf_mex_assign(&c8_rhs36, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs36, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs36), "rhs", "rhs",
                  36);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs36), "lhs", "lhs",
                  36);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_rem90.m"),
                  "context", "context", 37);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("eml_mtimes_helper"), "name",
                  "name", 37);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 37);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "resolved", "resolved", 37);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1383877294U), "fileTimeLo",
                  "fileTimeLo", 37);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 37);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 37);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 37);
  sf_mex_assign(&c8_rhs37, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs37, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs37), "rhs", "rhs",
                  37);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs37), "lhs", "lhs",
                  37);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m!common_checks"),
                  "context", "context", 38);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 38);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 38);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 38);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 38);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 38);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 38);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 38);
  sf_mex_assign(&c8_rhs38, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs38, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs38), "rhs", "rhs",
                  38);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs38), "lhs", "lhs",
                  38);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m!common_checks"),
                  "context", "context", 39);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 39);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 39);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 39);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 39);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 39);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 39);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 39);
  sf_mex_assign(&c8_rhs39, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs39, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs39), "rhs", "rhs",
                  39);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs39), "lhs", "lhs",
                  39);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(""), "context", "context", 40);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("sind"), "name", "name", 40);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 40);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sind.m"), "resolved",
                  "resolved", 40);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1343830386U), "fileTimeLo",
                  "fileTimeLo", 40);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 40);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 40);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 40);
  sf_mex_assign(&c8_rhs40, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs40, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs40), "rhs", "rhs",
                  40);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs40), "lhs", "lhs",
                  40);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sind.m"), "context",
                  "context", 41);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("eml_scalar_cosd_and_sind"),
                  "name", "name", 41);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 41);
  sf_mex_addfield(*c8_info, c8_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_cosd_and_sind.m"),
                  "resolved", "resolved", 41);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(1343830378U), "fileTimeLo",
                  "fileTimeLo", 41);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 41);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 41);
  sf_mex_addfield(*c8_info, c8_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 41);
  sf_mex_assign(&c8_rhs41, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c8_lhs41, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_rhs41), "rhs", "rhs",
                  41);
  sf_mex_addfield(*c8_info, sf_mex_duplicatearraysafe(&c8_lhs41), "lhs", "lhs",
                  41);
  sf_mex_destroy(&c8_rhs0);
  sf_mex_destroy(&c8_lhs0);
  sf_mex_destroy(&c8_rhs1);
  sf_mex_destroy(&c8_lhs1);
  sf_mex_destroy(&c8_rhs2);
  sf_mex_destroy(&c8_lhs2);
  sf_mex_destroy(&c8_rhs3);
  sf_mex_destroy(&c8_lhs3);
  sf_mex_destroy(&c8_rhs4);
  sf_mex_destroy(&c8_lhs4);
  sf_mex_destroy(&c8_rhs5);
  sf_mex_destroy(&c8_lhs5);
  sf_mex_destroy(&c8_rhs6);
  sf_mex_destroy(&c8_lhs6);
  sf_mex_destroy(&c8_rhs7);
  sf_mex_destroy(&c8_lhs7);
  sf_mex_destroy(&c8_rhs8);
  sf_mex_destroy(&c8_lhs8);
  sf_mex_destroy(&c8_rhs9);
  sf_mex_destroy(&c8_lhs9);
  sf_mex_destroy(&c8_rhs10);
  sf_mex_destroy(&c8_lhs10);
  sf_mex_destroy(&c8_rhs11);
  sf_mex_destroy(&c8_lhs11);
  sf_mex_destroy(&c8_rhs12);
  sf_mex_destroy(&c8_lhs12);
  sf_mex_destroy(&c8_rhs13);
  sf_mex_destroy(&c8_lhs13);
  sf_mex_destroy(&c8_rhs14);
  sf_mex_destroy(&c8_lhs14);
  sf_mex_destroy(&c8_rhs15);
  sf_mex_destroy(&c8_lhs15);
  sf_mex_destroy(&c8_rhs16);
  sf_mex_destroy(&c8_lhs16);
  sf_mex_destroy(&c8_rhs17);
  sf_mex_destroy(&c8_lhs17);
  sf_mex_destroy(&c8_rhs18);
  sf_mex_destroy(&c8_lhs18);
  sf_mex_destroy(&c8_rhs19);
  sf_mex_destroy(&c8_lhs19);
  sf_mex_destroy(&c8_rhs20);
  sf_mex_destroy(&c8_lhs20);
  sf_mex_destroy(&c8_rhs21);
  sf_mex_destroy(&c8_lhs21);
  sf_mex_destroy(&c8_rhs22);
  sf_mex_destroy(&c8_lhs22);
  sf_mex_destroy(&c8_rhs23);
  sf_mex_destroy(&c8_lhs23);
  sf_mex_destroy(&c8_rhs24);
  sf_mex_destroy(&c8_lhs24);
  sf_mex_destroy(&c8_rhs25);
  sf_mex_destroy(&c8_lhs25);
  sf_mex_destroy(&c8_rhs26);
  sf_mex_destroy(&c8_lhs26);
  sf_mex_destroy(&c8_rhs27);
  sf_mex_destroy(&c8_lhs27);
  sf_mex_destroy(&c8_rhs28);
  sf_mex_destroy(&c8_lhs28);
  sf_mex_destroy(&c8_rhs29);
  sf_mex_destroy(&c8_lhs29);
  sf_mex_destroy(&c8_rhs30);
  sf_mex_destroy(&c8_lhs30);
  sf_mex_destroy(&c8_rhs31);
  sf_mex_destroy(&c8_lhs31);
  sf_mex_destroy(&c8_rhs32);
  sf_mex_destroy(&c8_lhs32);
  sf_mex_destroy(&c8_rhs33);
  sf_mex_destroy(&c8_lhs33);
  sf_mex_destroy(&c8_rhs34);
  sf_mex_destroy(&c8_lhs34);
  sf_mex_destroy(&c8_rhs35);
  sf_mex_destroy(&c8_lhs35);
  sf_mex_destroy(&c8_rhs36);
  sf_mex_destroy(&c8_lhs36);
  sf_mex_destroy(&c8_rhs37);
  sf_mex_destroy(&c8_lhs37);
  sf_mex_destroy(&c8_rhs38);
  sf_mex_destroy(&c8_lhs38);
  sf_mex_destroy(&c8_rhs39);
  sf_mex_destroy(&c8_lhs39);
  sf_mex_destroy(&c8_rhs40);
  sf_mex_destroy(&c8_lhs40);
  sf_mex_destroy(&c8_rhs41);
  sf_mex_destroy(&c8_lhs41);
}

static const mxArray *c8_emlrt_marshallOut(const char * c8_u)
{
  const mxArray *c8_y = NULL;
  c8_y = NULL;
  sf_mex_assign(&c8_y, sf_mex_create("y", c8_u, 15, 0U, 0U, 0U, 2, 1, strlen
    (c8_u)), false);
  return c8_y;
}

static const mxArray *c8_b_emlrt_marshallOut(const uint32_T c8_u)
{
  const mxArray *c8_y = NULL;
  c8_y = NULL;
  sf_mex_assign(&c8_y, sf_mex_create("y", &c8_u, 7, 0U, 0U, 0U, 0), false);
  return c8_y;
}

static void c8_eml_li_find(SFc8_SimulationModelInstanceStruct *chartInstance,
  boolean_T c8_x[6], int32_T c8_y_data[], int32_T *c8_y_sizes)
{
  int32_T c8_k;
  int32_T c8_i;
  int32_T c8_b_i;
  int32_T c8_a;
  int32_T c8_b_a;
  const mxArray *c8_y = NULL;
  int32_T c8_tmp_sizes;
  int32_T c8_loop_ub;
  int32_T c8_i55;
  int32_T c8_tmp_data[6];
  int32_T c8_j;
  int32_T c8_c_i;
  int32_T c8_d_i;
  int32_T c8_c_a;
  int32_T c8_d_a;
  (void)chartInstance;
  c8_k = 0;
  for (c8_i = 1; c8_i < 7; c8_i++) {
    c8_b_i = c8_i - 1;
    if (c8_x[c8_b_i]) {
      c8_a = c8_k;
      c8_b_a = c8_a + 1;
      c8_k = c8_b_a;
    }
  }

  if (c8_k <= 6) {
  } else {
    c8_y = NULL;
    sf_mex_assign(&c8_y, sf_mex_create("y", "Assertion failed.", 15, 0U, 0U, 0U,
      2, 1, strlen("Assertion failed.")), false);
    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14, c8_y);
  }

  c8_tmp_sizes = (int32_T)_SFD_NON_NEGATIVE_CHECK("", (real_T)c8_k);
  c8_loop_ub = (int32_T)_SFD_NON_NEGATIVE_CHECK("", (real_T)c8_k) - 1;
  for (c8_i55 = 0; c8_i55 <= c8_loop_ub; c8_i55++) {
    c8_tmp_data[c8_i55] = 0;
  }

  *c8_y_sizes = c8_tmp_sizes;
  c8_j = 1;
  for (c8_c_i = 1; c8_c_i < 7; c8_c_i++) {
    c8_d_i = c8_c_i;
    if (c8_x[c8_d_i - 1]) {
      c8_y_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c8_j, 1, *c8_y_sizes, 1, 0) - 1]
        = c8_d_i;
      c8_c_a = c8_j;
      c8_d_a = c8_c_a + 1;
      c8_j = c8_d_a;
    }
  }
}

static void c8_cosd(SFc8_SimulationModelInstanceStruct *chartInstance, real32_T
                    c8_x_data[], int32_T c8_x_sizes, real32_T c8_b_x_data[],
                    int32_T *c8_b_x_sizes)
{
  int32_T c8_loop_ub;
  int32_T c8_i56;
  *c8_b_x_sizes = c8_x_sizes;
  c8_loop_ub = c8_x_sizes - 1;
  for (c8_i56 = 0; c8_i56 <= c8_loop_ub; c8_i56++) {
    c8_b_x_data[c8_i56] = c8_x_data[c8_i56];
  }

  c8_b_cosd(chartInstance, c8_b_x_data, c8_b_x_sizes);
}

static boolean_T c8_isfinite(SFc8_SimulationModelInstanceStruct *chartInstance,
  real32_T c8_x)
{
  real32_T c8_b_x;
  boolean_T c8_b_b;
  boolean_T c8_b0;
  real32_T c8_c_x;
  boolean_T c8_c_b;
  boolean_T c8_b1;
  (void)chartInstance;
  c8_b_x = c8_x;
  c8_b_b = muSingleScalarIsInf(c8_b_x);
  c8_b0 = !c8_b_b;
  c8_c_x = c8_x;
  c8_c_b = muSingleScalarIsNaN(c8_c_x);
  c8_b1 = !c8_c_b;
  return c8_b0 && c8_b1;
}

static void c8_sind(SFc8_SimulationModelInstanceStruct *chartInstance, real32_T
                    c8_x_data[], int32_T c8_x_sizes, real32_T c8_b_x_data[],
                    int32_T *c8_b_x_sizes)
{
  int32_T c8_loop_ub;
  int32_T c8_i57;
  *c8_b_x_sizes = c8_x_sizes;
  c8_loop_ub = c8_x_sizes - 1;
  for (c8_i57 = 0; c8_i57 <= c8_loop_ub; c8_i57++) {
    c8_b_x_data[c8_i57] = c8_x_data[c8_i57];
  }

  c8_b_sind(chartInstance, c8_b_x_data, c8_b_x_sizes);
}

static const mxArray *c8_h_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData)
{
  const mxArray *c8_mxArrayOutData = NULL;
  int32_T c8_u;
  const mxArray *c8_y = NULL;
  SFc8_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc8_SimulationModelInstanceStruct *)chartInstanceVoid;
  c8_mxArrayOutData = NULL;
  c8_u = *(int32_T *)c8_inData;
  c8_y = NULL;
  sf_mex_assign(&c8_y, sf_mex_create("y", &c8_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c8_mxArrayOutData, c8_y, false);
  return c8_mxArrayOutData;
}

static int32_T c8_g_emlrt_marshallIn(SFc8_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId)
{
  int32_T c8_y;
  int32_T c8_i58;
  (void)chartInstance;
  sf_mex_import(c8_parentId, sf_mex_dup(c8_u), &c8_i58, 1, 6, 0U, 0, 0U, 0);
  c8_y = c8_i58;
  sf_mex_destroy(&c8_u);
  return c8_y;
}

static void c8_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData)
{
  const mxArray *c8_b_sfEvent;
  const char_T *c8_identifier;
  emlrtMsgIdentifier c8_thisId;
  int32_T c8_y;
  SFc8_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc8_SimulationModelInstanceStruct *)chartInstanceVoid;
  c8_b_sfEvent = sf_mex_dup(c8_mxArrayInData);
  c8_identifier = c8_varName;
  c8_thisId.fIdentifier = c8_identifier;
  c8_thisId.fParent = NULL;
  c8_y = c8_g_emlrt_marshallIn(chartInstance, sf_mex_dup(c8_b_sfEvent),
    &c8_thisId);
  sf_mex_destroy(&c8_b_sfEvent);
  *(int32_T *)c8_outData = c8_y;
  sf_mex_destroy(&c8_mxArrayInData);
}

static uint8_T c8_h_emlrt_marshallIn(SFc8_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c8_b_is_active_c8_SimulationModel, const char_T
  *c8_identifier)
{
  uint8_T c8_y;
  emlrtMsgIdentifier c8_thisId;
  c8_thisId.fIdentifier = c8_identifier;
  c8_thisId.fParent = NULL;
  c8_y = c8_i_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c8_b_is_active_c8_SimulationModel), &c8_thisId);
  sf_mex_destroy(&c8_b_is_active_c8_SimulationModel);
  return c8_y;
}

static uint8_T c8_i_emlrt_marshallIn(SFc8_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId)
{
  uint8_T c8_y;
  uint8_T c8_u0;
  (void)chartInstance;
  sf_mex_import(c8_parentId, sf_mex_dup(c8_u), &c8_u0, 1, 3, 0U, 0, 0U, 0);
  c8_y = c8_u0;
  sf_mex_destroy(&c8_u);
  return c8_y;
}

static void c8_b_cosd(SFc8_SimulationModelInstanceStruct *chartInstance,
                      real32_T c8_x_data[], int32_T *c8_x_sizes)
{
  real_T c8_d1;
  int32_T c8_i59;
  int32_T c8_k;
  real_T c8_b_k;
  real32_T c8_x;
  real32_T c8_c;
  real32_T c8_b_x;
  real32_T c8_c_x;
  real32_T c8_d_x;
  real32_T c8_xk;
  real32_T c8_e_x;
  real32_T c8_f_x;
  real32_T c8_absx;
  real32_T c8_g_x;
  real32_T c8_h_x;
  real32_T c8_b;
  int8_T c8_n;
  real32_T c8_b_b;
  real32_T c8_c_b;
  real32_T c8_d_b;
  real32_T c8_e_b;
  int8_T c8_b_n;
  c8_d1 = (real_T)*c8_x_sizes;
  c8_i59 = (int32_T)c8_d1 - 1;
  for (c8_k = 0; c8_k <= c8_i59; c8_k++) {
    c8_b_k = 1.0 + (real_T)c8_k;
    c8_x = c8_x_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)c8_b_k, 1,
      *c8_x_sizes, 1, 0) - 1];
    if (!c8_isfinite(chartInstance, c8_x)) {
      c8_c = ((real32_T)rtNaN);
    } else {
      c8_b_x = c8_x;
      c8_c_x = c8_b_x;
      c8_d_x = c8_c_x;
      c8_xk = c8_d_x;
      c8_c_x = muSingleScalarRem(c8_xk, 360.0F);
      c8_e_x = c8_c_x;
      c8_f_x = c8_e_x;
      c8_absx = muSingleScalarAbs(c8_f_x);
      if (c8_absx > 180.0F) {
        if (c8_c_x > 0.0F) {
          c8_c_x -= 360.0F;
        } else {
          c8_c_x += 360.0F;
        }

        c8_g_x = c8_c_x;
        c8_h_x = c8_g_x;
        c8_absx = muSingleScalarAbs(c8_h_x);
      }

      if (c8_absx <= 45.0F) {
        c8_b = c8_c_x;
        c8_c_x = 0.0174532924F * c8_b;
        c8_n = 0;
      } else if (c8_absx <= 135.0F) {
        if (c8_c_x > 0.0F) {
          c8_b_b = c8_c_x - 90.0F;
          c8_c_x = 0.0174532924F * c8_b_b;
          c8_n = 1;
        } else {
          c8_c_b = c8_c_x + 90.0F;
          c8_c_x = 0.0174532924F * c8_c_b;
          c8_n = -1;
        }
      } else if (c8_c_x > 0.0F) {
        c8_d_b = c8_c_x - 180.0F;
        c8_c_x = 0.0174532924F * c8_d_b;
        c8_n = 2;
      } else {
        c8_e_b = c8_c_x + 180.0F;
        c8_c_x = 0.0174532924F * c8_e_b;
        c8_n = -2;
      }

      c8_b_n = c8_n;
      c8_x = c8_c_x;
      if ((real_T)c8_b_n == 0.0) {
        c8_c = muSingleScalarCos(c8_x);
      } else if ((real_T)c8_b_n == 1.0) {
        c8_c = -muSingleScalarSin(c8_x);
      } else if ((real_T)c8_b_n == -1.0) {
        c8_c = muSingleScalarSin(c8_x);
      } else {
        c8_c = -muSingleScalarCos(c8_x);
      }
    }

    c8_x_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)c8_b_k, 1, *c8_x_sizes, 1,
      0) - 1] = c8_c;
  }
}

static void c8_b_sind(SFc8_SimulationModelInstanceStruct *chartInstance,
                      real32_T c8_x_data[], int32_T *c8_x_sizes)
{
  real_T c8_d2;
  int32_T c8_i60;
  int32_T c8_k;
  real_T c8_b_k;
  real32_T c8_x;
  real32_T c8_f0;
  real32_T c8_b_x;
  real32_T c8_c_x;
  real32_T c8_d_x;
  real32_T c8_xk;
  real32_T c8_e_x;
  real32_T c8_f_x;
  real32_T c8_absx;
  real32_T c8_g_x;
  real32_T c8_h_x;
  real32_T c8_b;
  int8_T c8_n;
  real32_T c8_b_b;
  real32_T c8_c_b;
  real32_T c8_d_b;
  real32_T c8_e_b;
  int8_T c8_b_n;
  c8_d2 = (real_T)*c8_x_sizes;
  c8_i60 = (int32_T)c8_d2 - 1;
  for (c8_k = 0; c8_k <= c8_i60; c8_k++) {
    c8_b_k = 1.0 + (real_T)c8_k;
    c8_x = c8_x_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)c8_b_k, 1,
      *c8_x_sizes, 1, 0) - 1];
    if (!c8_isfinite(chartInstance, c8_x)) {
      c8_f0 = ((real32_T)rtNaN);
    } else {
      c8_b_x = c8_x;
      c8_c_x = c8_b_x;
      c8_d_x = c8_c_x;
      c8_xk = c8_d_x;
      c8_c_x = muSingleScalarRem(c8_xk, 360.0F);
      c8_e_x = c8_c_x;
      c8_f_x = c8_e_x;
      c8_absx = muSingleScalarAbs(c8_f_x);
      if (c8_absx > 180.0F) {
        if (c8_c_x > 0.0F) {
          c8_c_x -= 360.0F;
        } else {
          c8_c_x += 360.0F;
        }

        c8_g_x = c8_c_x;
        c8_h_x = c8_g_x;
        c8_absx = muSingleScalarAbs(c8_h_x);
      }

      if (c8_absx <= 45.0F) {
        c8_b = c8_c_x;
        c8_c_x = 0.0174532924F * c8_b;
        c8_n = 0;
      } else if (c8_absx <= 135.0F) {
        if (c8_c_x > 0.0F) {
          c8_b_b = c8_c_x - 90.0F;
          c8_c_x = 0.0174532924F * c8_b_b;
          c8_n = 1;
        } else {
          c8_c_b = c8_c_x + 90.0F;
          c8_c_x = 0.0174532924F * c8_c_b;
          c8_n = -1;
        }
      } else if (c8_c_x > 0.0F) {
        c8_d_b = c8_c_x - 180.0F;
        c8_c_x = 0.0174532924F * c8_d_b;
        c8_n = 2;
      } else {
        c8_e_b = c8_c_x + 180.0F;
        c8_c_x = 0.0174532924F * c8_e_b;
        c8_n = -2;
      }

      c8_b_n = c8_n;
      c8_x = c8_c_x;
      if ((real_T)c8_b_n == 0.0) {
        c8_f0 = muSingleScalarSin(c8_x);
      } else if ((real_T)c8_b_n == 1.0) {
        c8_f0 = muSingleScalarCos(c8_x);
      } else if ((real_T)c8_b_n == -1.0) {
        c8_f0 = -muSingleScalarCos(c8_x);
      } else {
        c8_f0 = -muSingleScalarSin(c8_x);
      }
    }

    c8_x_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)c8_b_k, 1, *c8_x_sizes, 1,
      0) - 1] = c8_f0;
  }
}

static void init_dsm_address_info(SFc8_SimulationModelInstanceStruct
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

void sf_c8_SimulationModel_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3955878422U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3506663024U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1021019922U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3422747712U);
}

mxArray *sf_c8_SimulationModel_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("iLYCs8bm7r2EOS18m9NOeE");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,4,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(2);
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
      pr[0] = (double)(6);
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
      pr[0] = (double)(6);
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
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,1,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(6);
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
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c8_SimulationModel_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c8_SimulationModel_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

static const mxArray *sf_get_sim_state_info_c8_SimulationModel(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x2'type','srcId','name','auxInfo'{{M[1],M[5],T\"locatedTargets\",},{M[8],M[0],T\"is_active_c8_SimulationModel\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 2, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c8_SimulationModel_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc8_SimulationModelInstanceStruct *chartInstance;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    chartInstance = (SFc8_SimulationModelInstanceStruct *)
      chartInfo->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _SimulationModelMachineNumber_,
           8,
           1,
           1,
           0,
           5,
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
          _SFD_SET_DATA_PROPS(0,1,1,0,"robotPos");
          _SFD_SET_DATA_PROPS(1,1,1,0,"dist");
          _SFD_SET_DATA_PROPS(2,2,0,1,"locatedTargets");
          _SFD_SET_DATA_PROPS(3,1,1,0,"bearing");
          _SFD_SET_DATA_PROPS(4,1,1,0,"robotTheta");
          _SFD_STATE_INFO(0,0,2);
          _SFD_CH_SUBSTATE_COUNT(0);
          _SFD_CH_SUBSTATE_DECOMP(0);
        }

        _SFD_CV_INIT_CHART(0,0,0,0);

        {
          _SFD_CV_INIT_STATE(0,0,0,0,0,0,NULL,NULL);
        }

        _SFD_CV_INIT_TRANS(0,0,NULL,NULL,0,NULL);

        /* Initialization of MATLAB Function Model Coverage */
        _SFD_CV_INIT_EML(0,1,1,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,402);

        {
          unsigned int dimVector[1];
          dimVector[0]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c8_d_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_SINGLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c8_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 6;
          dimVector[1]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c8_sf_marshallOut,(MexInFcnForType)
            c8_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(3,SF_SINGLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c8_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c8_b_sf_marshallOut,(MexInFcnForType)NULL);

        {
          real_T *c8_robotTheta;
          real_T (*c8_robotPos)[2];
          real32_T (*c8_dist)[6];
          real_T (*c8_locatedTargets)[12];
          real32_T (*c8_bearing)[6];
          c8_robotTheta = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
          c8_bearing = (real32_T (*)[6])ssGetInputPortSignal(chartInstance->S, 2);
          c8_locatedTargets = (real_T (*)[12])ssGetOutputPortSignal
            (chartInstance->S, 1);
          c8_dist = (real32_T (*)[6])ssGetInputPortSignal(chartInstance->S, 1);
          c8_robotPos = (real_T (*)[2])ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, *c8_robotPos);
          _SFD_SET_DATA_VALUE_PTR(1U, *c8_dist);
          _SFD_SET_DATA_VALUE_PTR(2U, *c8_locatedTargets);
          _SFD_SET_DATA_VALUE_PTR(3U, *c8_bearing);
          _SFD_SET_DATA_VALUE_PTR(4U, c8_robotTheta);
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
  return "Zs3TLFDt54K99IhInIYMoE";
}

static void sf_opaque_initialize_c8_SimulationModel(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc8_SimulationModelInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c8_SimulationModel((SFc8_SimulationModelInstanceStruct*)
    chartInstanceVar);
  initialize_c8_SimulationModel((SFc8_SimulationModelInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c8_SimulationModel(void *chartInstanceVar)
{
  enable_c8_SimulationModel((SFc8_SimulationModelInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c8_SimulationModel(void *chartInstanceVar)
{
  disable_c8_SimulationModel((SFc8_SimulationModelInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c8_SimulationModel(void *chartInstanceVar)
{
  sf_gateway_c8_SimulationModel((SFc8_SimulationModelInstanceStruct*)
    chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c8_SimulationModel(SimStruct* S)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c8_SimulationModel
    ((SFc8_SimulationModelInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c8_SimulationModel();/* state var info */
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

extern void sf_internal_set_sim_state_c8_SimulationModel(SimStruct* S, const
  mxArray *st)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[3];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxDuplicateArray(st);      /* high level simctx */
  prhs[2] = (mxArray*) sf_get_sim_state_info_c8_SimulationModel();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 3, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c8_SimulationModel((SFc8_SimulationModelInstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c8_SimulationModel(SimStruct* S)
{
  return sf_internal_get_sim_state_c8_SimulationModel(S);
}

static void sf_opaque_set_sim_state_c8_SimulationModel(SimStruct* S, const
  mxArray *st)
{
  sf_internal_set_sim_state_c8_SimulationModel(S, st);
}

static void sf_opaque_terminate_c8_SimulationModel(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc8_SimulationModelInstanceStruct*) chartInstanceVar)->S;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_SimulationModel_optimization_info();
    }

    finalize_c8_SimulationModel((SFc8_SimulationModelInstanceStruct*)
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
  initSimStructsc8_SimulationModel((SFc8_SimulationModelInstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c8_SimulationModel(SimStruct *S)
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
    initialize_params_c8_SimulationModel((SFc8_SimulationModelInstanceStruct*)
      (chartInfo->chartInstance));
  }
}

static void mdlSetWorkWidths_c8_SimulationModel(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_SimulationModel_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,8);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(sf_get_instance_specialization(),
                infoStruct,8,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,8,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(sf_get_instance_specialization(),infoStruct,8);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,8,4);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,8,1);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=1; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 4; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,8);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(2269641184U));
  ssSetChecksum1(S,(1948048890U));
  ssSetChecksum2(S,(4112146366U));
  ssSetChecksum3(S,(1994868857U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c8_SimulationModel(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c8_SimulationModel(SimStruct *S)
{
  SFc8_SimulationModelInstanceStruct *chartInstance;
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)utMalloc(sizeof
    (ChartRunTimeInfo));
  chartInstance = (SFc8_SimulationModelInstanceStruct *)utMalloc(sizeof
    (SFc8_SimulationModelInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc8_SimulationModelInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c8_SimulationModel;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c8_SimulationModel;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c8_SimulationModel;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c8_SimulationModel;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c8_SimulationModel;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c8_SimulationModel;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c8_SimulationModel;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c8_SimulationModel;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c8_SimulationModel;
  chartInstance->chartInfo.mdlStart = mdlStart_c8_SimulationModel;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c8_SimulationModel;
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

void c8_SimulationModel_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c8_SimulationModel(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c8_SimulationModel(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c8_SimulationModel(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c8_SimulationModel_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
