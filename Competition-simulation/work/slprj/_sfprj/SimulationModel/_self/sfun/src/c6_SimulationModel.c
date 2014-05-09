/* Include files */

#include <stddef.h>
#include "blas.h"
#include "SimulationModel_sfun.h"
#include "c6_SimulationModel.h"
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
static const char * c6_debug_family_names[11] = { "nargin", "nargout", "X", "T",
  "T_found", "t_last", "t", "T_located", "pcam", "lcam", "robotheta" };

/* Function Declarations */
static void initialize_c6_SimulationModel(SFc6_SimulationModelInstanceStruct
  *chartInstance);
static void initialize_params_c6_SimulationModel
  (SFc6_SimulationModelInstanceStruct *chartInstance);
static void enable_c6_SimulationModel(SFc6_SimulationModelInstanceStruct
  *chartInstance);
static void disable_c6_SimulationModel(SFc6_SimulationModelInstanceStruct
  *chartInstance);
static void c6_update_debugger_state_c6_SimulationModel
  (SFc6_SimulationModelInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c6_SimulationModel
  (SFc6_SimulationModelInstanceStruct *chartInstance);
static void set_sim_state_c6_SimulationModel(SFc6_SimulationModelInstanceStruct *
  chartInstance, const mxArray *c6_st);
static void finalize_c6_SimulationModel(SFc6_SimulationModelInstanceStruct
  *chartInstance);
static void sf_gateway_c6_SimulationModel(SFc6_SimulationModelInstanceStruct
  *chartInstance);
static void initSimStructsc6_SimulationModel(SFc6_SimulationModelInstanceStruct *
  chartInstance);
static void init_script_number_translation(uint32_T c6_machineNumber, uint32_T
  c6_chartNumber, uint32_T c6_instanceNumber);
static const mxArray *c6_sf_marshallOut(void *chartInstanceVoid, void *c6_inData);
static const mxArray *c6_b_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static void c6_emlrt_marshallIn(SFc6_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[2]);
static void c6_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static const mxArray *c6_c_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static const mxArray *c6_d_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static const mxArray *c6_e_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static real_T c6_b_emlrt_marshallIn(SFc6_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId);
static void c6_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static const mxArray *c6_f_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static int32_T c6_c_emlrt_marshallIn(SFc6_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId);
static void c6_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static uint8_T c6_d_emlrt_marshallIn(SFc6_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c6_b_is_active_c6_SimulationModel, const char_T
  *c6_identifier);
static uint8_T c6_e_emlrt_marshallIn(SFc6_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId);
static void init_dsm_address_info(SFc6_SimulationModelInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c6_SimulationModel(SFc6_SimulationModelInstanceStruct
  *chartInstance)
{
  chartInstance->c6_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c6_is_active_c6_SimulationModel = 0U;
}

static void initialize_params_c6_SimulationModel
  (SFc6_SimulationModelInstanceStruct *chartInstance)
{
  real_T c6_dv0[2];
  int32_T c6_i0;
  real_T c6_dv1[2];
  int32_T c6_i1;
  sf_mex_import_named("pcam", sf_mex_get_sfun_param(chartInstance->S, 1, 0),
                      c6_dv0, 0, 0, 0U, 1, 0U, 2, 1, 2);
  for (c6_i0 = 0; c6_i0 < 2; c6_i0++) {
    chartInstance->c6_pcam[c6_i0] = c6_dv0[c6_i0];
  }

  sf_mex_import_named("lcam", sf_mex_get_sfun_param(chartInstance->S, 0, 0),
                      c6_dv1, 0, 0, 0U, 1, 0U, 2, 1, 2);
  for (c6_i1 = 0; c6_i1 < 2; c6_i1++) {
    chartInstance->c6_lcam[c6_i1] = c6_dv1[c6_i1];
  }
}

static void enable_c6_SimulationModel(SFc6_SimulationModelInstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c6_SimulationModel(SFc6_SimulationModelInstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c6_update_debugger_state_c6_SimulationModel
  (SFc6_SimulationModelInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c6_SimulationModel
  (SFc6_SimulationModelInstanceStruct *chartInstance)
{
  const mxArray *c6_st;
  const mxArray *c6_y = NULL;
  uint8_T c6_hoistedGlobal;
  uint8_T c6_u;
  const mxArray *c6_b_y = NULL;
  c6_st = NULL;
  c6_st = NULL;
  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_createcellmatrix(1, 1), false);
  c6_hoistedGlobal = chartInstance->c6_is_active_c6_SimulationModel;
  c6_u = c6_hoistedGlobal;
  c6_b_y = NULL;
  sf_mex_assign(&c6_b_y, sf_mex_create("y", &c6_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c6_y, 0, c6_b_y);
  sf_mex_assign(&c6_st, c6_y, false);
  return c6_st;
}

static void set_sim_state_c6_SimulationModel(SFc6_SimulationModelInstanceStruct *
  chartInstance, const mxArray *c6_st)
{
  const mxArray *c6_u;
  chartInstance->c6_doneDoubleBufferReInit = true;
  c6_u = sf_mex_dup(c6_st);
  chartInstance->c6_is_active_c6_SimulationModel = c6_d_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c6_u, 0)),
     "is_active_c6_SimulationModel");
  sf_mex_destroy(&c6_u);
  c6_update_debugger_state_c6_SimulationModel(chartInstance);
  sf_mex_destroy(&c6_st);
}

static void finalize_c6_SimulationModel(SFc6_SimulationModelInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c6_SimulationModel(SFc6_SimulationModelInstanceStruct
  *chartInstance)
{
  int32_T c6_i2;
  int32_T c6_i3;
  int32_T c6_i4;
  int32_T c6_i5;
  int32_T c6_i6;
  int32_T c6_i7;
  real_T c6_hoistedGlobal;
  real_T c6_b_hoistedGlobal;
  real_T c6_c_hoistedGlobal;
  int32_T c6_i8;
  real_T c6_X[2];
  int32_T c6_i9;
  real32_T c6_T[16];
  int32_T c6_i10;
  boolean_T c6_T_found[8];
  real_T c6_t_last;
  real_T c6_t;
  int32_T c6_i11;
  real_T c6_T_located[12];
  int32_T c6_i12;
  real_T c6_b_pcam[2];
  int32_T c6_i13;
  real_T c6_b_lcam[2];
  real_T c6_robotheta;
  uint32_T c6_debug_family_var_map[11];
  real_T c6_nargin = 9.0;
  real_T c6_nargout = 0.0;
  int32_T c6_i14;
  real_T c6_u[2];
  const mxArray *c6_y = NULL;
  int32_T c6_i15;
  real32_T c6_b_u[16];
  const mxArray *c6_b_y = NULL;
  int32_T c6_i16;
  boolean_T c6_c_u[8];
  const mxArray *c6_c_y = NULL;
  real_T c6_d_u;
  const mxArray *c6_d_y = NULL;
  real_T c6_e_u;
  const mxArray *c6_e_y = NULL;
  int32_T c6_i17;
  real_T c6_f_u[12];
  const mxArray *c6_f_y = NULL;
  int32_T c6_i18;
  real_T c6_g_u[2];
  const mxArray *c6_g_y = NULL;
  int32_T c6_i19;
  real_T c6_h_u[2];
  const mxArray *c6_h_y = NULL;
  real_T c6_i_u;
  const mxArray *c6_i_y = NULL;
  real_T *c6_b_t_last;
  real_T *c6_b_t;
  real_T *c6_b_robotheta;
  real_T (*c6_b_T_located)[12];
  boolean_T (*c6_b_T_found)[8];
  real32_T (*c6_b_T)[16];
  real_T (*c6_b_X)[2];
  c6_b_robotheta = (real_T *)ssGetInputPortSignal(chartInstance->S, 6);
  c6_b_T_located = (real_T (*)[12])ssGetInputPortSignal(chartInstance->S, 5);
  c6_b_t = (real_T *)ssGetInputPortSignal(chartInstance->S, 4);
  c6_b_t_last = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
  c6_b_T_found = (boolean_T (*)[8])ssGetInputPortSignal(chartInstance->S, 2);
  c6_b_T = (real32_T (*)[16])ssGetInputPortSignal(chartInstance->S, 1);
  c6_b_X = (real_T (*)[2])ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 2U, chartInstance->c6_sfEvent);
  for (c6_i2 = 0; c6_i2 < 2; c6_i2++) {
    _SFD_DATA_RANGE_CHECK((*c6_b_X)[c6_i2], 0U);
  }

  for (c6_i3 = 0; c6_i3 < 16; c6_i3++) {
    _SFD_DATA_RANGE_CHECK((real_T)(*c6_b_T)[c6_i3], 1U);
  }

  for (c6_i4 = 0; c6_i4 < 8; c6_i4++) {
    _SFD_DATA_RANGE_CHECK((real_T)(*c6_b_T_found)[c6_i4], 2U);
  }

  _SFD_DATA_RANGE_CHECK(*c6_b_t_last, 3U);
  _SFD_DATA_RANGE_CHECK(*c6_b_t, 4U);
  for (c6_i5 = 0; c6_i5 < 12; c6_i5++) {
    _SFD_DATA_RANGE_CHECK((*c6_b_T_located)[c6_i5], 5U);
  }

  for (c6_i6 = 0; c6_i6 < 2; c6_i6++) {
    _SFD_DATA_RANGE_CHECK(chartInstance->c6_pcam[c6_i6], 6U);
  }

  for (c6_i7 = 0; c6_i7 < 2; c6_i7++) {
    _SFD_DATA_RANGE_CHECK(chartInstance->c6_lcam[c6_i7], 7U);
  }

  _SFD_DATA_RANGE_CHECK(*c6_b_robotheta, 8U);
  chartInstance->c6_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 2U, chartInstance->c6_sfEvent);
  c6_hoistedGlobal = *c6_b_t_last;
  c6_b_hoistedGlobal = *c6_b_t;
  c6_c_hoistedGlobal = *c6_b_robotheta;
  for (c6_i8 = 0; c6_i8 < 2; c6_i8++) {
    c6_X[c6_i8] = (*c6_b_X)[c6_i8];
  }

  for (c6_i9 = 0; c6_i9 < 16; c6_i9++) {
    c6_T[c6_i9] = (*c6_b_T)[c6_i9];
  }

  for (c6_i10 = 0; c6_i10 < 8; c6_i10++) {
    c6_T_found[c6_i10] = (*c6_b_T_found)[c6_i10];
  }

  c6_t_last = c6_hoistedGlobal;
  c6_t = c6_b_hoistedGlobal;
  for (c6_i11 = 0; c6_i11 < 12; c6_i11++) {
    c6_T_located[c6_i11] = (*c6_b_T_located)[c6_i11];
  }

  for (c6_i12 = 0; c6_i12 < 2; c6_i12++) {
    c6_b_pcam[c6_i12] = chartInstance->c6_pcam[c6_i12];
  }

  for (c6_i13 = 0; c6_i13 < 2; c6_i13++) {
    c6_b_lcam[c6_i13] = chartInstance->c6_lcam[c6_i13];
  }

  c6_robotheta = c6_c_hoistedGlobal;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 11U, 11U, c6_debug_family_names,
    c6_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_nargin, 0U, c6_sf_marshallOut,
    c6_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_nargout, 1U, c6_sf_marshallOut,
    c6_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c6_X, 2U, c6_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c6_T, 3U, c6_e_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c6_T_found, 4U, c6_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c6_t_last, 5U, c6_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c6_t, 6U, c6_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c6_T_located, 7U, c6_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c6_b_pcam, 8U, c6_b_sf_marshallOut,
    c6_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c6_b_lcam, 9U, c6_b_sf_marshallOut,
    c6_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c6_robotheta, 10U, c6_sf_marshallOut);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 2);
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 4);
  for (c6_i14 = 0; c6_i14 < 2; c6_i14++) {
    c6_u[c6_i14] = c6_X[c6_i14];
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 2, 1, 2), false);
  for (c6_i15 = 0; c6_i15 < 16; c6_i15++) {
    c6_b_u[c6_i15] = c6_T[c6_i15];
  }

  c6_b_y = NULL;
  sf_mex_assign(&c6_b_y, sf_mex_create("y", c6_b_u, 1, 0U, 1U, 0U, 2, 8, 2),
                false);
  for (c6_i16 = 0; c6_i16 < 8; c6_i16++) {
    c6_c_u[c6_i16] = c6_T_found[c6_i16];
  }

  c6_c_y = NULL;
  sf_mex_assign(&c6_c_y, sf_mex_create("y", c6_c_u, 11, 0U, 1U, 0U, 1, 8), false);
  c6_d_u = c6_t_last;
  c6_d_y = NULL;
  sf_mex_assign(&c6_d_y, sf_mex_create("y", &c6_d_u, 0, 0U, 0U, 0U, 0), false);
  c6_e_u = c6_t;
  c6_e_y = NULL;
  sf_mex_assign(&c6_e_y, sf_mex_create("y", &c6_e_u, 0, 0U, 0U, 0U, 0), false);
  for (c6_i17 = 0; c6_i17 < 12; c6_i17++) {
    c6_f_u[c6_i17] = c6_T_located[c6_i17];
  }

  c6_f_y = NULL;
  sf_mex_assign(&c6_f_y, sf_mex_create("y", c6_f_u, 0, 0U, 1U, 0U, 2, 6, 2),
                false);
  for (c6_i18 = 0; c6_i18 < 2; c6_i18++) {
    c6_g_u[c6_i18] = c6_b_pcam[c6_i18];
  }

  c6_g_y = NULL;
  sf_mex_assign(&c6_g_y, sf_mex_create("y", c6_g_u, 0, 0U, 1U, 0U, 2, 1, 2),
                false);
  for (c6_i19 = 0; c6_i19 < 2; c6_i19++) {
    c6_h_u[c6_i19] = c6_b_lcam[c6_i19];
  }

  c6_h_y = NULL;
  sf_mex_assign(&c6_h_y, sf_mex_create("y", c6_h_u, 0, 0U, 1U, 0U, 2, 1, 2),
                false);
  c6_i_u = c6_robotheta;
  c6_i_y = NULL;
  sf_mex_assign(&c6_i_y, sf_mex_create("y", &c6_i_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "plotFigure", 0U, 9U, 14, c6_y,
                    14, c6_b_y, 14, c6_c_y, 14, c6_d_y, 14, c6_e_y, 14, c6_f_y,
                    14, c6_g_y, 14, c6_h_y, 14, c6_i_y);
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, -4);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 2U, chartInstance->c6_sfEvent);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_SimulationModelMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void initSimStructsc6_SimulationModel(SFc6_SimulationModelInstanceStruct *
  chartInstance)
{
  (void)chartInstance;
}

static void init_script_number_translation(uint32_T c6_machineNumber, uint32_T
  c6_chartNumber, uint32_T c6_instanceNumber)
{
  (void)c6_machineNumber;
  (void)c6_chartNumber;
  (void)c6_instanceNumber;
}

static const mxArray *c6_sf_marshallOut(void *chartInstanceVoid, void *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  real_T c6_u;
  const mxArray *c6_y = NULL;
  SFc6_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc6_SimulationModelInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  c6_u = *(real_T *)c6_inData;
  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", &c6_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, false);
  return c6_mxArrayOutData;
}

static const mxArray *c6_b_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_i20;
  real_T c6_b_inData[2];
  int32_T c6_i21;
  real_T c6_u[2];
  const mxArray *c6_y = NULL;
  SFc6_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc6_SimulationModelInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  for (c6_i20 = 0; c6_i20 < 2; c6_i20++) {
    c6_b_inData[c6_i20] = (*(real_T (*)[2])c6_inData)[c6_i20];
  }

  for (c6_i21 = 0; c6_i21 < 2; c6_i21++) {
    c6_u[c6_i21] = c6_b_inData[c6_i21];
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 2, 1, 2), false);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, false);
  return c6_mxArrayOutData;
}

static void c6_emlrt_marshallIn(SFc6_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId,
  real_T c6_y[2])
{
  real_T c6_dv2[2];
  int32_T c6_i22;
  (void)chartInstance;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), c6_dv2, 1, 0, 0U, 1, 0U, 2, 1, 2);
  for (c6_i22 = 0; c6_i22 < 2; c6_i22++) {
    c6_y[c6_i22] = c6_dv2[c6_i22];
  }

  sf_mex_destroy(&c6_u);
}

static void c6_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_b_lcam;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  real_T c6_y[2];
  int32_T c6_i23;
  SFc6_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc6_SimulationModelInstanceStruct *)chartInstanceVoid;
  c6_b_lcam = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_b_lcam), &c6_thisId, c6_y);
  sf_mex_destroy(&c6_b_lcam);
  for (c6_i23 = 0; c6_i23 < 2; c6_i23++) {
    (*(real_T (*)[2])c6_outData)[c6_i23] = c6_y[c6_i23];
  }

  sf_mex_destroy(&c6_mxArrayInData);
}

static const mxArray *c6_c_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_i24;
  int32_T c6_i25;
  int32_T c6_i26;
  real_T c6_b_inData[12];
  int32_T c6_i27;
  int32_T c6_i28;
  int32_T c6_i29;
  real_T c6_u[12];
  const mxArray *c6_y = NULL;
  SFc6_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc6_SimulationModelInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  c6_i24 = 0;
  for (c6_i25 = 0; c6_i25 < 2; c6_i25++) {
    for (c6_i26 = 0; c6_i26 < 6; c6_i26++) {
      c6_b_inData[c6_i26 + c6_i24] = (*(real_T (*)[12])c6_inData)[c6_i26 +
        c6_i24];
    }

    c6_i24 += 6;
  }

  c6_i27 = 0;
  for (c6_i28 = 0; c6_i28 < 2; c6_i28++) {
    for (c6_i29 = 0; c6_i29 < 6; c6_i29++) {
      c6_u[c6_i29 + c6_i27] = c6_b_inData[c6_i29 + c6_i27];
    }

    c6_i27 += 6;
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 0, 0U, 1U, 0U, 2, 6, 2), false);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, false);
  return c6_mxArrayOutData;
}

static const mxArray *c6_d_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_i30;
  boolean_T c6_b_inData[8];
  int32_T c6_i31;
  boolean_T c6_u[8];
  const mxArray *c6_y = NULL;
  SFc6_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc6_SimulationModelInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  for (c6_i30 = 0; c6_i30 < 8; c6_i30++) {
    c6_b_inData[c6_i30] = (*(boolean_T (*)[8])c6_inData)[c6_i30];
  }

  for (c6_i31 = 0; c6_i31 < 8; c6_i31++) {
    c6_u[c6_i31] = c6_b_inData[c6_i31];
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 11, 0U, 1U, 0U, 1, 8), false);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, false);
  return c6_mxArrayOutData;
}

static const mxArray *c6_e_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_i32;
  int32_T c6_i33;
  int32_T c6_i34;
  real32_T c6_b_inData[16];
  int32_T c6_i35;
  int32_T c6_i36;
  int32_T c6_i37;
  real32_T c6_u[16];
  const mxArray *c6_y = NULL;
  SFc6_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc6_SimulationModelInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  c6_i32 = 0;
  for (c6_i33 = 0; c6_i33 < 2; c6_i33++) {
    for (c6_i34 = 0; c6_i34 < 8; c6_i34++) {
      c6_b_inData[c6_i34 + c6_i32] = (*(real32_T (*)[16])c6_inData)[c6_i34 +
        c6_i32];
    }

    c6_i32 += 8;
  }

  c6_i35 = 0;
  for (c6_i36 = 0; c6_i36 < 2; c6_i36++) {
    for (c6_i37 = 0; c6_i37 < 8; c6_i37++) {
      c6_u[c6_i37 + c6_i35] = c6_b_inData[c6_i37 + c6_i35];
    }

    c6_i35 += 8;
  }

  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 1, 0U, 1U, 0U, 2, 8, 2), false);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, false);
  return c6_mxArrayOutData;
}

static real_T c6_b_emlrt_marshallIn(SFc6_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId)
{
  real_T c6_y;
  real_T c6_d0;
  (void)chartInstance;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), &c6_d0, 1, 0, 0U, 0, 0U, 0);
  c6_y = c6_d0;
  sf_mex_destroy(&c6_u);
  return c6_y;
}

static void c6_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_nargout;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  real_T c6_y;
  SFc6_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc6_SimulationModelInstanceStruct *)chartInstanceVoid;
  c6_nargout = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_y = c6_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_nargout), &c6_thisId);
  sf_mex_destroy(&c6_nargout);
  *(real_T *)c6_outData = c6_y;
  sf_mex_destroy(&c6_mxArrayInData);
}

const mxArray *sf_c6_SimulationModel_get_eml_resolved_functions_info(void)
{
  const mxArray *c6_nameCaptureInfo = NULL;
  c6_nameCaptureInfo = NULL;
  sf_mex_assign(&c6_nameCaptureInfo, sf_mex_create("nameCaptureInfo", NULL, 0,
    0U, 1U, 0U, 2, 0, 1), false);
  return c6_nameCaptureInfo;
}

static const mxArray *c6_f_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_u;
  const mxArray *c6_y = NULL;
  SFc6_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc6_SimulationModelInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  c6_u = *(int32_T *)c6_inData;
  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", &c6_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, false);
  return c6_mxArrayOutData;
}

static int32_T c6_c_emlrt_marshallIn(SFc6_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId)
{
  int32_T c6_y;
  int32_T c6_i38;
  (void)chartInstance;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), &c6_i38, 1, 6, 0U, 0, 0U, 0);
  c6_y = c6_i38;
  sf_mex_destroy(&c6_u);
  return c6_y;
}

static void c6_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_b_sfEvent;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  int32_T c6_y;
  SFc6_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc6_SimulationModelInstanceStruct *)chartInstanceVoid;
  c6_b_sfEvent = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_y = c6_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_b_sfEvent),
    &c6_thisId);
  sf_mex_destroy(&c6_b_sfEvent);
  *(int32_T *)c6_outData = c6_y;
  sf_mex_destroy(&c6_mxArrayInData);
}

static uint8_T c6_d_emlrt_marshallIn(SFc6_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c6_b_is_active_c6_SimulationModel, const char_T
  *c6_identifier)
{
  uint8_T c6_y;
  emlrtMsgIdentifier c6_thisId;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_y = c6_e_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c6_b_is_active_c6_SimulationModel), &c6_thisId);
  sf_mex_destroy(&c6_b_is_active_c6_SimulationModel);
  return c6_y;
}

static uint8_T c6_e_emlrt_marshallIn(SFc6_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId)
{
  uint8_T c6_y;
  uint8_T c6_u0;
  (void)chartInstance;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), &c6_u0, 1, 3, 0U, 0, 0U, 0);
  c6_y = c6_u0;
  sf_mex_destroy(&c6_u);
  return c6_y;
}

static void init_dsm_address_info(SFc6_SimulationModelInstanceStruct
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

void sf_c6_SimulationModel_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(814359419U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(1532757517U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1883475851U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(464537683U);
}

mxArray *sf_c6_SimulationModel_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("e9eNiyjX7FdpIfaAp4Uz5E");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,7,3,dataFields);

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
      pr[0] = (double)(8);
      pr[1] = (double)(2);
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
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,4,"type",mxType);
    }

    mxSetField(mxData,4,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(6);
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
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,2,3,dataFields);

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
      pr[1] = (double)(2);
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
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c6_SimulationModel_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c6_SimulationModel_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

static const mxArray *sf_get_sim_state_info_c6_SimulationModel(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S'type','srcId','name','auxInfo'{{M[8],M[0],T\"is_active_c6_SimulationModel\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 1, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c6_SimulationModel_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc6_SimulationModelInstanceStruct *chartInstance;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    chartInstance = (SFc6_SimulationModelInstanceStruct *)
      chartInfo->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _SimulationModelMachineNumber_,
           6,
           1,
           1,
           0,
           9,
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
          _SFD_SET_DATA_PROPS(0,1,1,0,"X");
          _SFD_SET_DATA_PROPS(1,1,1,0,"T");
          _SFD_SET_DATA_PROPS(2,1,1,0,"T_found");
          _SFD_SET_DATA_PROPS(3,1,1,0,"t_last");
          _SFD_SET_DATA_PROPS(4,1,1,0,"t");
          _SFD_SET_DATA_PROPS(5,1,1,0,"T_located");
          _SFD_SET_DATA_PROPS(6,10,0,0,"pcam");
          _SFD_SET_DATA_PROPS(7,10,0,0,"lcam");
          _SFD_SET_DATA_PROPS(8,1,1,0,"robotheta");
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
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,171);

        {
          unsigned int dimVector[2];
          dimVector[0]= 1;
          dimVector[1]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c6_b_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 8;
          dimVector[1]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_SINGLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c6_e_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 8;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_UINT8,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c6_d_sf_marshallOut,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c6_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c6_sf_marshallOut,(MexInFcnForType)NULL);

        {
          unsigned int dimVector[2];
          dimVector[0]= 6;
          dimVector[1]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c6_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 1;
          dimVector[1]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c6_b_sf_marshallOut,(MexInFcnForType)
            c6_sf_marshallIn);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 1;
          dimVector[1]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(7,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c6_b_sf_marshallOut,(MexInFcnForType)
            c6_sf_marshallIn);
        }

        _SFD_SET_DATA_COMPILED_PROPS(8,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c6_sf_marshallOut,(MexInFcnForType)NULL);

        {
          real_T *c6_t_last;
          real_T *c6_t;
          real_T *c6_robotheta;
          real_T (*c6_X)[2];
          real32_T (*c6_T)[16];
          boolean_T (*c6_T_found)[8];
          real_T (*c6_T_located)[12];
          c6_robotheta = (real_T *)ssGetInputPortSignal(chartInstance->S, 6);
          c6_T_located = (real_T (*)[12])ssGetInputPortSignal(chartInstance->S,
            5);
          c6_t = (real_T *)ssGetInputPortSignal(chartInstance->S, 4);
          c6_t_last = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
          c6_T_found = (boolean_T (*)[8])ssGetInputPortSignal(chartInstance->S,
            2);
          c6_T = (real32_T (*)[16])ssGetInputPortSignal(chartInstance->S, 1);
          c6_X = (real_T (*)[2])ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, *c6_X);
          _SFD_SET_DATA_VALUE_PTR(1U, *c6_T);
          _SFD_SET_DATA_VALUE_PTR(2U, *c6_T_found);
          _SFD_SET_DATA_VALUE_PTR(3U, c6_t_last);
          _SFD_SET_DATA_VALUE_PTR(4U, c6_t);
          _SFD_SET_DATA_VALUE_PTR(5U, *c6_T_located);
          _SFD_SET_DATA_VALUE_PTR(6U, chartInstance->c6_pcam);
          _SFD_SET_DATA_VALUE_PTR(7U, chartInstance->c6_lcam);
          _SFD_SET_DATA_VALUE_PTR(8U, c6_robotheta);
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
  return "uXor2wIQrt8ncPgIOdiUrB";
}

static void sf_opaque_initialize_c6_SimulationModel(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc6_SimulationModelInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c6_SimulationModel((SFc6_SimulationModelInstanceStruct*)
    chartInstanceVar);
  initialize_c6_SimulationModel((SFc6_SimulationModelInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c6_SimulationModel(void *chartInstanceVar)
{
  enable_c6_SimulationModel((SFc6_SimulationModelInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c6_SimulationModel(void *chartInstanceVar)
{
  disable_c6_SimulationModel((SFc6_SimulationModelInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c6_SimulationModel(void *chartInstanceVar)
{
  sf_gateway_c6_SimulationModel((SFc6_SimulationModelInstanceStruct*)
    chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c6_SimulationModel(SimStruct* S)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c6_SimulationModel
    ((SFc6_SimulationModelInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c6_SimulationModel();/* state var info */
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

extern void sf_internal_set_sim_state_c6_SimulationModel(SimStruct* S, const
  mxArray *st)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[3];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxDuplicateArray(st);      /* high level simctx */
  prhs[2] = (mxArray*) sf_get_sim_state_info_c6_SimulationModel();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 3, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c6_SimulationModel((SFc6_SimulationModelInstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c6_SimulationModel(SimStruct* S)
{
  return sf_internal_get_sim_state_c6_SimulationModel(S);
}

static void sf_opaque_set_sim_state_c6_SimulationModel(SimStruct* S, const
  mxArray *st)
{
  sf_internal_set_sim_state_c6_SimulationModel(S, st);
}

static void sf_opaque_terminate_c6_SimulationModel(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc6_SimulationModelInstanceStruct*) chartInstanceVar)->S;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_SimulationModel_optimization_info();
    }

    finalize_c6_SimulationModel((SFc6_SimulationModelInstanceStruct*)
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
  initSimStructsc6_SimulationModel((SFc6_SimulationModelInstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c6_SimulationModel(SimStruct *S)
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
    initialize_params_c6_SimulationModel((SFc6_SimulationModelInstanceStruct*)
      (chartInfo->chartInstance));
  }
}

static void mdlSetWorkWidths_c6_SimulationModel(SimStruct *S)
{
  /* Actual parameters from chart:
     lcam pcam
   */
  const char_T *rtParamNames[] = { "lcam", "pcam" };

  ssSetNumRunTimeParams(S,ssGetSFcnParamsCount(S));

  /* registration for lcam*/
  ssRegDlgParamAsRunTimeParam(S, 0, 0, rtParamNames[0], SS_DOUBLE);

  /* registration for pcam*/
  ssRegDlgParamAsRunTimeParam(S, 1, 1, rtParamNames[1], SS_DOUBLE);
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_SimulationModel_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,6);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(sf_get_instance_specialization(),
                infoStruct,6,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,6,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(sf_get_instance_specialization(),infoStruct,6);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 4, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 5, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 6, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,6,7);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=0; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 7; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,6);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(2985898038U));
  ssSetChecksum1(S,(2756505485U));
  ssSetChecksum2(S,(2844862812U));
  ssSetChecksum3(S,(2476750111U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c6_SimulationModel(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c6_SimulationModel(SimStruct *S)
{
  SFc6_SimulationModelInstanceStruct *chartInstance;
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)utMalloc(sizeof
    (ChartRunTimeInfo));
  chartInstance = (SFc6_SimulationModelInstanceStruct *)utMalloc(sizeof
    (SFc6_SimulationModelInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc6_SimulationModelInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c6_SimulationModel;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c6_SimulationModel;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c6_SimulationModel;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c6_SimulationModel;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c6_SimulationModel;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c6_SimulationModel;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c6_SimulationModel;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c6_SimulationModel;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c6_SimulationModel;
  chartInstance->chartInfo.mdlStart = mdlStart_c6_SimulationModel;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c6_SimulationModel;
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

void c6_SimulationModel_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c6_SimulationModel(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c6_SimulationModel(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c6_SimulationModel(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c6_SimulationModel_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
