/* Include files */

#include <stddef.h>
#include "blas.h"
#include "SimulationModel_sfun.h"
#include "c7_SimulationModel.h"
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
static const char * c7_debug_family_names[14] = { "x", "y", "distance",
  "bearing", "idxsort", "nargin", "nargout", "robotPos", "robotTheta",
  "targetsPos", "pcam", "lcam", "dist", "bear" };

static const char * c7_b_debug_family_names[11] = { "x_dif", "y_dif", "nargin",
  "nargout", "x_robot", "y_robot", "theta_robot", "x_world", "y_world", "x_new",
  "y_new" };

static const char * c7_c_debug_family_names[11] = { "c1", "c2", "isvisible",
  "nargin", "nargout", "p", "l", "x", "y", "dist", "bearing" };

/* Function Declarations */
static void initialize_c7_SimulationModel(SFc7_SimulationModelInstanceStruct
  *chartInstance);
static void initialize_params_c7_SimulationModel
  (SFc7_SimulationModelInstanceStruct *chartInstance);
static void enable_c7_SimulationModel(SFc7_SimulationModelInstanceStruct
  *chartInstance);
static void disable_c7_SimulationModel(SFc7_SimulationModelInstanceStruct
  *chartInstance);
static void c7_update_debugger_state_c7_SimulationModel
  (SFc7_SimulationModelInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c7_SimulationModel
  (SFc7_SimulationModelInstanceStruct *chartInstance);
static void set_sim_state_c7_SimulationModel(SFc7_SimulationModelInstanceStruct *
  chartInstance, const mxArray *c7_st);
static void finalize_c7_SimulationModel(SFc7_SimulationModelInstanceStruct
  *chartInstance);
static void sf_gateway_c7_SimulationModel(SFc7_SimulationModelInstanceStruct
  *chartInstance);
static void initSimStructsc7_SimulationModel(SFc7_SimulationModelInstanceStruct *
  chartInstance);
static void init_script_number_translation(uint32_T c7_machineNumber, uint32_T
  c7_chartNumber, uint32_T c7_instanceNumber);
static const mxArray *c7_sf_marshallOut(void *chartInstanceVoid, void *c7_inData);
static void c7_emlrt_marshallIn(SFc7_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c7_bear, const char_T *c7_identifier, real32_T
  c7_y[6]);
static void c7_b_emlrt_marshallIn(SFc7_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId,
  real32_T c7_y[6]);
static void c7_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData);
static const mxArray *c7_b_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData);
static void c7_c_emlrt_marshallIn(SFc7_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId,
  real_T c7_y[2]);
static void c7_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData);
static const mxArray *c7_c_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData);
static const mxArray *c7_d_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData);
static real_T c7_d_emlrt_marshallIn(SFc7_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId);
static void c7_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData);
static const mxArray *c7_e_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData);
static void c7_e_emlrt_marshallIn(SFc7_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId,
  real_T c7_y[8]);
static void c7_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData);
static const mxArray *c7_f_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData);
static void c7_f_emlrt_marshallIn(SFc7_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId,
  real32_T c7_y[8]);
static void c7_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData);
static const mxArray *c7_g_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData);
static void c7_g_emlrt_marshallIn(SFc7_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId,
  boolean_T c7_y[8]);
static void c7_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData);
static void c7_info_helper(const mxArray **c7_info);
static const mxArray *c7_emlrt_marshallOut(const char * c7_u);
static const mxArray *c7_b_emlrt_marshallOut(const uint32_T c7_u);
static void c7_b_info_helper(const mxArray **c7_info);
static void c7_c_info_helper(const mxArray **c7_info);
static void c7_d_info_helper(const mxArray **c7_info);
static void c7_e_info_helper(const mxArray **c7_info);
static real_T c7_cosd(SFc7_SimulationModelInstanceStruct *chartInstance, real_T
                      c7_x);
static boolean_T c7_isfinite(SFc7_SimulationModelInstanceStruct *chartInstance,
  real_T c7_x);
static void c7_eml_scalar_eg(SFc7_SimulationModelInstanceStruct *chartInstance);
static real_T c7_sind(SFc7_SimulationModelInstanceStruct *chartInstance, real_T
                      c7_x);
static void c7_isincamera(SFc7_SimulationModelInstanceStruct *chartInstance,
  real_T c7_p[2], real_T c7_l[2], real32_T c7_x[8], real32_T c7_y[8], real32_T
  c7_dist[8], real32_T c7_bearing[8]);
static void c7_polyfit(SFc7_SimulationModelInstanceStruct *chartInstance, real_T
  c7_x[2], real_T c7_y[2], real_T c7_p[2]);
static void c7_b_eml_scalar_eg(SFc7_SimulationModelInstanceStruct *chartInstance);
static void c7_eml_switch_helper(SFc7_SimulationModelInstanceStruct
  *chartInstance);
static void c7_eps(SFc7_SimulationModelInstanceStruct *chartInstance);
static real_T c7_sqrt(SFc7_SimulationModelInstanceStruct *chartInstance, real_T
                      c7_x);
static void c7_eml_error(SFc7_SimulationModelInstanceStruct *chartInstance);
static void c7_threshold(SFc7_SimulationModelInstanceStruct *chartInstance);
static void c7_realmin(SFc7_SimulationModelInstanceStruct *chartInstance);
static void c7_eml_qrsolve(SFc7_SimulationModelInstanceStruct *chartInstance,
  real_T c7_A[4], real_T c7_B[2], real_T c7_Y[2], real_T *c7_rankR);
static real_T c7_eml_xnrm2(SFc7_SimulationModelInstanceStruct *chartInstance,
  real_T c7_x[4], int32_T c7_ix0);
static void c7_check_forloop_overflow_error(SFc7_SimulationModelInstanceStruct
  *chartInstance, boolean_T c7_overflow);
static int32_T c7_eml_ixamax(SFc7_SimulationModelInstanceStruct *chartInstance,
  int32_T c7_n, real_T c7_x[2], int32_T c7_ix0);
static void c7_eml_xswap(SFc7_SimulationModelInstanceStruct *chartInstance,
  real_T c7_x[4], int32_T c7_ix0, int32_T c7_iy0, real_T c7_b_x[4]);
static void c7_b_threshold(SFc7_SimulationModelInstanceStruct *chartInstance);
static void c7_eml_matlab_zlarfg(SFc7_SimulationModelInstanceStruct
  *chartInstance, int32_T c7_n, real_T c7_alpha1, real_T c7_x[4], int32_T c7_ix0,
  real_T *c7_b_alpha1, real_T c7_b_x[4], real_T *c7_tau);
static real_T c7_b_eml_xnrm2(SFc7_SimulationModelInstanceStruct *chartInstance,
  int32_T c7_n, real_T c7_x[4], int32_T c7_ix0);
static void c7_eml_xscal(SFc7_SimulationModelInstanceStruct *chartInstance,
  int32_T c7_n, real_T c7_a, real_T c7_x[4], int32_T c7_ix0, real_T c7_b_x[4]);
static void c7_c_threshold(SFc7_SimulationModelInstanceStruct *chartInstance);
static void c7_b_eml_matlab_zlarfg(SFc7_SimulationModelInstanceStruct
  *chartInstance, real_T c7_alpha1, real_T c7_x, real_T *c7_b_alpha1, real_T
  *c7_b_x, real_T *c7_tau);
static void c7_c_eml_xnrm2(SFc7_SimulationModelInstanceStruct *chartInstance);
static real_T c7_b_eml_xscal(SFc7_SimulationModelInstanceStruct *chartInstance,
  real_T c7_x);
static void c7_eml_xgemv(SFc7_SimulationModelInstanceStruct *chartInstance,
  int32_T c7_m, int32_T c7_n, real_T c7_A[4], int32_T c7_ia0, real_T c7_x[4],
  int32_T c7_ix0, real_T c7_y[2], real_T c7_b_y[2]);
static void c7_eml_xgerc(SFc7_SimulationModelInstanceStruct *chartInstance,
  int32_T c7_m, int32_T c7_n, real_T c7_alpha1, int32_T c7_ix0, real_T c7_y[2],
  real_T c7_A[4], int32_T c7_ia0, real_T c7_b_A[4]);
static void c7_eml_warning(SFc7_SimulationModelInstanceStruct *chartInstance);
static void c7_polyval(SFc7_SimulationModelInstanceStruct *chartInstance, real_T
  c7_p[2], real32_T c7_x[8], real32_T c7_y[8]);
static void c7_power(SFc7_SimulationModelInstanceStruct *chartInstance, real32_T
                     c7_a[8], real32_T c7_y[8]);
static void c7_eml_li_find(SFc7_SimulationModelInstanceStruct *chartInstance,
  boolean_T c7_x[8], int32_T c7_y_data[], int32_T *c7_y_sizes);
static void c7_eml_sort(SFc7_SimulationModelInstanceStruct *chartInstance,
  real32_T c7_x[8], real32_T c7_y[8], int32_T c7_idx[8]);
static void c7_b_eml_sort(SFc7_SimulationModelInstanceStruct *chartInstance,
  real32_T c7_x[8], real32_T c7_y[8], int32_T c7_idx[8]);
static void c7_eml_sort_idx(SFc7_SimulationModelInstanceStruct *chartInstance,
  real32_T c7_x[8], int32_T c7_idx[8], real32_T c7_y[8]);
static const mxArray *c7_h_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData);
static int32_T c7_h_emlrt_marshallIn(SFc7_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId);
static void c7_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData);
static uint8_T c7_i_emlrt_marshallIn(SFc7_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c7_b_is_active_c7_SimulationModel, const char_T
  *c7_identifier);
static uint8_T c7_j_emlrt_marshallIn(SFc7_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId);
static void c7_b_cosd(SFc7_SimulationModelInstanceStruct *chartInstance, real_T *
                      c7_x);
static void c7_b_sind(SFc7_SimulationModelInstanceStruct *chartInstance, real_T *
                      c7_x);
static void c7_b_sqrt(SFc7_SimulationModelInstanceStruct *chartInstance, real_T *
                      c7_x);
static void c7_b_eml_xswap(SFc7_SimulationModelInstanceStruct *chartInstance,
  real_T c7_x[4], int32_T c7_ix0, int32_T c7_iy0);
static real_T c7_c_eml_matlab_zlarfg(SFc7_SimulationModelInstanceStruct
  *chartInstance, int32_T c7_n, real_T *c7_alpha1, real_T c7_x[4], int32_T
  c7_ix0);
static void c7_c_eml_xscal(SFc7_SimulationModelInstanceStruct *chartInstance,
  int32_T c7_n, real_T c7_a, real_T c7_x[4], int32_T c7_ix0);
static real_T c7_d_eml_matlab_zlarfg(SFc7_SimulationModelInstanceStruct
  *chartInstance, real_T *c7_alpha1, real_T *c7_x);
static void c7_d_eml_xscal(SFc7_SimulationModelInstanceStruct *chartInstance,
  real_T *c7_x);
static void c7_b_eml_xgemv(SFc7_SimulationModelInstanceStruct *chartInstance,
  int32_T c7_m, int32_T c7_n, real_T c7_A[4], int32_T c7_ia0, real_T c7_x[4],
  int32_T c7_ix0, real_T c7_y[2]);
static void c7_b_eml_xgerc(SFc7_SimulationModelInstanceStruct *chartInstance,
  int32_T c7_m, int32_T c7_n, real_T c7_alpha1, int32_T c7_ix0, real_T c7_y[2],
  real_T c7_A[4], int32_T c7_ia0);
static void init_dsm_address_info(SFc7_SimulationModelInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c7_SimulationModel(SFc7_SimulationModelInstanceStruct
  *chartInstance)
{
  chartInstance->c7_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c7_is_active_c7_SimulationModel = 0U;
}

static void initialize_params_c7_SimulationModel
  (SFc7_SimulationModelInstanceStruct *chartInstance)
{
  real_T c7_dv0[2];
  int32_T c7_i0;
  real_T c7_dv1[2];
  int32_T c7_i1;
  sf_mex_import_named("pcam", sf_mex_get_sfun_param(chartInstance->S, 1, 0),
                      c7_dv0, 0, 0, 0U, 1, 0U, 2, 1, 2);
  for (c7_i0 = 0; c7_i0 < 2; c7_i0++) {
    chartInstance->c7_pcam[c7_i0] = c7_dv0[c7_i0];
  }

  sf_mex_import_named("lcam", sf_mex_get_sfun_param(chartInstance->S, 0, 0),
                      c7_dv1, 0, 0, 0U, 1, 0U, 2, 1, 2);
  for (c7_i1 = 0; c7_i1 < 2; c7_i1++) {
    chartInstance->c7_lcam[c7_i1] = c7_dv1[c7_i1];
  }
}

static void enable_c7_SimulationModel(SFc7_SimulationModelInstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c7_SimulationModel(SFc7_SimulationModelInstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c7_update_debugger_state_c7_SimulationModel
  (SFc7_SimulationModelInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c7_SimulationModel
  (SFc7_SimulationModelInstanceStruct *chartInstance)
{
  const mxArray *c7_st;
  const mxArray *c7_y = NULL;
  int32_T c7_i2;
  real32_T c7_u[6];
  const mxArray *c7_b_y = NULL;
  int32_T c7_i3;
  real32_T c7_b_u[6];
  const mxArray *c7_c_y = NULL;
  uint8_T c7_hoistedGlobal;
  uint8_T c7_c_u;
  const mxArray *c7_d_y = NULL;
  real32_T (*c7_dist)[6];
  real32_T (*c7_bear)[6];
  c7_bear = (real32_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 2);
  c7_dist = (real32_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 1);
  c7_st = NULL;
  c7_st = NULL;
  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_createcellmatrix(3, 1), false);
  for (c7_i2 = 0; c7_i2 < 6; c7_i2++) {
    c7_u[c7_i2] = (*c7_bear)[c7_i2];
  }

  c7_b_y = NULL;
  sf_mex_assign(&c7_b_y, sf_mex_create("y", c7_u, 1, 0U, 1U, 0U, 1, 6), false);
  sf_mex_setcell(c7_y, 0, c7_b_y);
  for (c7_i3 = 0; c7_i3 < 6; c7_i3++) {
    c7_b_u[c7_i3] = (*c7_dist)[c7_i3];
  }

  c7_c_y = NULL;
  sf_mex_assign(&c7_c_y, sf_mex_create("y", c7_b_u, 1, 0U, 1U, 0U, 1, 6), false);
  sf_mex_setcell(c7_y, 1, c7_c_y);
  c7_hoistedGlobal = chartInstance->c7_is_active_c7_SimulationModel;
  c7_c_u = c7_hoistedGlobal;
  c7_d_y = NULL;
  sf_mex_assign(&c7_d_y, sf_mex_create("y", &c7_c_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c7_y, 2, c7_d_y);
  sf_mex_assign(&c7_st, c7_y, false);
  return c7_st;
}

static void set_sim_state_c7_SimulationModel(SFc7_SimulationModelInstanceStruct *
  chartInstance, const mxArray *c7_st)
{
  const mxArray *c7_u;
  real32_T c7_fv0[6];
  int32_T c7_i4;
  real32_T c7_fv1[6];
  int32_T c7_i5;
  real32_T (*c7_bear)[6];
  real32_T (*c7_dist)[6];
  c7_bear = (real32_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 2);
  c7_dist = (real32_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c7_doneDoubleBufferReInit = true;
  c7_u = sf_mex_dup(c7_st);
  c7_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c7_u, 0)), "bear",
                      c7_fv0);
  for (c7_i4 = 0; c7_i4 < 6; c7_i4++) {
    (*c7_bear)[c7_i4] = c7_fv0[c7_i4];
  }

  c7_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c7_u, 1)), "dist",
                      c7_fv1);
  for (c7_i5 = 0; c7_i5 < 6; c7_i5++) {
    (*c7_dist)[c7_i5] = c7_fv1[c7_i5];
  }

  chartInstance->c7_is_active_c7_SimulationModel = c7_i_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c7_u, 2)),
     "is_active_c7_SimulationModel");
  sf_mex_destroy(&c7_u);
  c7_update_debugger_state_c7_SimulationModel(chartInstance);
  sf_mex_destroy(&c7_st);
}

static void finalize_c7_SimulationModel(SFc7_SimulationModelInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c7_SimulationModel(SFc7_SimulationModelInstanceStruct
  *chartInstance)
{
  int32_T c7_i6;
  real_T c7_hoistedGlobal;
  int32_T c7_i7;
  real_T c7_robotPos[2];
  real_T c7_robotTheta;
  int32_T c7_i8;
  real32_T c7_targetsPos[16];
  int32_T c7_i9;
  real_T c7_b_pcam[2];
  int32_T c7_i10;
  real_T c7_b_lcam[2];
  uint32_T c7_debug_family_var_map[14];
  real32_T c7_x[8];
  real32_T c7_y[8];
  real32_T c7_distance[8];
  real32_T c7_bearing[8];
  real_T c7_idxsort[8];
  real_T c7_nargin = 5.0;
  real_T c7_nargout = 2.0;
  real32_T c7_dist[6];
  real32_T c7_bear[6];
  real_T c7_x_robot;
  real_T c7_y_robot;
  real_T c7_theta_robot;
  int32_T c7_i11;
  real32_T c7_x_world[8];
  int32_T c7_i12;
  real32_T c7_y_world[8];
  uint32_T c7_b_debug_family_var_map[11];
  real32_T c7_x_dif[8];
  real32_T c7_y_dif[8];
  real_T c7_b_nargin = 5.0;
  real_T c7_b_nargout = 2.0;
  real32_T c7_b_x[8];
  real32_T c7_b_y[8];
  int32_T c7_i13;
  int32_T c7_i14;
  real_T c7_a;
  int32_T c7_i15;
  real32_T c7_b[8];
  int32_T c7_i16;
  real_T c7_b_a;
  int32_T c7_i17;
  real32_T c7_b_b[8];
  int32_T c7_i18;
  int32_T c7_i19;
  real_T c7_c_a;
  int32_T c7_i20;
  int32_T c7_i21;
  real_T c7_d_a;
  int32_T c7_i22;
  int32_T c7_i23;
  int32_T c7_i24;
  int32_T c7_i25;
  int32_T c7_i26;
  int32_T c7_i27;
  real_T c7_c_pcam[2];
  int32_T c7_i28;
  real_T c7_c_lcam[2];
  int32_T c7_i29;
  real32_T c7_c_x[8];
  int32_T c7_i30;
  real32_T c7_c_y[8];
  int32_T c7_i31;
  int32_T c7_i32;
  int32_T c7_i33;
  int32_T c7_i34;
  real32_T c7_c_b[8];
  int32_T c7_iidx[8];
  int32_T c7_i35;
  real_T c7_b_idxsort[8];
  int32_T c7_i36;
  int32_T c7_i37;
  int32_T c7_i38;
  real32_T c7_b_bearing[8];
  int32_T c7_i39;
  int32_T c7_i40;
  int32_T c7_i41;
  int32_T c7_i42;
  int32_T c7_i43;
  int32_T c7_i44;
  int32_T c7_i45;
  int32_T c7_i46;
  int32_T c7_i47;
  int32_T c7_i48;
  real_T *c7_b_robotTheta;
  real32_T (*c7_b_dist)[6];
  real32_T (*c7_b_bear)[6];
  real32_T (*c7_b_targetsPos)[16];
  real_T (*c7_b_robotPos)[2];
  c7_b_targetsPos = (real32_T (*)[16])ssGetInputPortSignal(chartInstance->S, 2);
  c7_b_bear = (real32_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 2);
  c7_b_dist = (real32_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 1);
  c7_b_robotTheta = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
  c7_b_robotPos = (real_T (*)[2])ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 3U, chartInstance->c7_sfEvent);
  for (c7_i6 = 0; c7_i6 < 2; c7_i6++) {
    _SFD_DATA_RANGE_CHECK((*c7_b_robotPos)[c7_i6], 0U);
  }

  _SFD_DATA_RANGE_CHECK(*c7_b_robotTheta, 1U);
  chartInstance->c7_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 3U, chartInstance->c7_sfEvent);
  c7_hoistedGlobal = *c7_b_robotTheta;
  for (c7_i7 = 0; c7_i7 < 2; c7_i7++) {
    c7_robotPos[c7_i7] = (*c7_b_robotPos)[c7_i7];
  }

  c7_robotTheta = c7_hoistedGlobal;
  for (c7_i8 = 0; c7_i8 < 16; c7_i8++) {
    c7_targetsPos[c7_i8] = (*c7_b_targetsPos)[c7_i8];
  }

  for (c7_i9 = 0; c7_i9 < 2; c7_i9++) {
    c7_b_pcam[c7_i9] = chartInstance->c7_pcam[c7_i9];
  }

  for (c7_i10 = 0; c7_i10 < 2; c7_i10++) {
    c7_b_lcam[c7_i10] = chartInstance->c7_lcam[c7_i10];
  }

  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 14U, 14U, c7_debug_family_names,
    c7_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c7_x, 0U, c7_f_sf_marshallOut,
    c7_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c7_y, 1U, c7_f_sf_marshallOut,
    c7_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c7_distance, 2U, c7_f_sf_marshallOut,
    c7_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c7_bearing, 3U, c7_f_sf_marshallOut,
    c7_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c7_idxsort, 4U, c7_e_sf_marshallOut,
    c7_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_nargin, 5U, c7_d_sf_marshallOut,
    c7_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_nargout, 6U, c7_d_sf_marshallOut,
    c7_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c7_robotPos, 7U, c7_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c7_robotTheta, 8U, c7_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c7_targetsPos, 9U, c7_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c7_b_pcam, 10U, c7_b_sf_marshallOut,
    c7_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c7_b_lcam, 11U, c7_b_sf_marshallOut,
    c7_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c7_dist, 12U, c7_sf_marshallOut,
    c7_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c7_bear, 13U, c7_sf_marshallOut,
    c7_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 30);
  c7_x_robot = c7_robotPos[0];
  c7_y_robot = c7_robotPos[1];
  c7_theta_robot = c7_robotTheta;
  for (c7_i11 = 0; c7_i11 < 8; c7_i11++) {
    c7_x_world[c7_i11] = c7_targetsPos[c7_i11];
  }

  for (c7_i12 = 0; c7_i12 < 8; c7_i12++) {
    c7_y_world[c7_i12] = c7_targetsPos[c7_i12 + 8];
  }

  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 11U, 11U, c7_b_debug_family_names,
    c7_b_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c7_x_dif, 0U, c7_f_sf_marshallOut,
    c7_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c7_y_dif, 1U, c7_f_sf_marshallOut,
    c7_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_b_nargin, 2U, c7_d_sf_marshallOut,
    c7_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_b_nargout, 3U, c7_d_sf_marshallOut,
    c7_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_x_robot, 4U, c7_d_sf_marshallOut,
    c7_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_y_robot, 5U, c7_d_sf_marshallOut,
    c7_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_theta_robot, 6U, c7_d_sf_marshallOut,
    c7_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c7_x_world, 7U, c7_f_sf_marshallOut,
    c7_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c7_y_world, 8U, c7_f_sf_marshallOut,
    c7_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c7_b_x, 9U, c7_f_sf_marshallOut,
    c7_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c7_b_y, 10U, c7_f_sf_marshallOut,
    c7_e_sf_marshallIn);
  CV_EML_FCN(0, 3);
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 133U);
  for (c7_i13 = 0; c7_i13 < 8; c7_i13++) {
    c7_x_dif[c7_i13] = c7_x_world[c7_i13] - (real32_T)c7_x_robot;
  }

  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 134U);
  for (c7_i14 = 0; c7_i14 < 8; c7_i14++) {
    c7_y_dif[c7_i14] = c7_y_world[c7_i14] - (real32_T)c7_y_robot;
  }

  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 139U);
  c7_a = c7_theta_robot;
  c7_b_cosd(chartInstance, &c7_a);
  for (c7_i15 = 0; c7_i15 < 8; c7_i15++) {
    c7_b[c7_i15] = c7_x_dif[c7_i15];
  }

  for (c7_i16 = 0; c7_i16 < 8; c7_i16++) {
    c7_b[c7_i16] *= (real32_T)c7_a;
  }

  c7_b_a = c7_theta_robot;
  c7_b_sind(chartInstance, &c7_b_a);
  for (c7_i17 = 0; c7_i17 < 8; c7_i17++) {
    c7_b_b[c7_i17] = c7_y_dif[c7_i17];
  }

  for (c7_i18 = 0; c7_i18 < 8; c7_i18++) {
    c7_b_b[c7_i18] *= (real32_T)c7_b_a;
  }

  for (c7_i19 = 0; c7_i19 < 8; c7_i19++) {
    c7_b_x[c7_i19] = c7_b[c7_i19] + c7_b_b[c7_i19];
  }

  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 140U);
  c7_c_a = c7_theta_robot;
  c7_b_cosd(chartInstance, &c7_c_a);
  for (c7_i20 = 0; c7_i20 < 8; c7_i20++) {
    c7_b[c7_i20] = c7_y_dif[c7_i20];
  }

  for (c7_i21 = 0; c7_i21 < 8; c7_i21++) {
    c7_b[c7_i21] *= (real32_T)c7_c_a;
  }

  c7_d_a = c7_theta_robot;
  c7_b_sind(chartInstance, &c7_d_a);
  for (c7_i22 = 0; c7_i22 < 8; c7_i22++) {
    c7_b_b[c7_i22] = c7_x_dif[c7_i22];
  }

  for (c7_i23 = 0; c7_i23 < 8; c7_i23++) {
    c7_b_b[c7_i23] *= (real32_T)c7_d_a;
  }

  for (c7_i24 = 0; c7_i24 < 8; c7_i24++) {
    c7_b_y[c7_i24] = c7_b[c7_i24] - c7_b_b[c7_i24];
  }

  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, -140);
  _SFD_SYMBOL_SCOPE_POP();
  for (c7_i25 = 0; c7_i25 < 8; c7_i25++) {
    c7_x[c7_i25] = c7_b_x[c7_i25];
  }

  for (c7_i26 = 0; c7_i26 < 8; c7_i26++) {
    c7_y[c7_i26] = c7_b_y[c7_i26];
  }

  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 33);
  for (c7_i27 = 0; c7_i27 < 2; c7_i27++) {
    c7_c_pcam[c7_i27] = c7_b_pcam[c7_i27];
  }

  for (c7_i28 = 0; c7_i28 < 2; c7_i28++) {
    c7_c_lcam[c7_i28] = c7_b_lcam[c7_i28];
  }

  for (c7_i29 = 0; c7_i29 < 8; c7_i29++) {
    c7_c_x[c7_i29] = c7_x[c7_i29];
  }

  for (c7_i30 = 0; c7_i30 < 8; c7_i30++) {
    c7_c_y[c7_i30] = c7_y[c7_i30];
  }

  c7_isincamera(chartInstance, c7_c_pcam, c7_c_lcam, c7_c_x, c7_c_y, c7_b,
                c7_b_b);
  for (c7_i31 = 0; c7_i31 < 8; c7_i31++) {
    c7_distance[c7_i31] = c7_b[c7_i31];
  }

  for (c7_i32 = 0; c7_i32 < 8; c7_i32++) {
    c7_bearing[c7_i32] = c7_b_b[c7_i32];
  }

  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 36);
  for (c7_i33 = 0; c7_i33 < 8; c7_i33++) {
    c7_b[c7_i33] = c7_distance[c7_i33];
  }

  for (c7_i34 = 0; c7_i34 < 8; c7_i34++) {
    c7_c_b[c7_i34] = c7_b[c7_i34];
  }

  c7_eml_sort(chartInstance, c7_c_b, c7_b, c7_iidx);
  for (c7_i35 = 0; c7_i35 < 8; c7_i35++) {
    c7_b_idxsort[c7_i35] = (real_T)c7_iidx[c7_i35];
  }

  for (c7_i36 = 0; c7_i36 < 8; c7_i36++) {
    c7_distance[c7_i36] = c7_b[c7_i36];
  }

  for (c7_i37 = 0; c7_i37 < 8; c7_i37++) {
    c7_idxsort[c7_i37] = c7_b_idxsort[c7_i37];
  }

  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 37);
  for (c7_i38 = 0; c7_i38 < 8; c7_i38++) {
    c7_b_bearing[c7_i38] = c7_bearing[_SFD_EML_ARRAY_BOUNDS_CHECK("bearing",
      (int32_T)c7_idxsort[c7_i38], 1, 8, 1, 0) - 1];
  }

  for (c7_i39 = 0; c7_i39 < 8; c7_i39++) {
    c7_bearing[c7_i39] = c7_b_bearing[c7_i39];
  }

  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 38);
  CV_EML_IF(0, 1, 0, false);
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 42);
  for (c7_i40 = 0; c7_i40 < 6; c7_i40++) {
    c7_dist[c7_i40] = c7_distance[c7_i40];
  }

  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 43);
  for (c7_i41 = 0; c7_i41 < 6; c7_i41++) {
    c7_bear[c7_i41] = c7_bearing[c7_i41];
  }

  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, -43);
  _SFD_SYMBOL_SCOPE_POP();
  for (c7_i42 = 0; c7_i42 < 6; c7_i42++) {
    (*c7_b_dist)[c7_i42] = c7_dist[c7_i42];
  }

  for (c7_i43 = 0; c7_i43 < 6; c7_i43++) {
    (*c7_b_bear)[c7_i43] = c7_bear[c7_i43];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 3U, chartInstance->c7_sfEvent);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_SimulationModelMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
  for (c7_i44 = 0; c7_i44 < 6; c7_i44++) {
    _SFD_DATA_RANGE_CHECK((real_T)(*c7_b_dist)[c7_i44], 2U);
  }

  for (c7_i45 = 0; c7_i45 < 6; c7_i45++) {
    _SFD_DATA_RANGE_CHECK((real_T)(*c7_b_bear)[c7_i45], 3U);
  }

  for (c7_i46 = 0; c7_i46 < 16; c7_i46++) {
    _SFD_DATA_RANGE_CHECK((real_T)(*c7_b_targetsPos)[c7_i46], 4U);
  }

  for (c7_i47 = 0; c7_i47 < 2; c7_i47++) {
    _SFD_DATA_RANGE_CHECK(chartInstance->c7_pcam[c7_i47], 5U);
  }

  for (c7_i48 = 0; c7_i48 < 2; c7_i48++) {
    _SFD_DATA_RANGE_CHECK(chartInstance->c7_lcam[c7_i48], 6U);
  }
}

static void initSimStructsc7_SimulationModel(SFc7_SimulationModelInstanceStruct *
  chartInstance)
{
  (void)chartInstance;
}

static void init_script_number_translation(uint32_T c7_machineNumber, uint32_T
  c7_chartNumber, uint32_T c7_instanceNumber)
{
  (void)c7_machineNumber;
  (void)c7_chartNumber;
  (void)c7_instanceNumber;
}

static const mxArray *c7_sf_marshallOut(void *chartInstanceVoid, void *c7_inData)
{
  const mxArray *c7_mxArrayOutData = NULL;
  int32_T c7_i49;
  real32_T c7_b_inData[6];
  int32_T c7_i50;
  real32_T c7_u[6];
  const mxArray *c7_y = NULL;
  SFc7_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc7_SimulationModelInstanceStruct *)chartInstanceVoid;
  c7_mxArrayOutData = NULL;
  for (c7_i49 = 0; c7_i49 < 6; c7_i49++) {
    c7_b_inData[c7_i49] = (*(real32_T (*)[6])c7_inData)[c7_i49];
  }

  for (c7_i50 = 0; c7_i50 < 6; c7_i50++) {
    c7_u[c7_i50] = c7_b_inData[c7_i50];
  }

  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_create("y", c7_u, 1, 0U, 1U, 0U, 1, 6), false);
  sf_mex_assign(&c7_mxArrayOutData, c7_y, false);
  return c7_mxArrayOutData;
}

static void c7_emlrt_marshallIn(SFc7_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c7_bear, const char_T *c7_identifier, real32_T
  c7_y[6])
{
  emlrtMsgIdentifier c7_thisId;
  c7_thisId.fIdentifier = c7_identifier;
  c7_thisId.fParent = NULL;
  c7_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c7_bear), &c7_thisId, c7_y);
  sf_mex_destroy(&c7_bear);
}

static void c7_b_emlrt_marshallIn(SFc7_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId,
  real32_T c7_y[6])
{
  real32_T c7_fv2[6];
  int32_T c7_i51;
  (void)chartInstance;
  sf_mex_import(c7_parentId, sf_mex_dup(c7_u), c7_fv2, 1, 1, 0U, 1, 0U, 1, 6);
  for (c7_i51 = 0; c7_i51 < 6; c7_i51++) {
    c7_y[c7_i51] = c7_fv2[c7_i51];
  }

  sf_mex_destroy(&c7_u);
}

static void c7_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData)
{
  const mxArray *c7_bear;
  const char_T *c7_identifier;
  emlrtMsgIdentifier c7_thisId;
  real32_T c7_y[6];
  int32_T c7_i52;
  SFc7_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc7_SimulationModelInstanceStruct *)chartInstanceVoid;
  c7_bear = sf_mex_dup(c7_mxArrayInData);
  c7_identifier = c7_varName;
  c7_thisId.fIdentifier = c7_identifier;
  c7_thisId.fParent = NULL;
  c7_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c7_bear), &c7_thisId, c7_y);
  sf_mex_destroy(&c7_bear);
  for (c7_i52 = 0; c7_i52 < 6; c7_i52++) {
    (*(real32_T (*)[6])c7_outData)[c7_i52] = c7_y[c7_i52];
  }

  sf_mex_destroy(&c7_mxArrayInData);
}

static const mxArray *c7_b_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData)
{
  const mxArray *c7_mxArrayOutData = NULL;
  int32_T c7_i53;
  real_T c7_b_inData[2];
  int32_T c7_i54;
  real_T c7_u[2];
  const mxArray *c7_y = NULL;
  SFc7_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc7_SimulationModelInstanceStruct *)chartInstanceVoid;
  c7_mxArrayOutData = NULL;
  for (c7_i53 = 0; c7_i53 < 2; c7_i53++) {
    c7_b_inData[c7_i53] = (*(real_T (*)[2])c7_inData)[c7_i53];
  }

  for (c7_i54 = 0; c7_i54 < 2; c7_i54++) {
    c7_u[c7_i54] = c7_b_inData[c7_i54];
  }

  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_create("y", c7_u, 0, 0U, 1U, 0U, 2, 1, 2), false);
  sf_mex_assign(&c7_mxArrayOutData, c7_y, false);
  return c7_mxArrayOutData;
}

static void c7_c_emlrt_marshallIn(SFc7_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId,
  real_T c7_y[2])
{
  real_T c7_dv2[2];
  int32_T c7_i55;
  (void)chartInstance;
  sf_mex_import(c7_parentId, sf_mex_dup(c7_u), c7_dv2, 1, 0, 0U, 1, 0U, 2, 1, 2);
  for (c7_i55 = 0; c7_i55 < 2; c7_i55++) {
    c7_y[c7_i55] = c7_dv2[c7_i55];
  }

  sf_mex_destroy(&c7_u);
}

static void c7_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData)
{
  const mxArray *c7_b_lcam;
  const char_T *c7_identifier;
  emlrtMsgIdentifier c7_thisId;
  real_T c7_y[2];
  int32_T c7_i56;
  SFc7_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc7_SimulationModelInstanceStruct *)chartInstanceVoid;
  c7_b_lcam = sf_mex_dup(c7_mxArrayInData);
  c7_identifier = c7_varName;
  c7_thisId.fIdentifier = c7_identifier;
  c7_thisId.fParent = NULL;
  c7_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c7_b_lcam), &c7_thisId, c7_y);
  sf_mex_destroy(&c7_b_lcam);
  for (c7_i56 = 0; c7_i56 < 2; c7_i56++) {
    (*(real_T (*)[2])c7_outData)[c7_i56] = c7_y[c7_i56];
  }

  sf_mex_destroy(&c7_mxArrayInData);
}

static const mxArray *c7_c_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData)
{
  const mxArray *c7_mxArrayOutData = NULL;
  int32_T c7_i57;
  int32_T c7_i58;
  int32_T c7_i59;
  real32_T c7_b_inData[16];
  int32_T c7_i60;
  int32_T c7_i61;
  int32_T c7_i62;
  real32_T c7_u[16];
  const mxArray *c7_y = NULL;
  SFc7_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc7_SimulationModelInstanceStruct *)chartInstanceVoid;
  c7_mxArrayOutData = NULL;
  c7_i57 = 0;
  for (c7_i58 = 0; c7_i58 < 2; c7_i58++) {
    for (c7_i59 = 0; c7_i59 < 8; c7_i59++) {
      c7_b_inData[c7_i59 + c7_i57] = (*(real32_T (*)[16])c7_inData)[c7_i59 +
        c7_i57];
    }

    c7_i57 += 8;
  }

  c7_i60 = 0;
  for (c7_i61 = 0; c7_i61 < 2; c7_i61++) {
    for (c7_i62 = 0; c7_i62 < 8; c7_i62++) {
      c7_u[c7_i62 + c7_i60] = c7_b_inData[c7_i62 + c7_i60];
    }

    c7_i60 += 8;
  }

  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_create("y", c7_u, 1, 0U, 1U, 0U, 2, 8, 2), false);
  sf_mex_assign(&c7_mxArrayOutData, c7_y, false);
  return c7_mxArrayOutData;
}

static const mxArray *c7_d_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData)
{
  const mxArray *c7_mxArrayOutData = NULL;
  real_T c7_u;
  const mxArray *c7_y = NULL;
  SFc7_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc7_SimulationModelInstanceStruct *)chartInstanceVoid;
  c7_mxArrayOutData = NULL;
  c7_u = *(real_T *)c7_inData;
  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_create("y", &c7_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c7_mxArrayOutData, c7_y, false);
  return c7_mxArrayOutData;
}

static real_T c7_d_emlrt_marshallIn(SFc7_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId)
{
  real_T c7_y;
  real_T c7_d0;
  (void)chartInstance;
  sf_mex_import(c7_parentId, sf_mex_dup(c7_u), &c7_d0, 1, 0, 0U, 0, 0U, 0);
  c7_y = c7_d0;
  sf_mex_destroy(&c7_u);
  return c7_y;
}

static void c7_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData)
{
  const mxArray *c7_nargout;
  const char_T *c7_identifier;
  emlrtMsgIdentifier c7_thisId;
  real_T c7_y;
  SFc7_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc7_SimulationModelInstanceStruct *)chartInstanceVoid;
  c7_nargout = sf_mex_dup(c7_mxArrayInData);
  c7_identifier = c7_varName;
  c7_thisId.fIdentifier = c7_identifier;
  c7_thisId.fParent = NULL;
  c7_y = c7_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c7_nargout), &c7_thisId);
  sf_mex_destroy(&c7_nargout);
  *(real_T *)c7_outData = c7_y;
  sf_mex_destroy(&c7_mxArrayInData);
}

static const mxArray *c7_e_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData)
{
  const mxArray *c7_mxArrayOutData = NULL;
  int32_T c7_i63;
  real_T c7_b_inData[8];
  int32_T c7_i64;
  real_T c7_u[8];
  const mxArray *c7_y = NULL;
  SFc7_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc7_SimulationModelInstanceStruct *)chartInstanceVoid;
  c7_mxArrayOutData = NULL;
  for (c7_i63 = 0; c7_i63 < 8; c7_i63++) {
    c7_b_inData[c7_i63] = (*(real_T (*)[8])c7_inData)[c7_i63];
  }

  for (c7_i64 = 0; c7_i64 < 8; c7_i64++) {
    c7_u[c7_i64] = c7_b_inData[c7_i64];
  }

  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_create("y", c7_u, 0, 0U, 1U, 0U, 1, 8), false);
  sf_mex_assign(&c7_mxArrayOutData, c7_y, false);
  return c7_mxArrayOutData;
}

static void c7_e_emlrt_marshallIn(SFc7_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId,
  real_T c7_y[8])
{
  real_T c7_dv3[8];
  int32_T c7_i65;
  (void)chartInstance;
  sf_mex_import(c7_parentId, sf_mex_dup(c7_u), c7_dv3, 1, 0, 0U, 1, 0U, 1, 8);
  for (c7_i65 = 0; c7_i65 < 8; c7_i65++) {
    c7_y[c7_i65] = c7_dv3[c7_i65];
  }

  sf_mex_destroy(&c7_u);
}

static void c7_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData)
{
  const mxArray *c7_idxsort;
  const char_T *c7_identifier;
  emlrtMsgIdentifier c7_thisId;
  real_T c7_y[8];
  int32_T c7_i66;
  SFc7_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc7_SimulationModelInstanceStruct *)chartInstanceVoid;
  c7_idxsort = sf_mex_dup(c7_mxArrayInData);
  c7_identifier = c7_varName;
  c7_thisId.fIdentifier = c7_identifier;
  c7_thisId.fParent = NULL;
  c7_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c7_idxsort), &c7_thisId, c7_y);
  sf_mex_destroy(&c7_idxsort);
  for (c7_i66 = 0; c7_i66 < 8; c7_i66++) {
    (*(real_T (*)[8])c7_outData)[c7_i66] = c7_y[c7_i66];
  }

  sf_mex_destroy(&c7_mxArrayInData);
}

static const mxArray *c7_f_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData)
{
  const mxArray *c7_mxArrayOutData = NULL;
  int32_T c7_i67;
  real32_T c7_b_inData[8];
  int32_T c7_i68;
  real32_T c7_u[8];
  const mxArray *c7_y = NULL;
  SFc7_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc7_SimulationModelInstanceStruct *)chartInstanceVoid;
  c7_mxArrayOutData = NULL;
  for (c7_i67 = 0; c7_i67 < 8; c7_i67++) {
    c7_b_inData[c7_i67] = (*(real32_T (*)[8])c7_inData)[c7_i67];
  }

  for (c7_i68 = 0; c7_i68 < 8; c7_i68++) {
    c7_u[c7_i68] = c7_b_inData[c7_i68];
  }

  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_create("y", c7_u, 1, 0U, 1U, 0U, 1, 8), false);
  sf_mex_assign(&c7_mxArrayOutData, c7_y, false);
  return c7_mxArrayOutData;
}

static void c7_f_emlrt_marshallIn(SFc7_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId,
  real32_T c7_y[8])
{
  real32_T c7_fv3[8];
  int32_T c7_i69;
  (void)chartInstance;
  sf_mex_import(c7_parentId, sf_mex_dup(c7_u), c7_fv3, 1, 1, 0U, 1, 0U, 1, 8);
  for (c7_i69 = 0; c7_i69 < 8; c7_i69++) {
    c7_y[c7_i69] = c7_fv3[c7_i69];
  }

  sf_mex_destroy(&c7_u);
}

static void c7_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData)
{
  const mxArray *c7_bearing;
  const char_T *c7_identifier;
  emlrtMsgIdentifier c7_thisId;
  real32_T c7_y[8];
  int32_T c7_i70;
  SFc7_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc7_SimulationModelInstanceStruct *)chartInstanceVoid;
  c7_bearing = sf_mex_dup(c7_mxArrayInData);
  c7_identifier = c7_varName;
  c7_thisId.fIdentifier = c7_identifier;
  c7_thisId.fParent = NULL;
  c7_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c7_bearing), &c7_thisId, c7_y);
  sf_mex_destroy(&c7_bearing);
  for (c7_i70 = 0; c7_i70 < 8; c7_i70++) {
    (*(real32_T (*)[8])c7_outData)[c7_i70] = c7_y[c7_i70];
  }

  sf_mex_destroy(&c7_mxArrayInData);
}

static const mxArray *c7_g_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData)
{
  const mxArray *c7_mxArrayOutData = NULL;
  int32_T c7_i71;
  boolean_T c7_b_inData[8];
  int32_T c7_i72;
  boolean_T c7_u[8];
  const mxArray *c7_y = NULL;
  SFc7_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc7_SimulationModelInstanceStruct *)chartInstanceVoid;
  c7_mxArrayOutData = NULL;
  for (c7_i71 = 0; c7_i71 < 8; c7_i71++) {
    c7_b_inData[c7_i71] = (*(boolean_T (*)[8])c7_inData)[c7_i71];
  }

  for (c7_i72 = 0; c7_i72 < 8; c7_i72++) {
    c7_u[c7_i72] = c7_b_inData[c7_i72];
  }

  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_create("y", c7_u, 11, 0U, 1U, 0U, 1, 8), false);
  sf_mex_assign(&c7_mxArrayOutData, c7_y, false);
  return c7_mxArrayOutData;
}

static void c7_g_emlrt_marshallIn(SFc7_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId,
  boolean_T c7_y[8])
{
  boolean_T c7_bv0[8];
  int32_T c7_i73;
  (void)chartInstance;
  sf_mex_import(c7_parentId, sf_mex_dup(c7_u), c7_bv0, 1, 11, 0U, 1, 0U, 1, 8);
  for (c7_i73 = 0; c7_i73 < 8; c7_i73++) {
    c7_y[c7_i73] = c7_bv0[c7_i73];
  }

  sf_mex_destroy(&c7_u);
}

static void c7_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData)
{
  const mxArray *c7_isvisible;
  const char_T *c7_identifier;
  emlrtMsgIdentifier c7_thisId;
  boolean_T c7_y[8];
  int32_T c7_i74;
  SFc7_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc7_SimulationModelInstanceStruct *)chartInstanceVoid;
  c7_isvisible = sf_mex_dup(c7_mxArrayInData);
  c7_identifier = c7_varName;
  c7_thisId.fIdentifier = c7_identifier;
  c7_thisId.fParent = NULL;
  c7_g_emlrt_marshallIn(chartInstance, sf_mex_dup(c7_isvisible), &c7_thisId,
                        c7_y);
  sf_mex_destroy(&c7_isvisible);
  for (c7_i74 = 0; c7_i74 < 8; c7_i74++) {
    (*(boolean_T (*)[8])c7_outData)[c7_i74] = c7_y[c7_i74];
  }

  sf_mex_destroy(&c7_mxArrayInData);
}

const mxArray *sf_c7_SimulationModel_get_eml_resolved_functions_info(void)
{
  const mxArray *c7_nameCaptureInfo = NULL;
  c7_nameCaptureInfo = NULL;
  sf_mex_assign(&c7_nameCaptureInfo, sf_mex_createstruct("structure", 2, 276, 1),
                false);
  c7_info_helper(&c7_nameCaptureInfo);
  c7_b_info_helper(&c7_nameCaptureInfo);
  c7_c_info_helper(&c7_nameCaptureInfo);
  c7_d_info_helper(&c7_nameCaptureInfo);
  c7_e_info_helper(&c7_nameCaptureInfo);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c7_nameCaptureInfo);
  return c7_nameCaptureInfo;
}

static void c7_info_helper(const mxArray **c7_info)
{
  const mxArray *c7_rhs0 = NULL;
  const mxArray *c7_lhs0 = NULL;
  const mxArray *c7_rhs1 = NULL;
  const mxArray *c7_lhs1 = NULL;
  const mxArray *c7_rhs2 = NULL;
  const mxArray *c7_lhs2 = NULL;
  const mxArray *c7_rhs3 = NULL;
  const mxArray *c7_lhs3 = NULL;
  const mxArray *c7_rhs4 = NULL;
  const mxArray *c7_lhs4 = NULL;
  const mxArray *c7_rhs5 = NULL;
  const mxArray *c7_lhs5 = NULL;
  const mxArray *c7_rhs6 = NULL;
  const mxArray *c7_lhs6 = NULL;
  const mxArray *c7_rhs7 = NULL;
  const mxArray *c7_lhs7 = NULL;
  const mxArray *c7_rhs8 = NULL;
  const mxArray *c7_lhs8 = NULL;
  const mxArray *c7_rhs9 = NULL;
  const mxArray *c7_lhs9 = NULL;
  const mxArray *c7_rhs10 = NULL;
  const mxArray *c7_lhs10 = NULL;
  const mxArray *c7_rhs11 = NULL;
  const mxArray *c7_lhs11 = NULL;
  const mxArray *c7_rhs12 = NULL;
  const mxArray *c7_lhs12 = NULL;
  const mxArray *c7_rhs13 = NULL;
  const mxArray *c7_lhs13 = NULL;
  const mxArray *c7_rhs14 = NULL;
  const mxArray *c7_lhs14 = NULL;
  const mxArray *c7_rhs15 = NULL;
  const mxArray *c7_lhs15 = NULL;
  const mxArray *c7_rhs16 = NULL;
  const mxArray *c7_lhs16 = NULL;
  const mxArray *c7_rhs17 = NULL;
  const mxArray *c7_lhs17 = NULL;
  const mxArray *c7_rhs18 = NULL;
  const mxArray *c7_lhs18 = NULL;
  const mxArray *c7_rhs19 = NULL;
  const mxArray *c7_lhs19 = NULL;
  const mxArray *c7_rhs20 = NULL;
  const mxArray *c7_lhs20 = NULL;
  const mxArray *c7_rhs21 = NULL;
  const mxArray *c7_lhs21 = NULL;
  const mxArray *c7_rhs22 = NULL;
  const mxArray *c7_lhs22 = NULL;
  const mxArray *c7_rhs23 = NULL;
  const mxArray *c7_lhs23 = NULL;
  const mxArray *c7_rhs24 = NULL;
  const mxArray *c7_lhs24 = NULL;
  const mxArray *c7_rhs25 = NULL;
  const mxArray *c7_lhs25 = NULL;
  const mxArray *c7_rhs26 = NULL;
  const mxArray *c7_lhs26 = NULL;
  const mxArray *c7_rhs27 = NULL;
  const mxArray *c7_lhs27 = NULL;
  const mxArray *c7_rhs28 = NULL;
  const mxArray *c7_lhs28 = NULL;
  const mxArray *c7_rhs29 = NULL;
  const mxArray *c7_lhs29 = NULL;
  const mxArray *c7_rhs30 = NULL;
  const mxArray *c7_lhs30 = NULL;
  const mxArray *c7_rhs31 = NULL;
  const mxArray *c7_lhs31 = NULL;
  const mxArray *c7_rhs32 = NULL;
  const mxArray *c7_lhs32 = NULL;
  const mxArray *c7_rhs33 = NULL;
  const mxArray *c7_lhs33 = NULL;
  const mxArray *c7_rhs34 = NULL;
  const mxArray *c7_lhs34 = NULL;
  const mxArray *c7_rhs35 = NULL;
  const mxArray *c7_lhs35 = NULL;
  const mxArray *c7_rhs36 = NULL;
  const mxArray *c7_lhs36 = NULL;
  const mxArray *c7_rhs37 = NULL;
  const mxArray *c7_lhs37 = NULL;
  const mxArray *c7_rhs38 = NULL;
  const mxArray *c7_lhs38 = NULL;
  const mxArray *c7_rhs39 = NULL;
  const mxArray *c7_lhs39 = NULL;
  const mxArray *c7_rhs40 = NULL;
  const mxArray *c7_lhs40 = NULL;
  const mxArray *c7_rhs41 = NULL;
  const mxArray *c7_lhs41 = NULL;
  const mxArray *c7_rhs42 = NULL;
  const mxArray *c7_lhs42 = NULL;
  const mxArray *c7_rhs43 = NULL;
  const mxArray *c7_lhs43 = NULL;
  const mxArray *c7_rhs44 = NULL;
  const mxArray *c7_lhs44 = NULL;
  const mxArray *c7_rhs45 = NULL;
  const mxArray *c7_lhs45 = NULL;
  const mxArray *c7_rhs46 = NULL;
  const mxArray *c7_lhs46 = NULL;
  const mxArray *c7_rhs47 = NULL;
  const mxArray *c7_lhs47 = NULL;
  const mxArray *c7_rhs48 = NULL;
  const mxArray *c7_lhs48 = NULL;
  const mxArray *c7_rhs49 = NULL;
  const mxArray *c7_lhs49 = NULL;
  const mxArray *c7_rhs50 = NULL;
  const mxArray *c7_lhs50 = NULL;
  const mxArray *c7_rhs51 = NULL;
  const mxArray *c7_lhs51 = NULL;
  const mxArray *c7_rhs52 = NULL;
  const mxArray *c7_lhs52 = NULL;
  const mxArray *c7_rhs53 = NULL;
  const mxArray *c7_lhs53 = NULL;
  const mxArray *c7_rhs54 = NULL;
  const mxArray *c7_lhs54 = NULL;
  const mxArray *c7_rhs55 = NULL;
  const mxArray *c7_lhs55 = NULL;
  const mxArray *c7_rhs56 = NULL;
  const mxArray *c7_lhs56 = NULL;
  const mxArray *c7_rhs57 = NULL;
  const mxArray *c7_lhs57 = NULL;
  const mxArray *c7_rhs58 = NULL;
  const mxArray *c7_lhs58 = NULL;
  const mxArray *c7_rhs59 = NULL;
  const mxArray *c7_lhs59 = NULL;
  const mxArray *c7_rhs60 = NULL;
  const mxArray *c7_lhs60 = NULL;
  const mxArray *c7_rhs61 = NULL;
  const mxArray *c7_lhs61 = NULL;
  const mxArray *c7_rhs62 = NULL;
  const mxArray *c7_lhs62 = NULL;
  const mxArray *c7_rhs63 = NULL;
  const mxArray *c7_lhs63 = NULL;
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "context", "context", 0);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("cosd"), "name", "name", 0);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 0);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cosd.m"), "resolved",
                  "resolved", 0);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1343830372U), "fileTimeLo",
                  "fileTimeLo", 0);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 0);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 0);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 0);
  sf_mex_assign(&c7_rhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs0), "rhs", "rhs", 0);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs0), "lhs", "lhs", 0);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cosd.m"), "context",
                  "context", 1);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_scalar_cosd_and_sind"),
                  "name", "name", 1);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 1);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_cosd_and_sind.m"),
                  "resolved", "resolved", 1);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1343830378U), "fileTimeLo",
                  "fileTimeLo", 1);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 1);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 1);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 1);
  sf_mex_assign(&c7_rhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs1), "rhs", "rhs", 1);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs1), "lhs", "lhs", 1);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_cosd_and_sind.m"),
                  "context", "context", 2);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("isfinite"), "name", "name", 2);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 2);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m"), "resolved",
                  "resolved", 2);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363713856U), "fileTimeLo",
                  "fileTimeLo", 2);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 2);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 2);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 2);
  sf_mex_assign(&c7_rhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs2), "rhs", "rhs", 2);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs2), "lhs", "lhs", 2);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m"), "context",
                  "context", 3);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 3);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 3);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 3);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 3);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 3);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 3);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 3);
  sf_mex_assign(&c7_rhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs3), "rhs", "rhs", 3);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs3), "lhs", "lhs", 3);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m"), "context",
                  "context", 4);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("isinf"), "name", "name", 4);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 4);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isinf.m"), "resolved",
                  "resolved", 4);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363713856U), "fileTimeLo",
                  "fileTimeLo", 4);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 4);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 4);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 4);
  sf_mex_assign(&c7_rhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs4), "rhs", "rhs", 4);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs4), "lhs", "lhs", 4);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isinf.m"), "context",
                  "context", 5);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 5);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 5);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 5);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 5);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 5);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 5);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 5);
  sf_mex_assign(&c7_rhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs5), "rhs", "rhs", 5);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs5), "lhs", "lhs", 5);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m"), "context",
                  "context", 6);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("isnan"), "name", "name", 6);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 6);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m"), "resolved",
                  "resolved", 6);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363713858U), "fileTimeLo",
                  "fileTimeLo", 6);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 6);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 6);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 6);
  sf_mex_assign(&c7_rhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs6), "rhs", "rhs", 6);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs6), "lhs", "lhs", 6);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m"), "context",
                  "context", 7);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 7);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 7);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 7);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 7);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 7);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 7);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 7);
  sf_mex_assign(&c7_rhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs7), "rhs", "rhs", 7);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs7), "lhs", "lhs", 7);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_cosd_and_sind.m"),
                  "context", "context", 8);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_guarded_nan"), "name",
                  "name", 8);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 8);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_guarded_nan.m"),
                  "resolved", "resolved", 8);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1286818776U), "fileTimeLo",
                  "fileTimeLo", 8);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 8);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 8);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 8);
  sf_mex_assign(&c7_rhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs8), "rhs", "rhs", 8);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs8), "lhs", "lhs", 8);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_guarded_nan.m"),
                  "context", "context", 9);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_is_float_class"), "name",
                  "name", 9);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 9);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_float_class.m"),
                  "resolved", "resolved", 9);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1286818782U), "fileTimeLo",
                  "fileTimeLo", 9);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 9);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 9);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 9);
  sf_mex_assign(&c7_rhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs9), "rhs", "rhs", 9);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs9), "lhs", "lhs", 9);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_cosd_and_sind.m"),
                  "context", "context", 10);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_scalar_rem90"), "name",
                  "name", 10);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 10);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_rem90.m"),
                  "resolved", "resolved", 10);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1343830378U), "fileTimeLo",
                  "fileTimeLo", 10);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 10);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 10);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 10);
  sf_mex_assign(&c7_rhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs10), "rhs", "rhs",
                  10);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs10), "lhs", "lhs",
                  10);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_rem90.m"),
                  "context", "context", 11);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("mrdivide"), "name", "name", 11);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 11);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "resolved",
                  "resolved", 11);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1388460096U), "fileTimeLo",
                  "fileTimeLo", 11);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 11);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1370009886U), "mFileTimeLo",
                  "mFileTimeLo", 11);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 11);
  sf_mex_assign(&c7_rhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs11), "rhs", "rhs",
                  11);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs11), "lhs", "lhs",
                  11);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 12);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.assert"),
                  "name", "name", 12);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 12);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assert.m"),
                  "resolved", "resolved", 12);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 12);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 12);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 12);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 12);
  sf_mex_assign(&c7_rhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs12), "rhs", "rhs",
                  12);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs12), "lhs", "lhs",
                  12);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 13);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("rdivide"), "name", "name", 13);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 13);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "resolved",
                  "resolved", 13);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363713880U), "fileTimeLo",
                  "fileTimeLo", 13);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 13);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 13);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 13);
  sf_mex_assign(&c7_rhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs13), "rhs", "rhs",
                  13);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs13), "lhs", "lhs",
                  13);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 14);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 14);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 14);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 14);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 14);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 14);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 14);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 14);
  sf_mex_assign(&c7_rhs14, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs14, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs14), "rhs", "rhs",
                  14);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs14), "lhs", "lhs",
                  14);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 15);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_scalexp_compatible"),
                  "name", "name", 15);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 15);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m"),
                  "resolved", "resolved", 15);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1286818796U), "fileTimeLo",
                  "fileTimeLo", 15);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 15);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 15);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 15);
  sf_mex_assign(&c7_rhs15, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs15, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs15), "rhs", "rhs",
                  15);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs15), "lhs", "lhs",
                  15);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 16);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_div"), "name", "name", 16);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 16);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "resolved",
                  "resolved", 16);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 16);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 16);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 16);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 16);
  sf_mex_assign(&c7_rhs16, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs16, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs16), "rhs", "rhs",
                  16);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs16), "lhs", "lhs",
                  16);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "context",
                  "context", 17);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.div"), "name",
                  "name", 17);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 17);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/div.p"), "resolved",
                  "resolved", 17);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1389307920U), "fileTimeLo",
                  "fileTimeLo", 17);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 17);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 17);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 17);
  sf_mex_assign(&c7_rhs17, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs17, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs17), "rhs", "rhs",
                  17);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs17), "lhs", "lhs",
                  17);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_rem90.m"),
                  "context", "context", 18);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("rem"), "name", "name", 18);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 18);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/rem.m"), "resolved",
                  "resolved", 18);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363713854U), "fileTimeLo",
                  "fileTimeLo", 18);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 18);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 18);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 18);
  sf_mex_assign(&c7_rhs18, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs18, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs18), "rhs", "rhs",
                  18);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs18), "lhs", "lhs",
                  18);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/rem.m"), "context",
                  "context", 19);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 19);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 19);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 19);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 19);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 19);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 19);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 19);
  sf_mex_assign(&c7_rhs19, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs19, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs19), "rhs", "rhs",
                  19);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs19), "lhs", "lhs",
                  19);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/rem.m"), "context",
                  "context", 20);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 20);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 20);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 20);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 20);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 20);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 20);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 20);
  sf_mex_assign(&c7_rhs20, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs20, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs20), "rhs", "rhs",
                  20);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs20), "lhs", "lhs",
                  20);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "context",
                  "context", 21);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 21);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 21);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 21);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1389307920U), "fileTimeLo",
                  "fileTimeLo", 21);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 21);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 21);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 21);
  sf_mex_assign(&c7_rhs21, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs21, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs21), "rhs", "rhs",
                  21);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs21), "lhs", "lhs",
                  21);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/rem.m"), "context",
                  "context", 22);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_scalexp_alloc"), "name",
                  "name", 22);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 22);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "resolved", "resolved", 22);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 22);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 22);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 22);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 22);
  sf_mex_assign(&c7_rhs22, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs22, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs22), "rhs", "rhs",
                  22);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs22), "lhs", "lhs",
                  22);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "context", "context", 23);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.scalexpAlloc"),
                  "name", "name", 23);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 23);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalexpAlloc.p"),
                  "resolved", "resolved", 23);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1389307920U), "fileTimeLo",
                  "fileTimeLo", 23);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 23);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 23);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 23);
  sf_mex_assign(&c7_rhs23, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs23, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs23), "rhs", "rhs",
                  23);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs23), "lhs", "lhs",
                  23);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_rem90.m"),
                  "context", "context", 24);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("abs"), "name", "name", 24);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 24);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "resolved",
                  "resolved", 24);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363713852U), "fileTimeLo",
                  "fileTimeLo", 24);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 24);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 24);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 24);
  sf_mex_assign(&c7_rhs24, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs24, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs24), "rhs", "rhs",
                  24);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs24), "lhs", "lhs",
                  24);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 25);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 25);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 25);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 25);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 25);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 25);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 25);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 25);
  sf_mex_assign(&c7_rhs25, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs25, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs25), "rhs", "rhs",
                  25);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs25), "lhs", "lhs",
                  25);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 26);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_scalar_abs"), "name",
                  "name", 26);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 26);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m"),
                  "resolved", "resolved", 26);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1286818712U), "fileTimeLo",
                  "fileTimeLo", 26);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 26);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 26);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 26);
  sf_mex_assign(&c7_rhs26, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs26, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs26), "rhs", "rhs",
                  26);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs26), "lhs", "lhs",
                  26);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "context", "context", 27);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_mtimes_helper"), "name",
                  "name", 27);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 27);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "resolved", "resolved", 27);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1383877294U), "fileTimeLo",
                  "fileTimeLo", 27);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 27);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 27);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 27);
  sf_mex_assign(&c7_rhs27, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs27, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs27), "rhs", "rhs",
                  27);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs27), "lhs", "lhs",
                  27);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m!common_checks"),
                  "context", "context", 28);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 28);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 28);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 28);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 28);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 28);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 28);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 28);
  sf_mex_assign(&c7_rhs28, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs28, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs28), "rhs", "rhs",
                  28);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs28), "lhs", "lhs",
                  28);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m!common_checks"),
                  "context", "context", 29);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 29);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 29);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 29);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 29);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 29);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 29);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 29);
  sf_mex_assign(&c7_rhs29, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs29, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs29), "rhs", "rhs",
                  29);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs29), "lhs", "lhs",
                  29);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "context", "context", 30);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("sind"), "name", "name", 30);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 30);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sind.m"), "resolved",
                  "resolved", 30);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1343830386U), "fileTimeLo",
                  "fileTimeLo", 30);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 30);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 30);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 30);
  sf_mex_assign(&c7_rhs30, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs30, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs30), "rhs", "rhs",
                  30);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs30), "lhs", "lhs",
                  30);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sind.m"), "context",
                  "context", 31);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_scalar_cosd_and_sind"),
                  "name", "name", 31);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 31);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_cosd_and_sind.m"),
                  "resolved", "resolved", 31);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1343830378U), "fileTimeLo",
                  "fileTimeLo", 31);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 31);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 31);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 31);
  sf_mex_assign(&c7_rhs31, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs31, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs31), "rhs", "rhs",
                  31);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs31), "lhs", "lhs",
                  31);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "context", "context", 32);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("mrdivide"), "name", "name", 32);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 32);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "resolved",
                  "resolved", 32);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1388460096U), "fileTimeLo",
                  "fileTimeLo", 32);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 32);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1370009886U), "mFileTimeLo",
                  "mFileTimeLo", 32);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 32);
  sf_mex_assign(&c7_rhs32, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs32, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs32), "rhs", "rhs",
                  32);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs32), "lhs", "lhs",
                  32);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "context", "context", 33);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("polyfit"), "name", "name", 33);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 33);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/polyfun/polyfit.m"), "resolved",
                  "resolved", 33);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363713878U), "fileTimeLo",
                  "fileTimeLo", 33);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 33);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 33);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 33);
  sf_mex_assign(&c7_rhs33, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs33, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs33), "rhs", "rhs",
                  33);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs33), "lhs", "lhs",
                  33);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/polyfun/polyfit.m"), "context",
                  "context", 34);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 34);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 34);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 34);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 34);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 34);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 34);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 34);
  sf_mex_assign(&c7_rhs34, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs34, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs34), "rhs", "rhs",
                  34);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs34), "lhs", "lhs",
                  34);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/polyfun/polyfit.m"), "context",
                  "context", 35);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_vander"), "name", "name",
                  35);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 35);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_vander.m"), "resolved",
                  "resolved", 35);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1286818802U), "fileTimeLo",
                  "fileTimeLo", 35);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 35);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 35);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 35);
  sf_mex_assign(&c7_rhs35, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs35, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs35), "rhs", "rhs",
                  35);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs35), "lhs", "lhs",
                  35);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_vander.m"), "context",
                  "context", 36);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 36);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 36);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 36);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 36);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 36);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 36);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 36);
  sf_mex_assign(&c7_rhs36, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs36, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs36), "rhs", "rhs",
                  36);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs36), "lhs", "lhs",
                  36);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/polyfun/polyfit.m"), "context",
                  "context", 37);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_qrsolve"), "name", "name",
                  37);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 37);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_qrsolve.m"), "resolved",
                  "resolved", 37);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1360282350U), "fileTimeLo",
                  "fileTimeLo", 37);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 37);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 37);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 37);
  sf_mex_assign(&c7_rhs37, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs37, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs37), "rhs", "rhs",
                  37);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs37), "lhs", "lhs",
                  37);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_qrsolve.m"), "context",
                  "context", 38);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("min"), "name", "name", 38);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 38);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m"), "resolved",
                  "resolved", 38);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1311255318U), "fileTimeLo",
                  "fileTimeLo", 38);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 38);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 38);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 38);
  sf_mex_assign(&c7_rhs38, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs38, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs38), "rhs", "rhs",
                  38);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs38), "lhs", "lhs",
                  38);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m"), "context",
                  "context", 39);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_min_or_max"), "name",
                  "name", 39);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 39);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"),
                  "resolved", "resolved", 39);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1378295984U), "fileTimeLo",
                  "fileTimeLo", 39);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 39);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 39);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 39);
  sf_mex_assign(&c7_rhs39, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs39, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs39), "rhs", "rhs",
                  39);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs39), "lhs", "lhs",
                  39);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum"),
                  "context", "context", 40);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 40);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 40);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 40);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 40);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 40);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 40);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 40);
  sf_mex_assign(&c7_rhs40, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs40, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs40), "rhs", "rhs",
                  40);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs40), "lhs", "lhs",
                  40);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum"),
                  "context", "context", 41);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_scalexp_alloc"), "name",
                  "name", 41);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 41);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "resolved", "resolved", 41);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 41);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 41);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 41);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 41);
  sf_mex_assign(&c7_rhs41, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs41, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs41), "rhs", "rhs",
                  41);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs41), "lhs", "lhs",
                  41);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum"),
                  "context", "context", 42);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 42);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 42);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 42);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1323170578U), "fileTimeLo",
                  "fileTimeLo", 42);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 42);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 42);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 42);
  sf_mex_assign(&c7_rhs42, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs42, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs42), "rhs", "rhs",
                  42);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs42), "lhs", "lhs",
                  42);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum"),
                  "context", "context", 43);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 43);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 43);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 43);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 43);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 43);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 43);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 43);
  sf_mex_assign(&c7_rhs43, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs43, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs43), "rhs", "rhs",
                  43);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs43), "lhs", "lhs",
                  43);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum"),
                  "context", "context", 44);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 44);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 44);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 44);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 44);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 44);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 44);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 44);
  sf_mex_assign(&c7_rhs44, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs44, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs44), "rhs", "rhs",
                  44);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs44), "lhs", "lhs",
                  44);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_qrsolve.m"), "context",
                  "context", 45);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_xgeqp3"), "name", "name",
                  45);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 45);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/eml_xgeqp3.m"),
                  "resolved", "resolved", 45);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1286818804U), "fileTimeLo",
                  "fileTimeLo", 45);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 45);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 45);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 45);
  sf_mex_assign(&c7_rhs45, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs45, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs45), "rhs", "rhs",
                  45);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs45), "lhs", "lhs",
                  45);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/eml_xgeqp3.m"),
                  "context", "context", 46);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_lapack_xgeqp3"), "name",
                  "name", 46);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 46);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xgeqp3.m"),
                  "resolved", "resolved", 46);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1286818808U), "fileTimeLo",
                  "fileTimeLo", 46);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 46);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 46);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 46);
  sf_mex_assign(&c7_rhs46, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs46, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs46), "rhs", "rhs",
                  46);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs46), "lhs", "lhs",
                  46);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/internal/eml_lapack_xgeqp3.m"),
                  "context", "context", 47);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_matlab_zgeqp3"), "name",
                  "name", 47);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 47);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgeqp3.m"),
                  "resolved", "resolved", 47);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1289999366U), "fileTimeLo",
                  "fileTimeLo", 47);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 47);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 47);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 47);
  sf_mex_assign(&c7_rhs47, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs47, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs47), "rhs", "rhs",
                  47);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs47), "lhs", "lhs",
                  47);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgeqp3.m"),
                  "context", "context", 48);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 48);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 48);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 48);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1323170578U), "fileTimeLo",
                  "fileTimeLo", 48);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 48);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 48);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 48);
  sf_mex_assign(&c7_rhs48, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs48, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs48), "rhs", "rhs",
                  48);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs48), "lhs", "lhs",
                  48);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgeqp3.m"),
                  "context", "context", 49);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("min"), "name", "name", 49);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 49);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m"), "resolved",
                  "resolved", 49);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1311255318U), "fileTimeLo",
                  "fileTimeLo", 49);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 49);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 49);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 49);
  sf_mex_assign(&c7_rhs49, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs49, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs49), "rhs", "rhs",
                  49);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs49), "lhs", "lhs",
                  49);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum"),
                  "context", "context", 50);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 50);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 50);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 50);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 50);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 50);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 50);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 50);
  sf_mex_assign(&c7_rhs50, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs50, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs50), "rhs", "rhs",
                  50);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs50), "lhs", "lhs",
                  50);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "context",
                  "context", 51);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 51);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 51);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 51);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1389307920U), "fileTimeLo",
                  "fileTimeLo", 51);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 51);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 51);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 51);
  sf_mex_assign(&c7_rhs51, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs51, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs51), "rhs", "rhs",
                  51);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs51), "lhs", "lhs",
                  51);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum"),
                  "context", "context", 52);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_scalexp_alloc"), "name",
                  "name", 52);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 52);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "resolved", "resolved", 52);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 52);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 52);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 52);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 52);
  sf_mex_assign(&c7_rhs52, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs52, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs52), "rhs", "rhs",
                  52);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs52), "lhs", "lhs",
                  52);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "context", "context", 53);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.scalexpAlloc"),
                  "name", "name", 53);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 53);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalexpAlloc.p"),
                  "resolved", "resolved", 53);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1389307920U), "fileTimeLo",
                  "fileTimeLo", 53);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 53);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 53);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 53);
  sf_mex_assign(&c7_rhs53, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs53, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs53), "rhs", "rhs",
                  53);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs53), "lhs", "lhs",
                  53);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum"),
                  "context", "context", 54);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 54);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 54);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 54);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 54);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 54);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 54);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 54);
  sf_mex_assign(&c7_rhs54, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs54, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs54), "rhs", "rhs",
                  54);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs54), "lhs", "lhs",
                  54);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum"),
                  "context", "context", 55);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 55);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 55);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 55);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 55);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 55);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 55);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 55);
  sf_mex_assign(&c7_rhs55, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs55, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs55), "rhs", "rhs",
                  55);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs55), "lhs", "lhs",
                  55);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgeqp3.m"),
                  "context", "context", 56);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 56);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 56);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 56);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 56);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 56);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 56);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 56);
  sf_mex_assign(&c7_rhs56, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs56, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs56), "rhs", "rhs",
                  56);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs56), "lhs", "lhs",
                  56);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgeqp3.m"),
                  "context", "context", 57);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("colon"), "name", "name", 57);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 57);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m"), "resolved",
                  "resolved", 57);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1378295988U), "fileTimeLo",
                  "fileTimeLo", 57);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 57);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 57);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 57);
  sf_mex_assign(&c7_rhs57, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs57, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs57), "rhs", "rhs",
                  57);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs57), "lhs", "lhs",
                  57);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m"), "context",
                  "context", 58);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("colon"), "name", "name", 58);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 58);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m"), "resolved",
                  "resolved", 58);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1378295988U), "fileTimeLo",
                  "fileTimeLo", 58);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 58);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 58);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 58);
  sf_mex_assign(&c7_rhs58, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs58, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs58), "rhs", "rhs",
                  58);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs58), "lhs", "lhs",
                  58);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m"), "context",
                  "context", 59);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 59);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 59);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 59);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 59);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 59);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 59);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 59);
  sf_mex_assign(&c7_rhs59, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs59, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs59), "rhs", "rhs",
                  59);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs59), "lhs", "lhs",
                  59);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m"), "context",
                  "context", 60);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 60);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 60);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 60);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 60);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 60);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 60);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 60);
  sf_mex_assign(&c7_rhs60, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs60, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs60), "rhs", "rhs",
                  60);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs60), "lhs", "lhs",
                  60);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m"), "context",
                  "context", 61);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("floor"), "name", "name", 61);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 61);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "resolved",
                  "resolved", 61);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363713854U), "fileTimeLo",
                  "fileTimeLo", 61);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 61);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 61);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 61);
  sf_mex_assign(&c7_rhs61, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs61, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs61), "rhs", "rhs",
                  61);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs61), "lhs", "lhs",
                  61);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "context",
                  "context", 62);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 62);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 62);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 62);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 62);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 62);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 62);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 62);
  sf_mex_assign(&c7_rhs62, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs62, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs62), "rhs", "rhs",
                  62);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs62), "lhs", "lhs",
                  62);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "context",
                  "context", 63);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_scalar_floor"), "name",
                  "name", 63);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 63);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m"),
                  "resolved", "resolved", 63);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1286818726U), "fileTimeLo",
                  "fileTimeLo", 63);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 63);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 63);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 63);
  sf_mex_assign(&c7_rhs63, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs63, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs63), "rhs", "rhs",
                  63);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs63), "lhs", "lhs",
                  63);
  sf_mex_destroy(&c7_rhs0);
  sf_mex_destroy(&c7_lhs0);
  sf_mex_destroy(&c7_rhs1);
  sf_mex_destroy(&c7_lhs1);
  sf_mex_destroy(&c7_rhs2);
  sf_mex_destroy(&c7_lhs2);
  sf_mex_destroy(&c7_rhs3);
  sf_mex_destroy(&c7_lhs3);
  sf_mex_destroy(&c7_rhs4);
  sf_mex_destroy(&c7_lhs4);
  sf_mex_destroy(&c7_rhs5);
  sf_mex_destroy(&c7_lhs5);
  sf_mex_destroy(&c7_rhs6);
  sf_mex_destroy(&c7_lhs6);
  sf_mex_destroy(&c7_rhs7);
  sf_mex_destroy(&c7_lhs7);
  sf_mex_destroy(&c7_rhs8);
  sf_mex_destroy(&c7_lhs8);
  sf_mex_destroy(&c7_rhs9);
  sf_mex_destroy(&c7_lhs9);
  sf_mex_destroy(&c7_rhs10);
  sf_mex_destroy(&c7_lhs10);
  sf_mex_destroy(&c7_rhs11);
  sf_mex_destroy(&c7_lhs11);
  sf_mex_destroy(&c7_rhs12);
  sf_mex_destroy(&c7_lhs12);
  sf_mex_destroy(&c7_rhs13);
  sf_mex_destroy(&c7_lhs13);
  sf_mex_destroy(&c7_rhs14);
  sf_mex_destroy(&c7_lhs14);
  sf_mex_destroy(&c7_rhs15);
  sf_mex_destroy(&c7_lhs15);
  sf_mex_destroy(&c7_rhs16);
  sf_mex_destroy(&c7_lhs16);
  sf_mex_destroy(&c7_rhs17);
  sf_mex_destroy(&c7_lhs17);
  sf_mex_destroy(&c7_rhs18);
  sf_mex_destroy(&c7_lhs18);
  sf_mex_destroy(&c7_rhs19);
  sf_mex_destroy(&c7_lhs19);
  sf_mex_destroy(&c7_rhs20);
  sf_mex_destroy(&c7_lhs20);
  sf_mex_destroy(&c7_rhs21);
  sf_mex_destroy(&c7_lhs21);
  sf_mex_destroy(&c7_rhs22);
  sf_mex_destroy(&c7_lhs22);
  sf_mex_destroy(&c7_rhs23);
  sf_mex_destroy(&c7_lhs23);
  sf_mex_destroy(&c7_rhs24);
  sf_mex_destroy(&c7_lhs24);
  sf_mex_destroy(&c7_rhs25);
  sf_mex_destroy(&c7_lhs25);
  sf_mex_destroy(&c7_rhs26);
  sf_mex_destroy(&c7_lhs26);
  sf_mex_destroy(&c7_rhs27);
  sf_mex_destroy(&c7_lhs27);
  sf_mex_destroy(&c7_rhs28);
  sf_mex_destroy(&c7_lhs28);
  sf_mex_destroy(&c7_rhs29);
  sf_mex_destroy(&c7_lhs29);
  sf_mex_destroy(&c7_rhs30);
  sf_mex_destroy(&c7_lhs30);
  sf_mex_destroy(&c7_rhs31);
  sf_mex_destroy(&c7_lhs31);
  sf_mex_destroy(&c7_rhs32);
  sf_mex_destroy(&c7_lhs32);
  sf_mex_destroy(&c7_rhs33);
  sf_mex_destroy(&c7_lhs33);
  sf_mex_destroy(&c7_rhs34);
  sf_mex_destroy(&c7_lhs34);
  sf_mex_destroy(&c7_rhs35);
  sf_mex_destroy(&c7_lhs35);
  sf_mex_destroy(&c7_rhs36);
  sf_mex_destroy(&c7_lhs36);
  sf_mex_destroy(&c7_rhs37);
  sf_mex_destroy(&c7_lhs37);
  sf_mex_destroy(&c7_rhs38);
  sf_mex_destroy(&c7_lhs38);
  sf_mex_destroy(&c7_rhs39);
  sf_mex_destroy(&c7_lhs39);
  sf_mex_destroy(&c7_rhs40);
  sf_mex_destroy(&c7_lhs40);
  sf_mex_destroy(&c7_rhs41);
  sf_mex_destroy(&c7_lhs41);
  sf_mex_destroy(&c7_rhs42);
  sf_mex_destroy(&c7_lhs42);
  sf_mex_destroy(&c7_rhs43);
  sf_mex_destroy(&c7_lhs43);
  sf_mex_destroy(&c7_rhs44);
  sf_mex_destroy(&c7_lhs44);
  sf_mex_destroy(&c7_rhs45);
  sf_mex_destroy(&c7_lhs45);
  sf_mex_destroy(&c7_rhs46);
  sf_mex_destroy(&c7_lhs46);
  sf_mex_destroy(&c7_rhs47);
  sf_mex_destroy(&c7_lhs47);
  sf_mex_destroy(&c7_rhs48);
  sf_mex_destroy(&c7_lhs48);
  sf_mex_destroy(&c7_rhs49);
  sf_mex_destroy(&c7_lhs49);
  sf_mex_destroy(&c7_rhs50);
  sf_mex_destroy(&c7_lhs50);
  sf_mex_destroy(&c7_rhs51);
  sf_mex_destroy(&c7_lhs51);
  sf_mex_destroy(&c7_rhs52);
  sf_mex_destroy(&c7_lhs52);
  sf_mex_destroy(&c7_rhs53);
  sf_mex_destroy(&c7_lhs53);
  sf_mex_destroy(&c7_rhs54);
  sf_mex_destroy(&c7_lhs54);
  sf_mex_destroy(&c7_rhs55);
  sf_mex_destroy(&c7_lhs55);
  sf_mex_destroy(&c7_rhs56);
  sf_mex_destroy(&c7_lhs56);
  sf_mex_destroy(&c7_rhs57);
  sf_mex_destroy(&c7_lhs57);
  sf_mex_destroy(&c7_rhs58);
  sf_mex_destroy(&c7_lhs58);
  sf_mex_destroy(&c7_rhs59);
  sf_mex_destroy(&c7_lhs59);
  sf_mex_destroy(&c7_rhs60);
  sf_mex_destroy(&c7_lhs60);
  sf_mex_destroy(&c7_rhs61);
  sf_mex_destroy(&c7_lhs61);
  sf_mex_destroy(&c7_rhs62);
  sf_mex_destroy(&c7_lhs62);
  sf_mex_destroy(&c7_rhs63);
  sf_mex_destroy(&c7_lhs63);
}

static const mxArray *c7_emlrt_marshallOut(const char * c7_u)
{
  const mxArray *c7_y = NULL;
  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_create("y", c7_u, 15, 0U, 0U, 0U, 2, 1, strlen
    (c7_u)), false);
  return c7_y;
}

static const mxArray *c7_b_emlrt_marshallOut(const uint32_T c7_u)
{
  const mxArray *c7_y = NULL;
  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_create("y", &c7_u, 7, 0U, 0U, 0U, 0), false);
  return c7_y;
}

static void c7_b_info_helper(const mxArray **c7_info)
{
  const mxArray *c7_rhs64 = NULL;
  const mxArray *c7_lhs64 = NULL;
  const mxArray *c7_rhs65 = NULL;
  const mxArray *c7_lhs65 = NULL;
  const mxArray *c7_rhs66 = NULL;
  const mxArray *c7_lhs66 = NULL;
  const mxArray *c7_rhs67 = NULL;
  const mxArray *c7_lhs67 = NULL;
  const mxArray *c7_rhs68 = NULL;
  const mxArray *c7_lhs68 = NULL;
  const mxArray *c7_rhs69 = NULL;
  const mxArray *c7_lhs69 = NULL;
  const mxArray *c7_rhs70 = NULL;
  const mxArray *c7_lhs70 = NULL;
  const mxArray *c7_rhs71 = NULL;
  const mxArray *c7_lhs71 = NULL;
  const mxArray *c7_rhs72 = NULL;
  const mxArray *c7_lhs72 = NULL;
  const mxArray *c7_rhs73 = NULL;
  const mxArray *c7_lhs73 = NULL;
  const mxArray *c7_rhs74 = NULL;
  const mxArray *c7_lhs74 = NULL;
  const mxArray *c7_rhs75 = NULL;
  const mxArray *c7_lhs75 = NULL;
  const mxArray *c7_rhs76 = NULL;
  const mxArray *c7_lhs76 = NULL;
  const mxArray *c7_rhs77 = NULL;
  const mxArray *c7_lhs77 = NULL;
  const mxArray *c7_rhs78 = NULL;
  const mxArray *c7_lhs78 = NULL;
  const mxArray *c7_rhs79 = NULL;
  const mxArray *c7_lhs79 = NULL;
  const mxArray *c7_rhs80 = NULL;
  const mxArray *c7_lhs80 = NULL;
  const mxArray *c7_rhs81 = NULL;
  const mxArray *c7_lhs81 = NULL;
  const mxArray *c7_rhs82 = NULL;
  const mxArray *c7_lhs82 = NULL;
  const mxArray *c7_rhs83 = NULL;
  const mxArray *c7_lhs83 = NULL;
  const mxArray *c7_rhs84 = NULL;
  const mxArray *c7_lhs84 = NULL;
  const mxArray *c7_rhs85 = NULL;
  const mxArray *c7_lhs85 = NULL;
  const mxArray *c7_rhs86 = NULL;
  const mxArray *c7_lhs86 = NULL;
  const mxArray *c7_rhs87 = NULL;
  const mxArray *c7_lhs87 = NULL;
  const mxArray *c7_rhs88 = NULL;
  const mxArray *c7_lhs88 = NULL;
  const mxArray *c7_rhs89 = NULL;
  const mxArray *c7_lhs89 = NULL;
  const mxArray *c7_rhs90 = NULL;
  const mxArray *c7_lhs90 = NULL;
  const mxArray *c7_rhs91 = NULL;
  const mxArray *c7_lhs91 = NULL;
  const mxArray *c7_rhs92 = NULL;
  const mxArray *c7_lhs92 = NULL;
  const mxArray *c7_rhs93 = NULL;
  const mxArray *c7_lhs93 = NULL;
  const mxArray *c7_rhs94 = NULL;
  const mxArray *c7_lhs94 = NULL;
  const mxArray *c7_rhs95 = NULL;
  const mxArray *c7_lhs95 = NULL;
  const mxArray *c7_rhs96 = NULL;
  const mxArray *c7_lhs96 = NULL;
  const mxArray *c7_rhs97 = NULL;
  const mxArray *c7_lhs97 = NULL;
  const mxArray *c7_rhs98 = NULL;
  const mxArray *c7_lhs98 = NULL;
  const mxArray *c7_rhs99 = NULL;
  const mxArray *c7_lhs99 = NULL;
  const mxArray *c7_rhs100 = NULL;
  const mxArray *c7_lhs100 = NULL;
  const mxArray *c7_rhs101 = NULL;
  const mxArray *c7_lhs101 = NULL;
  const mxArray *c7_rhs102 = NULL;
  const mxArray *c7_lhs102 = NULL;
  const mxArray *c7_rhs103 = NULL;
  const mxArray *c7_lhs103 = NULL;
  const mxArray *c7_rhs104 = NULL;
  const mxArray *c7_lhs104 = NULL;
  const mxArray *c7_rhs105 = NULL;
  const mxArray *c7_lhs105 = NULL;
  const mxArray *c7_rhs106 = NULL;
  const mxArray *c7_lhs106 = NULL;
  const mxArray *c7_rhs107 = NULL;
  const mxArray *c7_lhs107 = NULL;
  const mxArray *c7_rhs108 = NULL;
  const mxArray *c7_lhs108 = NULL;
  const mxArray *c7_rhs109 = NULL;
  const mxArray *c7_lhs109 = NULL;
  const mxArray *c7_rhs110 = NULL;
  const mxArray *c7_lhs110 = NULL;
  const mxArray *c7_rhs111 = NULL;
  const mxArray *c7_lhs111 = NULL;
  const mxArray *c7_rhs112 = NULL;
  const mxArray *c7_lhs112 = NULL;
  const mxArray *c7_rhs113 = NULL;
  const mxArray *c7_lhs113 = NULL;
  const mxArray *c7_rhs114 = NULL;
  const mxArray *c7_lhs114 = NULL;
  const mxArray *c7_rhs115 = NULL;
  const mxArray *c7_lhs115 = NULL;
  const mxArray *c7_rhs116 = NULL;
  const mxArray *c7_lhs116 = NULL;
  const mxArray *c7_rhs117 = NULL;
  const mxArray *c7_lhs117 = NULL;
  const mxArray *c7_rhs118 = NULL;
  const mxArray *c7_lhs118 = NULL;
  const mxArray *c7_rhs119 = NULL;
  const mxArray *c7_lhs119 = NULL;
  const mxArray *c7_rhs120 = NULL;
  const mxArray *c7_lhs120 = NULL;
  const mxArray *c7_rhs121 = NULL;
  const mxArray *c7_lhs121 = NULL;
  const mxArray *c7_rhs122 = NULL;
  const mxArray *c7_lhs122 = NULL;
  const mxArray *c7_rhs123 = NULL;
  const mxArray *c7_lhs123 = NULL;
  const mxArray *c7_rhs124 = NULL;
  const mxArray *c7_lhs124 = NULL;
  const mxArray *c7_rhs125 = NULL;
  const mxArray *c7_lhs125 = NULL;
  const mxArray *c7_rhs126 = NULL;
  const mxArray *c7_lhs126 = NULL;
  const mxArray *c7_rhs127 = NULL;
  const mxArray *c7_lhs127 = NULL;
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!checkrange"),
                  "context", "context", 64);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("intmin"), "name", "name", 64);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 64);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m"), "resolved",
                  "resolved", 64);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1362261882U), "fileTimeLo",
                  "fileTimeLo", 64);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 64);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 64);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 64);
  sf_mex_assign(&c7_rhs64, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs64, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs64), "rhs", "rhs",
                  64);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs64), "lhs", "lhs",
                  64);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m"), "context",
                  "context", 65);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 65);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 65);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 65);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1381850300U), "fileTimeLo",
                  "fileTimeLo", 65);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 65);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 65);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 65);
  sf_mex_assign(&c7_rhs65, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs65, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs65), "rhs", "rhs",
                  65);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs65), "lhs", "lhs",
                  65);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!checkrange"),
                  "context", "context", 66);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("intmax"), "name", "name", 66);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 66);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "resolved",
                  "resolved", 66);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1362261882U), "fileTimeLo",
                  "fileTimeLo", 66);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 66);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 66);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 66);
  sf_mex_assign(&c7_rhs66, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs66, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs66), "rhs", "rhs",
                  66);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs66), "lhs", "lhs",
                  66);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "context",
                  "context", 67);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 67);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 67);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 67);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1381850300U), "fileTimeLo",
                  "fileTimeLo", 67);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 67);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 67);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 67);
  sf_mex_assign(&c7_rhs67, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs67, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs67), "rhs", "rhs",
                  67);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs67), "lhs", "lhs",
                  67);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher"),
                  "context", "context", 68);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("intmin"), "name", "name", 68);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 68);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m"), "resolved",
                  "resolved", 68);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1362261882U), "fileTimeLo",
                  "fileTimeLo", 68);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 68);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 68);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 68);
  sf_mex_assign(&c7_rhs68, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs68, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs68), "rhs", "rhs",
                  68);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs68), "lhs", "lhs",
                  68);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher"),
                  "context", "context", 69);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("intmax"), "name", "name", 69);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 69);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "resolved",
                  "resolved", 69);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1362261882U), "fileTimeLo",
                  "fileTimeLo", 69);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 69);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 69);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 69);
  sf_mex_assign(&c7_rhs69, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs69, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs69), "rhs", "rhs",
                  69);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs69), "lhs", "lhs",
                  69);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_integer_colon_dispatcher"),
                  "context", "context", 70);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_isa_uint"), "name", "name",
                  70);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 70);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isa_uint.m"), "resolved",
                  "resolved", 70);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 70);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 70);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 70);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 70);
  sf_mex_assign(&c7_rhs70, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs70, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs70), "rhs", "rhs",
                  70);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs70), "lhs", "lhs",
                  70);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isa_uint.m"), "context",
                  "context", 71);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.isaUint"),
                  "name", "name", 71);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 71);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/isaUint.p"),
                  "resolved", "resolved", 71);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1389307920U), "fileTimeLo",
                  "fileTimeLo", 71);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 71);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 71);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 71);
  sf_mex_assign(&c7_rhs71, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs71, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs71), "rhs", "rhs",
                  71);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs71), "lhs", "lhs",
                  71);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd"),
                  "context", "context", 72);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_unsigned_class"), "name",
                  "name", 72);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 72);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_unsigned_class.m"),
                  "resolved", "resolved", 72);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 72);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 72);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 72);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 72);
  sf_mex_assign(&c7_rhs72, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs72, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs72), "rhs", "rhs",
                  72);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs72), "lhs", "lhs",
                  72);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_unsigned_class.m"),
                  "context", "context", 73);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.unsignedClass"),
                  "name", "name", 73);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 73);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/unsignedClass.p"),
                  "resolved", "resolved", 73);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 73);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 73);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 73);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 73);
  sf_mex_assign(&c7_rhs73, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs73, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs73), "rhs", "rhs",
                  73);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs73), "lhs", "lhs",
                  73);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/unsignedClass.p"),
                  "context", "context", 74);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 74);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 74);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 74);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1381850300U), "fileTimeLo",
                  "fileTimeLo", 74);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 74);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 74);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 74);
  sf_mex_assign(&c7_rhs74, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs74, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs74), "rhs", "rhs",
                  74);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs74), "lhs", "lhs",
                  74);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/unsignedClass.p"),
                  "context", "context", 75);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 75);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 75);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 75);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1323170578U), "fileTimeLo",
                  "fileTimeLo", 75);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 75);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 75);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 75);
  sf_mex_assign(&c7_rhs75, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs75, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs75), "rhs", "rhs",
                  75);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs75), "lhs", "lhs",
                  75);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd"),
                  "context", "context", 76);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 76);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 76);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 76);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1323170578U), "fileTimeLo",
                  "fileTimeLo", 76);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 76);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 76);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 76);
  sf_mex_assign(&c7_rhs76, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs76, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs76), "rhs", "rhs",
                  76);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs76), "lhs", "lhs",
                  76);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd"),
                  "context", "context", 77);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("intmax"), "name", "name", 77);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 77);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "resolved",
                  "resolved", 77);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1362261882U), "fileTimeLo",
                  "fileTimeLo", 77);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 77);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 77);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 77);
  sf_mex_assign(&c7_rhs77, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs77, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs77), "rhs", "rhs",
                  77);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs77), "lhs", "lhs",
                  77);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd"),
                  "context", "context", 78);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_isa_uint"), "name", "name",
                  78);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 78);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isa_uint.m"), "resolved",
                  "resolved", 78);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 78);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 78);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 78);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 78);
  sf_mex_assign(&c7_rhs78, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs78, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs78), "rhs", "rhs",
                  78);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs78), "lhs", "lhs",
                  78);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!integer_colon_length_nonnegd"),
                  "context", "context", 79);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 79);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 79);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 79);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1372582416U), "fileTimeLo",
                  "fileTimeLo", 79);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 79);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 79);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 79);
  sf_mex_assign(&c7_rhs79, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs79, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs79), "rhs", "rhs",
                  79);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs79), "lhs", "lhs",
                  79);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"), "context",
                  "context", 80);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 80);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 80);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 80);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 80);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 80);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 80);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 80);
  sf_mex_assign(&c7_rhs80, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs80, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs80), "rhs", "rhs",
                  80);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs80), "lhs", "lhs",
                  80);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgeqp3.m"),
                  "context", "context", 81);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eps"), "name", "name", 81);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 81);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m"), "resolved",
                  "resolved", 81);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1326727996U), "fileTimeLo",
                  "fileTimeLo", 81);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 81);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 81);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 81);
  sf_mex_assign(&c7_rhs81, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs81, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs81), "rhs", "rhs",
                  81);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs81), "lhs", "lhs",
                  81);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m"), "context",
                  "context", 82);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_is_float_class"), "name",
                  "name", 82);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 82);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_float_class.m"),
                  "resolved", "resolved", 82);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1286818782U), "fileTimeLo",
                  "fileTimeLo", 82);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 82);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 82);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 82);
  sf_mex_assign(&c7_rhs82, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs82, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs82), "rhs", "rhs",
                  82);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs82), "lhs", "lhs",
                  82);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m"), "context",
                  "context", 83);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_eps"), "name", "name", 83);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 83);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m"), "resolved",
                  "resolved", 83);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1326727996U), "fileTimeLo",
                  "fileTimeLo", 83);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 83);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 83);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 83);
  sf_mex_assign(&c7_rhs83, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs83, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs83), "rhs", "rhs",
                  83);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs83), "lhs", "lhs",
                  83);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_eps.m"), "context",
                  "context", 84);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_float_model"), "name",
                  "name", 84);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 84);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m"),
                  "resolved", "resolved", 84);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1326727996U), "fileTimeLo",
                  "fileTimeLo", 84);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 84);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 84);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 84);
  sf_mex_assign(&c7_rhs84, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs84, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs84), "rhs", "rhs",
                  84);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs84), "lhs", "lhs",
                  84);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgeqp3.m"),
                  "context", "context", 85);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("sqrt"), "name", "name", 85);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 85);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "resolved",
                  "resolved", 85);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1343830386U), "fileTimeLo",
                  "fileTimeLo", 85);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 85);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 85);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 85);
  sf_mex_assign(&c7_rhs85, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs85, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs85), "rhs", "rhs",
                  85);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs85), "lhs", "lhs",
                  85);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "context",
                  "context", 86);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_error"), "name", "name",
                  86);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 86);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m"), "resolved",
                  "resolved", 86);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1343830358U), "fileTimeLo",
                  "fileTimeLo", 86);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 86);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 86);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 86);
  sf_mex_assign(&c7_rhs86, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs86, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs86), "rhs", "rhs",
                  86);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs86), "lhs", "lhs",
                  86);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "context",
                  "context", 87);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_scalar_sqrt"), "name",
                  "name", 87);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 87);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sqrt.m"),
                  "resolved", "resolved", 87);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1286818738U), "fileTimeLo",
                  "fileTimeLo", 87);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 87);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 87);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 87);
  sf_mex_assign(&c7_rhs87, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs87, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs87), "rhs", "rhs",
                  87);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs87), "lhs", "lhs",
                  87);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgeqp3.m"),
                  "context", "context", 88);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 88);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 88);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 88);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 88);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 88);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 88);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 88);
  sf_mex_assign(&c7_rhs88, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs88, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs88), "rhs", "rhs",
                  88);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs88), "lhs", "lhs",
                  88);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper"),
                  "context", "context", 89);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("intmax"), "name", "name", 89);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 89);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "resolved",
                  "resolved", 89);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1362261882U), "fileTimeLo",
                  "fileTimeLo", 89);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 89);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 89);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 89);
  sf_mex_assign(&c7_rhs89, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs89, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs89), "rhs", "rhs",
                  89);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs89), "lhs", "lhs",
                  89);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgeqp3.m"),
                  "context", "context", 90);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_xnrm2"), "name", "name",
                  90);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 90);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xnrm2.m"),
                  "resolved", "resolved", 90);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1375980692U), "fileTimeLo",
                  "fileTimeLo", 90);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 90);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 90);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 90);
  sf_mex_assign(&c7_rhs90, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs90, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs90), "rhs", "rhs",
                  90);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs90), "lhs", "lhs",
                  90);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xnrm2.m"), "context",
                  "context", 91);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 91);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 91);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 91);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 91);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 91);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 91);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 91);
  sf_mex_assign(&c7_rhs91, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs91, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs91), "rhs", "rhs",
                  91);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs91), "lhs", "lhs",
                  91);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xnrm2.m"), "context",
                  "context", 92);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.blas.xnrm2"),
                  "name", "name", 92);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 92);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xnrm2.p"),
                  "resolved", "resolved", 92);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 92);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 92);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 92);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 92);
  sf_mex_assign(&c7_rhs92, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs92, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs92), "rhs", "rhs",
                  92);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs92), "lhs", "lhs",
                  92);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xnrm2.p"),
                  "context", "context", 93);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 93);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 93);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 93);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 93);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 93);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 93);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 93);
  sf_mex_assign(&c7_rhs93, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs93, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs93), "rhs", "rhs",
                  93);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs93), "lhs", "lhs",
                  93);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xnrm2.p!below_threshold"),
                  "context", "context", 94);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.blas.threshold"),
                  "name", "name", 94);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 94);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 94);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 94);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 94);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 94);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 94);
  sf_mex_assign(&c7_rhs94, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs94, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs94), "rhs", "rhs",
                  94);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs94), "lhs", "lhs",
                  94);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "context", "context", 95);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 95);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 95);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 95);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1381850300U), "fileTimeLo",
                  "fileTimeLo", 95);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 95);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 95);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 95);
  sf_mex_assign(&c7_rhs95, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs95, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs95), "rhs", "rhs",
                  95);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs95), "lhs", "lhs",
                  95);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xnrm2.p"),
                  "context", "context", 96);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.refblas.xnrm2"),
                  "name", "name", 96);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 96);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "resolved", "resolved", 96);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 96);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 96);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 96);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 96);
  sf_mex_assign(&c7_rhs96, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs96, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs96), "rhs", "rhs",
                  96);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs96), "lhs", "lhs",
                  96);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "context", "context", 97);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("realmin"), "name", "name", 97);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 97);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m"), "resolved",
                  "resolved", 97);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1307651242U), "fileTimeLo",
                  "fileTimeLo", 97);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 97);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 97);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 97);
  sf_mex_assign(&c7_rhs97, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs97, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs97), "rhs", "rhs",
                  97);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs97), "lhs", "lhs",
                  97);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m"), "context",
                  "context", 98);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_realmin"), "name", "name",
                  98);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 98);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmin.m"), "resolved",
                  "resolved", 98);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1307651244U), "fileTimeLo",
                  "fileTimeLo", 98);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 98);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 98);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 98);
  sf_mex_assign(&c7_rhs98, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs98, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs98), "rhs", "rhs",
                  98);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs98), "lhs", "lhs",
                  98);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmin.m"), "context",
                  "context", 99);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_float_model"), "name",
                  "name", 99);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 99);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m"),
                  "resolved", "resolved", 99);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1326727996U), "fileTimeLo",
                  "fileTimeLo", 99);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 99);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 99);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 99);
  sf_mex_assign(&c7_rhs99, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs99, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs99), "rhs", "rhs",
                  99);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs99), "lhs", "lhs",
                  99);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "context", "context", 100);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexMinus"),
                  "name", "name", 100);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 100);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexMinus.m"),
                  "resolved", "resolved", 100);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 100);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 100);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 100);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 100);
  sf_mex_assign(&c7_rhs100, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs100, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs100), "rhs", "rhs",
                  100);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs100), "lhs", "lhs",
                  100);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "context", "context", 101);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexTimes"),
                  "name", "name", 101);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 101);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexTimes.m"),
                  "resolved", "resolved", 101);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 101);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 101);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 101);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 101);
  sf_mex_assign(&c7_rhs101, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs101, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs101), "rhs", "rhs",
                  101);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs101), "lhs", "lhs",
                  101);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "context", "context", 102);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 102);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 102);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 102);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 102);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 102);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 102);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 102);
  sf_mex_assign(&c7_rhs102, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs102, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs102), "rhs", "rhs",
                  102);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs102), "lhs", "lhs",
                  102);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "context", "context", 103);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 103);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 103);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 103);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 103);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 103);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 103);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 103);
  sf_mex_assign(&c7_rhs103, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs103, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs103), "rhs", "rhs",
                  103);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs103), "lhs", "lhs",
                  103);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xnrm2.p"),
                  "context", "context", 104);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("abs"), "name", "name", 104);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 104);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "resolved",
                  "resolved", 104);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363713852U), "fileTimeLo",
                  "fileTimeLo", 104);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 104);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 104);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 104);
  sf_mex_assign(&c7_rhs104, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs104, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs104), "rhs", "rhs",
                  104);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs104), "lhs", "lhs",
                  104);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgeqp3.m"),
                  "context", "context", 105);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 105);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 105);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 105);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1372582416U), "fileTimeLo",
                  "fileTimeLo", 105);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 105);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 105);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 105);
  sf_mex_assign(&c7_rhs105, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs105, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs105), "rhs", "rhs",
                  105);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs105), "lhs", "lhs",
                  105);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"), "context",
                  "context", 106);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 106);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 106);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 106);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 106);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 106);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 106);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 106);
  sf_mex_assign(&c7_rhs106, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs106, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs106), "rhs", "rhs",
                  106);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs106), "lhs", "lhs",
                  106);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgeqp3.m"),
                  "context", "context", 107);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_index_minus"), "name",
                  "name", 107);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 107);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m"),
                  "resolved", "resolved", 107);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1372582416U), "fileTimeLo",
                  "fileTimeLo", 107);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 107);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 107);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 107);
  sf_mex_assign(&c7_rhs107, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs107, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs107), "rhs", "rhs",
                  107);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs107), "lhs", "lhs",
                  107);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m"),
                  "context", "context", 108);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexMinus"),
                  "name", "name", 108);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 108);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexMinus.m"),
                  "resolved", "resolved", 108);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 108);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 108);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 108);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 108);
  sf_mex_assign(&c7_rhs108, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs108, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs108), "rhs", "rhs",
                  108);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs108), "lhs", "lhs",
                  108);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgeqp3.m"),
                  "context", "context", 109);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 109);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 109);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 109);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1372582416U), "fileTimeLo",
                  "fileTimeLo", 109);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 109);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 109);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 109);
  sf_mex_assign(&c7_rhs109, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs109, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs109), "rhs", "rhs",
                  109);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs109), "lhs", "lhs",
                  109);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgeqp3.m"),
                  "context", "context", 110);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_index_times"), "name",
                  "name", 110);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 110);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m"),
                  "resolved", "resolved", 110);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1372582416U), "fileTimeLo",
                  "fileTimeLo", 110);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 110);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 110);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 110);
  sf_mex_assign(&c7_rhs110, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs110, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs110), "rhs", "rhs",
                  110);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs110), "lhs", "lhs",
                  110);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m"),
                  "context", "context", 111);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexTimes"),
                  "name", "name", 111);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 111);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexTimes.m"),
                  "resolved", "resolved", 111);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 111);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 111);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 111);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 111);
  sf_mex_assign(&c7_rhs111, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs111, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs111), "rhs", "rhs",
                  111);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs111), "lhs", "lhs",
                  111);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgeqp3.m"),
                  "context", "context", 112);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_index_minus"), "name",
                  "name", 112);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 112);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m"),
                  "resolved", "resolved", 112);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1372582416U), "fileTimeLo",
                  "fileTimeLo", 112);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 112);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 112);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 112);
  sf_mex_assign(&c7_rhs112, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs112, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs112), "rhs", "rhs",
                  112);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs112), "lhs", "lhs",
                  112);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m"),
                  "context", "context", 113);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexMinus"),
                  "name", "name", 113);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 113);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexMinus.m"),
                  "resolved", "resolved", 113);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 113);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 113);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 113);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 113);
  sf_mex_assign(&c7_rhs113, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs113, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs113), "rhs", "rhs",
                  113);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs113), "lhs", "lhs",
                  113);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgeqp3.m"),
                  "context", "context", 114);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_ixamax"), "name", "name",
                  114);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 114);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_ixamax.m"),
                  "resolved", "resolved", 114);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 114);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 114);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 114);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 114);
  sf_mex_assign(&c7_rhs114, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs114, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs114), "rhs", "rhs",
                  114);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs114), "lhs", "lhs",
                  114);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_ixamax.m"),
                  "context", "context", 115);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 115);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 115);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 115);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 115);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 115);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 115);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 115);
  sf_mex_assign(&c7_rhs115, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs115, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs115), "rhs", "rhs",
                  115);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs115), "lhs", "lhs",
                  115);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_ixamax.m"),
                  "context", "context", 116);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.blas.ixamax"),
                  "name", "name", 116);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 116);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/ixamax.p"),
                  "resolved", "resolved", 116);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 116);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 116);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 116);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 116);
  sf_mex_assign(&c7_rhs116, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs116, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs116), "rhs", "rhs",
                  116);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs116), "lhs", "lhs",
                  116);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/ixamax.p"),
                  "context", "context", 117);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 117);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 117);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 117);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 117);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 117);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 117);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 117);
  sf_mex_assign(&c7_rhs117, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs117, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs117), "rhs", "rhs",
                  117);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs117), "lhs", "lhs",
                  117);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/ixamax.p!below_threshold"),
                  "context", "context", 118);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.blas.threshold"),
                  "name", "name", 118);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 118);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 118);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 118);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 118);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 118);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 118);
  sf_mex_assign(&c7_rhs118, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs118, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs118), "rhs", "rhs",
                  118);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs118), "lhs", "lhs",
                  118);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/ixamax.p!below_threshold"),
                  "context", "context", 119);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("length"), "name", "name", 119);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 119);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m"), "resolved",
                  "resolved", 119);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1303146206U), "fileTimeLo",
                  "fileTimeLo", 119);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 119);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 119);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 119);
  sf_mex_assign(&c7_rhs119, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs119, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs119), "rhs", "rhs",
                  119);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs119), "lhs", "lhs",
                  119);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/ixamax.p"),
                  "context", "context", 120);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.refblas.ixamax"),
                  "name", "name", 120);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 120);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/ixamax.p"),
                  "resolved", "resolved", 120);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 120);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 120);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 120);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 120);
  sf_mex_assign(&c7_rhs120, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs120, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs120), "rhs", "rhs",
                  120);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs120), "lhs", "lhs",
                  120);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/ixamax.p"),
                  "context", "context", 121);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.refblas.xcabs1"),
                  "name", "name", 121);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 121);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xcabs1.p"),
                  "resolved", "resolved", 121);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 121);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 121);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 121);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 121);
  sf_mex_assign(&c7_rhs121, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs121, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs121), "rhs", "rhs",
                  121);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs121), "lhs", "lhs",
                  121);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xcabs1.p"),
                  "context", "context", 122);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("abs"), "name", "name", 122);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 122);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "resolved",
                  "resolved", 122);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363713852U), "fileTimeLo",
                  "fileTimeLo", 122);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 122);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 122);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 122);
  sf_mex_assign(&c7_rhs122, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs122, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs122), "rhs", "rhs",
                  122);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs122), "lhs", "lhs",
                  122);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/ixamax.p"),
                  "context", "context", 123);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 123);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 123);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 123);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 123);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 123);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 123);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 123);
  sf_mex_assign(&c7_rhs123, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs123, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs123), "rhs", "rhs",
                  123);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs123), "lhs", "lhs",
                  123);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/ixamax.p"),
                  "context", "context", 124);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 124);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 124);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 124);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 124);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 124);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 124);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 124);
  sf_mex_assign(&c7_rhs124, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs124, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs124), "rhs", "rhs",
                  124);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs124), "lhs", "lhs",
                  124);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgeqp3.m"),
                  "context", "context", 125);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_xswap"), "name", "name",
                  125);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 125);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m"),
                  "resolved", "resolved", 125);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1375980692U), "fileTimeLo",
                  "fileTimeLo", 125);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 125);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 125);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 125);
  sf_mex_assign(&c7_rhs125, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs125, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs125), "rhs", "rhs",
                  125);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs125), "lhs", "lhs",
                  125);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m"), "context",
                  "context", 126);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 126);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 126);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 126);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 126);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 126);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 126);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 126);
  sf_mex_assign(&c7_rhs126, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs126, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs126), "rhs", "rhs",
                  126);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs126), "lhs", "lhs",
                  126);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xswap.m"), "context",
                  "context", 127);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.blas.xswap"),
                  "name", "name", 127);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 127);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xswap.p"),
                  "resolved", "resolved", 127);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 127);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 127);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 127);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 127);
  sf_mex_assign(&c7_rhs127, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs127, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs127), "rhs", "rhs",
                  127);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs127), "lhs", "lhs",
                  127);
  sf_mex_destroy(&c7_rhs64);
  sf_mex_destroy(&c7_lhs64);
  sf_mex_destroy(&c7_rhs65);
  sf_mex_destroy(&c7_lhs65);
  sf_mex_destroy(&c7_rhs66);
  sf_mex_destroy(&c7_lhs66);
  sf_mex_destroy(&c7_rhs67);
  sf_mex_destroy(&c7_lhs67);
  sf_mex_destroy(&c7_rhs68);
  sf_mex_destroy(&c7_lhs68);
  sf_mex_destroy(&c7_rhs69);
  sf_mex_destroy(&c7_lhs69);
  sf_mex_destroy(&c7_rhs70);
  sf_mex_destroy(&c7_lhs70);
  sf_mex_destroy(&c7_rhs71);
  sf_mex_destroy(&c7_lhs71);
  sf_mex_destroy(&c7_rhs72);
  sf_mex_destroy(&c7_lhs72);
  sf_mex_destroy(&c7_rhs73);
  sf_mex_destroy(&c7_lhs73);
  sf_mex_destroy(&c7_rhs74);
  sf_mex_destroy(&c7_lhs74);
  sf_mex_destroy(&c7_rhs75);
  sf_mex_destroy(&c7_lhs75);
  sf_mex_destroy(&c7_rhs76);
  sf_mex_destroy(&c7_lhs76);
  sf_mex_destroy(&c7_rhs77);
  sf_mex_destroy(&c7_lhs77);
  sf_mex_destroy(&c7_rhs78);
  sf_mex_destroy(&c7_lhs78);
  sf_mex_destroy(&c7_rhs79);
  sf_mex_destroy(&c7_lhs79);
  sf_mex_destroy(&c7_rhs80);
  sf_mex_destroy(&c7_lhs80);
  sf_mex_destroy(&c7_rhs81);
  sf_mex_destroy(&c7_lhs81);
  sf_mex_destroy(&c7_rhs82);
  sf_mex_destroy(&c7_lhs82);
  sf_mex_destroy(&c7_rhs83);
  sf_mex_destroy(&c7_lhs83);
  sf_mex_destroy(&c7_rhs84);
  sf_mex_destroy(&c7_lhs84);
  sf_mex_destroy(&c7_rhs85);
  sf_mex_destroy(&c7_lhs85);
  sf_mex_destroy(&c7_rhs86);
  sf_mex_destroy(&c7_lhs86);
  sf_mex_destroy(&c7_rhs87);
  sf_mex_destroy(&c7_lhs87);
  sf_mex_destroy(&c7_rhs88);
  sf_mex_destroy(&c7_lhs88);
  sf_mex_destroy(&c7_rhs89);
  sf_mex_destroy(&c7_lhs89);
  sf_mex_destroy(&c7_rhs90);
  sf_mex_destroy(&c7_lhs90);
  sf_mex_destroy(&c7_rhs91);
  sf_mex_destroy(&c7_lhs91);
  sf_mex_destroy(&c7_rhs92);
  sf_mex_destroy(&c7_lhs92);
  sf_mex_destroy(&c7_rhs93);
  sf_mex_destroy(&c7_lhs93);
  sf_mex_destroy(&c7_rhs94);
  sf_mex_destroy(&c7_lhs94);
  sf_mex_destroy(&c7_rhs95);
  sf_mex_destroy(&c7_lhs95);
  sf_mex_destroy(&c7_rhs96);
  sf_mex_destroy(&c7_lhs96);
  sf_mex_destroy(&c7_rhs97);
  sf_mex_destroy(&c7_lhs97);
  sf_mex_destroy(&c7_rhs98);
  sf_mex_destroy(&c7_lhs98);
  sf_mex_destroy(&c7_rhs99);
  sf_mex_destroy(&c7_lhs99);
  sf_mex_destroy(&c7_rhs100);
  sf_mex_destroy(&c7_lhs100);
  sf_mex_destroy(&c7_rhs101);
  sf_mex_destroy(&c7_lhs101);
  sf_mex_destroy(&c7_rhs102);
  sf_mex_destroy(&c7_lhs102);
  sf_mex_destroy(&c7_rhs103);
  sf_mex_destroy(&c7_lhs103);
  sf_mex_destroy(&c7_rhs104);
  sf_mex_destroy(&c7_lhs104);
  sf_mex_destroy(&c7_rhs105);
  sf_mex_destroy(&c7_lhs105);
  sf_mex_destroy(&c7_rhs106);
  sf_mex_destroy(&c7_lhs106);
  sf_mex_destroy(&c7_rhs107);
  sf_mex_destroy(&c7_lhs107);
  sf_mex_destroy(&c7_rhs108);
  sf_mex_destroy(&c7_lhs108);
  sf_mex_destroy(&c7_rhs109);
  sf_mex_destroy(&c7_lhs109);
  sf_mex_destroy(&c7_rhs110);
  sf_mex_destroy(&c7_lhs110);
  sf_mex_destroy(&c7_rhs111);
  sf_mex_destroy(&c7_lhs111);
  sf_mex_destroy(&c7_rhs112);
  sf_mex_destroy(&c7_lhs112);
  sf_mex_destroy(&c7_rhs113);
  sf_mex_destroy(&c7_lhs113);
  sf_mex_destroy(&c7_rhs114);
  sf_mex_destroy(&c7_lhs114);
  sf_mex_destroy(&c7_rhs115);
  sf_mex_destroy(&c7_lhs115);
  sf_mex_destroy(&c7_rhs116);
  sf_mex_destroy(&c7_lhs116);
  sf_mex_destroy(&c7_rhs117);
  sf_mex_destroy(&c7_lhs117);
  sf_mex_destroy(&c7_rhs118);
  sf_mex_destroy(&c7_lhs118);
  sf_mex_destroy(&c7_rhs119);
  sf_mex_destroy(&c7_lhs119);
  sf_mex_destroy(&c7_rhs120);
  sf_mex_destroy(&c7_lhs120);
  sf_mex_destroy(&c7_rhs121);
  sf_mex_destroy(&c7_lhs121);
  sf_mex_destroy(&c7_rhs122);
  sf_mex_destroy(&c7_lhs122);
  sf_mex_destroy(&c7_rhs123);
  sf_mex_destroy(&c7_lhs123);
  sf_mex_destroy(&c7_rhs124);
  sf_mex_destroy(&c7_lhs124);
  sf_mex_destroy(&c7_rhs125);
  sf_mex_destroy(&c7_lhs125);
  sf_mex_destroy(&c7_rhs126);
  sf_mex_destroy(&c7_lhs126);
  sf_mex_destroy(&c7_rhs127);
  sf_mex_destroy(&c7_lhs127);
}

static void c7_c_info_helper(const mxArray **c7_info)
{
  const mxArray *c7_rhs128 = NULL;
  const mxArray *c7_lhs128 = NULL;
  const mxArray *c7_rhs129 = NULL;
  const mxArray *c7_lhs129 = NULL;
  const mxArray *c7_rhs130 = NULL;
  const mxArray *c7_lhs130 = NULL;
  const mxArray *c7_rhs131 = NULL;
  const mxArray *c7_lhs131 = NULL;
  const mxArray *c7_rhs132 = NULL;
  const mxArray *c7_lhs132 = NULL;
  const mxArray *c7_rhs133 = NULL;
  const mxArray *c7_lhs133 = NULL;
  const mxArray *c7_rhs134 = NULL;
  const mxArray *c7_lhs134 = NULL;
  const mxArray *c7_rhs135 = NULL;
  const mxArray *c7_lhs135 = NULL;
  const mxArray *c7_rhs136 = NULL;
  const mxArray *c7_lhs136 = NULL;
  const mxArray *c7_rhs137 = NULL;
  const mxArray *c7_lhs137 = NULL;
  const mxArray *c7_rhs138 = NULL;
  const mxArray *c7_lhs138 = NULL;
  const mxArray *c7_rhs139 = NULL;
  const mxArray *c7_lhs139 = NULL;
  const mxArray *c7_rhs140 = NULL;
  const mxArray *c7_lhs140 = NULL;
  const mxArray *c7_rhs141 = NULL;
  const mxArray *c7_lhs141 = NULL;
  const mxArray *c7_rhs142 = NULL;
  const mxArray *c7_lhs142 = NULL;
  const mxArray *c7_rhs143 = NULL;
  const mxArray *c7_lhs143 = NULL;
  const mxArray *c7_rhs144 = NULL;
  const mxArray *c7_lhs144 = NULL;
  const mxArray *c7_rhs145 = NULL;
  const mxArray *c7_lhs145 = NULL;
  const mxArray *c7_rhs146 = NULL;
  const mxArray *c7_lhs146 = NULL;
  const mxArray *c7_rhs147 = NULL;
  const mxArray *c7_lhs147 = NULL;
  const mxArray *c7_rhs148 = NULL;
  const mxArray *c7_lhs148 = NULL;
  const mxArray *c7_rhs149 = NULL;
  const mxArray *c7_lhs149 = NULL;
  const mxArray *c7_rhs150 = NULL;
  const mxArray *c7_lhs150 = NULL;
  const mxArray *c7_rhs151 = NULL;
  const mxArray *c7_lhs151 = NULL;
  const mxArray *c7_rhs152 = NULL;
  const mxArray *c7_lhs152 = NULL;
  const mxArray *c7_rhs153 = NULL;
  const mxArray *c7_lhs153 = NULL;
  const mxArray *c7_rhs154 = NULL;
  const mxArray *c7_lhs154 = NULL;
  const mxArray *c7_rhs155 = NULL;
  const mxArray *c7_lhs155 = NULL;
  const mxArray *c7_rhs156 = NULL;
  const mxArray *c7_lhs156 = NULL;
  const mxArray *c7_rhs157 = NULL;
  const mxArray *c7_lhs157 = NULL;
  const mxArray *c7_rhs158 = NULL;
  const mxArray *c7_lhs158 = NULL;
  const mxArray *c7_rhs159 = NULL;
  const mxArray *c7_lhs159 = NULL;
  const mxArray *c7_rhs160 = NULL;
  const mxArray *c7_lhs160 = NULL;
  const mxArray *c7_rhs161 = NULL;
  const mxArray *c7_lhs161 = NULL;
  const mxArray *c7_rhs162 = NULL;
  const mxArray *c7_lhs162 = NULL;
  const mxArray *c7_rhs163 = NULL;
  const mxArray *c7_lhs163 = NULL;
  const mxArray *c7_rhs164 = NULL;
  const mxArray *c7_lhs164 = NULL;
  const mxArray *c7_rhs165 = NULL;
  const mxArray *c7_lhs165 = NULL;
  const mxArray *c7_rhs166 = NULL;
  const mxArray *c7_lhs166 = NULL;
  const mxArray *c7_rhs167 = NULL;
  const mxArray *c7_lhs167 = NULL;
  const mxArray *c7_rhs168 = NULL;
  const mxArray *c7_lhs168 = NULL;
  const mxArray *c7_rhs169 = NULL;
  const mxArray *c7_lhs169 = NULL;
  const mxArray *c7_rhs170 = NULL;
  const mxArray *c7_lhs170 = NULL;
  const mxArray *c7_rhs171 = NULL;
  const mxArray *c7_lhs171 = NULL;
  const mxArray *c7_rhs172 = NULL;
  const mxArray *c7_lhs172 = NULL;
  const mxArray *c7_rhs173 = NULL;
  const mxArray *c7_lhs173 = NULL;
  const mxArray *c7_rhs174 = NULL;
  const mxArray *c7_lhs174 = NULL;
  const mxArray *c7_rhs175 = NULL;
  const mxArray *c7_lhs175 = NULL;
  const mxArray *c7_rhs176 = NULL;
  const mxArray *c7_lhs176 = NULL;
  const mxArray *c7_rhs177 = NULL;
  const mxArray *c7_lhs177 = NULL;
  const mxArray *c7_rhs178 = NULL;
  const mxArray *c7_lhs178 = NULL;
  const mxArray *c7_rhs179 = NULL;
  const mxArray *c7_lhs179 = NULL;
  const mxArray *c7_rhs180 = NULL;
  const mxArray *c7_lhs180 = NULL;
  const mxArray *c7_rhs181 = NULL;
  const mxArray *c7_lhs181 = NULL;
  const mxArray *c7_rhs182 = NULL;
  const mxArray *c7_lhs182 = NULL;
  const mxArray *c7_rhs183 = NULL;
  const mxArray *c7_lhs183 = NULL;
  const mxArray *c7_rhs184 = NULL;
  const mxArray *c7_lhs184 = NULL;
  const mxArray *c7_rhs185 = NULL;
  const mxArray *c7_lhs185 = NULL;
  const mxArray *c7_rhs186 = NULL;
  const mxArray *c7_lhs186 = NULL;
  const mxArray *c7_rhs187 = NULL;
  const mxArray *c7_lhs187 = NULL;
  const mxArray *c7_rhs188 = NULL;
  const mxArray *c7_lhs188 = NULL;
  const mxArray *c7_rhs189 = NULL;
  const mxArray *c7_lhs189 = NULL;
  const mxArray *c7_rhs190 = NULL;
  const mxArray *c7_lhs190 = NULL;
  const mxArray *c7_rhs191 = NULL;
  const mxArray *c7_lhs191 = NULL;
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xswap.p"),
                  "context", "context", 128);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 128);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 128);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 128);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 128);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 128);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 128);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 128);
  sf_mex_assign(&c7_rhs128, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs128, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs128), "rhs", "rhs",
                  128);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs128), "lhs", "lhs",
                  128);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xswap.p!below_threshold"),
                  "context", "context", 129);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.blas.threshold"),
                  "name", "name", 129);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 129);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 129);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 129);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 129);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 129);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 129);
  sf_mex_assign(&c7_rhs129, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs129, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs129), "rhs", "rhs",
                  129);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs129), "lhs", "lhs",
                  129);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xswap.p"),
                  "context", "context", 130);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.refblas.xswap"),
                  "name", "name", 130);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 130);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xswap.p"),
                  "resolved", "resolved", 130);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 130);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 130);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 130);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 130);
  sf_mex_assign(&c7_rhs130, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs130, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs130), "rhs", "rhs",
                  130);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs130), "lhs", "lhs",
                  130);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xswap.p"),
                  "context", "context", 131);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("abs"), "name", "name", 131);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 131);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "resolved",
                  "resolved", 131);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363713852U), "fileTimeLo",
                  "fileTimeLo", 131);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 131);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 131);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 131);
  sf_mex_assign(&c7_rhs131, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs131, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs131), "rhs", "rhs",
                  131);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs131), "lhs", "lhs",
                  131);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 132);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 132);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 132);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 132);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 132);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 132);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 132);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 132);
  sf_mex_assign(&c7_rhs132, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs132, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs132), "rhs", "rhs",
                  132);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs132), "lhs", "lhs",
                  132);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 133);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_scalar_abs"), "name",
                  "name", 133);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 133);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m"),
                  "resolved", "resolved", 133);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1286818712U), "fileTimeLo",
                  "fileTimeLo", 133);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 133);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 133);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 133);
  sf_mex_assign(&c7_rhs133, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs133, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs133), "rhs", "rhs",
                  133);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs133), "lhs", "lhs",
                  133);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xswap.p"),
                  "context", "context", 134);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 134);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 134);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 134);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 134);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 134);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 134);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 134);
  sf_mex_assign(&c7_rhs134, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs134, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs134), "rhs", "rhs",
                  134);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs134), "lhs", "lhs",
                  134);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xswap.p"),
                  "context", "context", 135);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 135);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 135);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 135);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 135);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 135);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 135);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 135);
  sf_mex_assign(&c7_rhs135, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs135, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs135), "rhs", "rhs",
                  135);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs135), "lhs", "lhs",
                  135);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgeqp3.m"),
                  "context", "context", 136);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_matlab_zlarfg"), "name",
                  "name", 136);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 136);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarfg.m"),
                  "resolved", "resolved", 136);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1286818822U), "fileTimeLo",
                  "fileTimeLo", 136);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 136);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 136);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 136);
  sf_mex_assign(&c7_rhs136, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs136, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs136), "rhs", "rhs",
                  136);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs136), "lhs", "lhs",
                  136);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarfg.m"),
                  "context", "context", 137);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 137);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 137);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 137);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 137);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 137);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 137);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 137);
  sf_mex_assign(&c7_rhs137, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs137, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs137), "rhs", "rhs",
                  137);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs137), "lhs", "lhs",
                  137);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarfg.m"),
                  "context", "context", 138);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_xnrm2"), "name", "name",
                  138);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 138);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xnrm2.m"),
                  "resolved", "resolved", 138);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1375980692U), "fileTimeLo",
                  "fileTimeLo", 138);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 138);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 138);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 138);
  sf_mex_assign(&c7_rhs138, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs138, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs138), "rhs", "rhs",
                  138);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs138), "lhs", "lhs",
                  138);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xnrm2.p!below_threshold"),
                  "context", "context", 139);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("length"), "name", "name", 139);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 139);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m"), "resolved",
                  "resolved", 139);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1303146206U), "fileTimeLo",
                  "fileTimeLo", 139);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 139);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 139);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 139);
  sf_mex_assign(&c7_rhs139, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs139, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs139), "rhs", "rhs",
                  139);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs139), "lhs", "lhs",
                  139);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m!intlength"),
                  "context", "context", 140);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 140);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 140);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 140);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1323170578U), "fileTimeLo",
                  "fileTimeLo", 140);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 140);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 140);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 140);
  sf_mex_assign(&c7_rhs140, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs140, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs140), "rhs", "rhs",
                  140);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs140), "lhs", "lhs",
                  140);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarfg.m"),
                  "context", "context", 141);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_dlapy2"), "name", "name",
                  141);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 141);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_dlapy2.m"), "resolved",
                  "resolved", 141);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1350410654U), "fileTimeLo",
                  "fileTimeLo", 141);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 141);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 141);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 141);
  sf_mex_assign(&c7_rhs141, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs141, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs141), "rhs", "rhs",
                  141);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs141), "lhs", "lhs",
                  141);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarfg.m"),
                  "context", "context", 142);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("realmin"), "name", "name", 142);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 142);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmin.m"), "resolved",
                  "resolved", 142);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1307651242U), "fileTimeLo",
                  "fileTimeLo", 142);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 142);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 142);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 142);
  sf_mex_assign(&c7_rhs142, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs142, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs142), "rhs", "rhs",
                  142);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs142), "lhs", "lhs",
                  142);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarfg.m"),
                  "context", "context", 143);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eps"), "name", "name", 143);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 143);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m"), "resolved",
                  "resolved", 143);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1326727996U), "fileTimeLo",
                  "fileTimeLo", 143);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 143);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 143);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 143);
  sf_mex_assign(&c7_rhs143, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs143, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs143), "rhs", "rhs",
                  143);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs143), "lhs", "lhs",
                  143);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarfg.m"),
                  "context", "context", 144);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("abs"), "name", "name", 144);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 144);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "resolved",
                  "resolved", 144);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363713852U), "fileTimeLo",
                  "fileTimeLo", 144);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 144);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 144);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 144);
  sf_mex_assign(&c7_rhs144, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs144, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs144), "rhs", "rhs",
                  144);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs144), "lhs", "lhs",
                  144);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarfg.m"),
                  "context", "context", 145);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 145);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 145);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 145);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1323170578U), "fileTimeLo",
                  "fileTimeLo", 145);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 145);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 145);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 145);
  sf_mex_assign(&c7_rhs145, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs145, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs145), "rhs", "rhs",
                  145);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs145), "lhs", "lhs",
                  145);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarfg.m"),
                  "context", "context", 146);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 146);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 146);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 146);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1372582416U), "fileTimeLo",
                  "fileTimeLo", 146);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 146);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 146);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 146);
  sf_mex_assign(&c7_rhs146, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs146, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs146), "rhs", "rhs",
                  146);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs146), "lhs", "lhs",
                  146);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarfg.m"),
                  "context", "context", 147);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_xscal"), "name", "name",
                  147);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 147);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xscal.m"),
                  "resolved", "resolved", 147);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1375980692U), "fileTimeLo",
                  "fileTimeLo", 147);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 147);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 147);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 147);
  sf_mex_assign(&c7_rhs147, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs147, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs147), "rhs", "rhs",
                  147);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs147), "lhs", "lhs",
                  147);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xscal.m"), "context",
                  "context", 148);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 148);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 148);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 148);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 148);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 148);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 148);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 148);
  sf_mex_assign(&c7_rhs148, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs148, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs148), "rhs", "rhs",
                  148);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs148), "lhs", "lhs",
                  148);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xscal.m"), "context",
                  "context", 149);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.blas.xscal"),
                  "name", "name", 149);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 149);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xscal.p"),
                  "resolved", "resolved", 149);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 149);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 149);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 149);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 149);
  sf_mex_assign(&c7_rhs149, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs149, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs149), "rhs", "rhs",
                  149);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs149), "lhs", "lhs",
                  149);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xscal.p"),
                  "context", "context", 150);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 150);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 150);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 150);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 150);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 150);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 150);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 150);
  sf_mex_assign(&c7_rhs150, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs150, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs150), "rhs", "rhs",
                  150);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs150), "lhs", "lhs",
                  150);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xscal.p!below_threshold"),
                  "context", "context", 151);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.blas.threshold"),
                  "name", "name", 151);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 151);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 151);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 151);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 151);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 151);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 151);
  sf_mex_assign(&c7_rhs151, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs151, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs151), "rhs", "rhs",
                  151);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs151), "lhs", "lhs",
                  151);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xscal.p!below_threshold"),
                  "context", "context", 152);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("length"), "name", "name", 152);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 152);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m"), "resolved",
                  "resolved", 152);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1303146206U), "fileTimeLo",
                  "fileTimeLo", 152);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 152);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 152);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 152);
  sf_mex_assign(&c7_rhs152, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs152, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs152), "rhs", "rhs",
                  152);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs152), "lhs", "lhs",
                  152);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xscal.p"),
                  "context", "context", 153);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 153);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 153);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 153);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1389307920U), "fileTimeLo",
                  "fileTimeLo", 153);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 153);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 153);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 153);
  sf_mex_assign(&c7_rhs153, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs153, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs153), "rhs", "rhs",
                  153);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs153), "lhs", "lhs",
                  153);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xscal.p"),
                  "context", "context", 154);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.refblas.xscal"),
                  "name", "name", 154);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 154);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xscal.p"),
                  "resolved", "resolved", 154);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 154);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 154);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 154);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 154);
  sf_mex_assign(&c7_rhs154, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs154, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs154), "rhs", "rhs",
                  154);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs154), "lhs", "lhs",
                  154);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xscal.p"),
                  "context", "context", 155);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexMinus"),
                  "name", "name", 155);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 155);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexMinus.m"),
                  "resolved", "resolved", 155);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 155);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 155);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 155);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 155);
  sf_mex_assign(&c7_rhs155, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs155, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs155), "rhs", "rhs",
                  155);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs155), "lhs", "lhs",
                  155);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xscal.p"),
                  "context", "context", 156);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexTimes"),
                  "name", "name", 156);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 156);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexTimes.m"),
                  "resolved", "resolved", 156);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 156);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 156);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 156);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 156);
  sf_mex_assign(&c7_rhs156, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs156, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs156), "rhs", "rhs",
                  156);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs156), "lhs", "lhs",
                  156);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xscal.p"),
                  "context", "context", 157);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 157);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 157);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 157);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 157);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 157);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 157);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 157);
  sf_mex_assign(&c7_rhs157, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs157, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs157), "rhs", "rhs",
                  157);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs157), "lhs", "lhs",
                  157);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xscal.p"),
                  "context", "context", 158);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 158);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 158);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 158);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 158);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 158);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 158);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 158);
  sf_mex_assign(&c7_rhs158, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs158, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs158), "rhs", "rhs",
                  158);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs158), "lhs", "lhs",
                  158);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarfg.m"),
                  "context", "context", 159);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_div"), "name", "name", 159);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 159);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "resolved",
                  "resolved", 159);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 159);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 159);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 159);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 159);
  sf_mex_assign(&c7_rhs159, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs159, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs159), "rhs", "rhs",
                  159);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs159), "lhs", "lhs",
                  159);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarfg.m"),
                  "context", "context", 160);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 160);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 160);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 160);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 160);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 160);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 160);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 160);
  sf_mex_assign(&c7_rhs160, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs160, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs160), "rhs", "rhs",
                  160);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs160), "lhs", "lhs",
                  160);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgeqp3.m"),
                  "context", "context", 161);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_matlab_zlarf"), "name",
                  "name", 161);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 161);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarf.m"),
                  "resolved", "resolved", 161);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1286818822U), "fileTimeLo",
                  "fileTimeLo", 161);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 161);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 161);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 161);
  sf_mex_assign(&c7_rhs161, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs161, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs161), "rhs", "rhs",
                  161);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs161), "lhs", "lhs",
                  161);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarf.m"),
                  "context", "context", 162);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 162);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 162);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 162);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 162);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 162);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 162);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 162);
  sf_mex_assign(&c7_rhs162, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs162, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs162), "rhs", "rhs",
                  162);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs162), "lhs", "lhs",
                  162);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarf.m"),
                  "context", "context", 163);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 163);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 163);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 163);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1323170578U), "fileTimeLo",
                  "fileTimeLo", 163);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 163);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 163);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 163);
  sf_mex_assign(&c7_rhs163, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs163, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs163), "rhs", "rhs",
                  163);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs163), "lhs", "lhs",
                  163);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarf.m"),
                  "context", "context", 164);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("isequal"), "name", "name", 164);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 164);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isequal.m"), "resolved",
                  "resolved", 164);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1286818758U), "fileTimeLo",
                  "fileTimeLo", 164);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 164);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 164);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 164);
  sf_mex_assign(&c7_rhs164, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs164, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs164), "rhs", "rhs",
                  164);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs164), "lhs", "lhs",
                  164);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isequal.m"), "context",
                  "context", 165);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_isequal_core"), "name",
                  "name", 165);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 165);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_isequal_core.m"),
                  "resolved", "resolved", 165);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1286818786U), "fileTimeLo",
                  "fileTimeLo", 165);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 165);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 165);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 165);
  sf_mex_assign(&c7_rhs165, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs165, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs165), "rhs", "rhs",
                  165);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs165), "lhs", "lhs",
                  165);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarf.m"),
                  "context", "context", 166);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexIntRelop"),
                  "name", "name", 166);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 166);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m"),
                  "resolved", "resolved", 166);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1326728322U), "fileTimeLo",
                  "fileTimeLo", 166);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 166);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 166);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 166);
  sf_mex_assign(&c7_rhs166, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs166, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs166), "rhs", "rhs",
                  166);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs166), "lhs", "lhs",
                  166);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m!apply_float_relop"),
                  "context", "context", 167);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 167);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 167);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 167);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1381850300U), "fileTimeLo",
                  "fileTimeLo", 167);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 167);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 167);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 167);
  sf_mex_assign(&c7_rhs167, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs167, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs167), "rhs", "rhs",
                  167);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs167), "lhs", "lhs",
                  167);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m!float_class_contains_indexIntClass"),
                  "context", "context", 168);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_float_model"), "name",
                  "name", 168);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 168);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m"),
                  "resolved", "resolved", 168);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1326727996U), "fileTimeLo",
                  "fileTimeLo", 168);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 168);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 168);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 168);
  sf_mex_assign(&c7_rhs168, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs168, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs168), "rhs", "rhs",
                  168);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs168), "lhs", "lhs",
                  168);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m!is_signed_indexIntClass"),
                  "context", "context", 169);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("intmin"), "name", "name", 169);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 169);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m"), "resolved",
                  "resolved", 169);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1362261882U), "fileTimeLo",
                  "fileTimeLo", 169);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 169);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 169);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 169);
  sf_mex_assign(&c7_rhs169, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs169, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs169), "rhs", "rhs",
                  169);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs169), "lhs", "lhs",
                  169);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarf.m"),
                  "context", "context", 170);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_index_minus"), "name",
                  "name", 170);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 170);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m"),
                  "resolved", "resolved", 170);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1372582416U), "fileTimeLo",
                  "fileTimeLo", 170);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 170);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 170);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 170);
  sf_mex_assign(&c7_rhs170, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs170, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs170), "rhs", "rhs",
                  170);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs170), "lhs", "lhs",
                  170);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarf.m"),
                  "context", "context", 171);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_index_times"), "name",
                  "name", 171);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 171);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m"),
                  "resolved", "resolved", 171);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1372582416U), "fileTimeLo",
                  "fileTimeLo", 171);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 171);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 171);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 171);
  sf_mex_assign(&c7_rhs171, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs171, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs171), "rhs", "rhs",
                  171);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs171), "lhs", "lhs",
                  171);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m"),
                  "context", "context", 172);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexTimes"),
                  "name", "name", 172);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 172);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexTimes.m"),
                  "resolved", "resolved", 172);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 172);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 172);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 172);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 172);
  sf_mex_assign(&c7_rhs172, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs172, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs172), "rhs", "rhs",
                  172);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs172), "lhs", "lhs",
                  172);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarf.m"),
                  "context", "context", 173);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 173);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 173);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 173);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1372582416U), "fileTimeLo",
                  "fileTimeLo", 173);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 173);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 173);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 173);
  sf_mex_assign(&c7_rhs173, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs173, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs173), "rhs", "rhs",
                  173);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs173), "lhs", "lhs",
                  173);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarf.m!ilazlc"),
                  "context", "context", 174);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 174);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 174);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 174);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1323170578U), "fileTimeLo",
                  "fileTimeLo", 174);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 174);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 174);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 174);
  sf_mex_assign(&c7_rhs174, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs174, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs174), "rhs", "rhs",
                  174);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs174), "lhs", "lhs",
                  174);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarf.m!ilazlc"),
                  "context", "context", 175);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_index_minus"), "name",
                  "name", 175);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 175);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m"),
                  "resolved", "resolved", 175);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1372582416U), "fileTimeLo",
                  "fileTimeLo", 175);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 175);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 175);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 175);
  sf_mex_assign(&c7_rhs175, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs175, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs175), "rhs", "rhs",
                  175);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs175), "lhs", "lhs",
                  175);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarf.m!ilazlc"),
                  "context", "context", 176);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_index_times"), "name",
                  "name", 176);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 176);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m"),
                  "resolved", "resolved", 176);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1372582416U), "fileTimeLo",
                  "fileTimeLo", 176);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 176);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 176);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 176);
  sf_mex_assign(&c7_rhs176, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs176, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs176), "rhs", "rhs",
                  176);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs176), "lhs", "lhs",
                  176);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarf.m!ilazlc"),
                  "context", "context", 177);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 177);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 177);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 177);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1372582416U), "fileTimeLo",
                  "fileTimeLo", 177);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 177);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 177);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 177);
  sf_mex_assign(&c7_rhs177, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs177, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs177), "rhs", "rhs",
                  177);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs177), "lhs", "lhs",
                  177);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarf.m!ilazlc"),
                  "context", "context", 178);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 178);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 178);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 178);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 178);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 178);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 178);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 178);
  sf_mex_assign(&c7_rhs178, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs178, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs178), "rhs", "rhs",
                  178);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs178), "lhs", "lhs",
                  178);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarf.m"),
                  "context", "context", 179);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_xgemv"), "name", "name",
                  179);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 179);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemv.m"),
                  "resolved", "resolved", 179);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1375980690U), "fileTimeLo",
                  "fileTimeLo", 179);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 179);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 179);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 179);
  sf_mex_assign(&c7_rhs179, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs179, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs179), "rhs", "rhs",
                  179);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs179), "lhs", "lhs",
                  179);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemv.m"), "context",
                  "context", 180);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 180);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 180);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 180);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 180);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 180);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 180);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 180);
  sf_mex_assign(&c7_rhs180, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs180, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs180), "rhs", "rhs",
                  180);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs180), "lhs", "lhs",
                  180);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemv.m"), "context",
                  "context", 181);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.blas.xgemv"),
                  "name", "name", 181);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 181);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemv.p"),
                  "resolved", "resolved", 181);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 181);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 181);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 181);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 181);
  sf_mex_assign(&c7_rhs181, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs181, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs181), "rhs", "rhs",
                  181);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs181), "lhs", "lhs",
                  181);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemv.p"),
                  "context", "context", 182);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 182);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 182);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 182);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 182);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 182);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 182);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 182);
  sf_mex_assign(&c7_rhs182, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs182, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs182), "rhs", "rhs",
                  182);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs182), "lhs", "lhs",
                  182);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemv.p!below_threshold"),
                  "context", "context", 183);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.blas.threshold"),
                  "name", "name", 183);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 183);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 183);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 183);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 183);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 183);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 183);
  sf_mex_assign(&c7_rhs183, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs183, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs183), "rhs", "rhs",
                  183);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs183), "lhs", "lhs",
                  183);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemv.p!below_threshold"),
                  "context", "context", 184);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("length"), "name", "name", 184);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 184);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m"), "resolved",
                  "resolved", 184);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1303146206U), "fileTimeLo",
                  "fileTimeLo", 184);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 184);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 184);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 184);
  sf_mex_assign(&c7_rhs184, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs184, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs184), "rhs", "rhs",
                  184);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs184), "lhs", "lhs",
                  184);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemv.p!below_threshold"),
                  "context", "context", 185);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("min"), "name", "name", 185);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 185);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m"), "resolved",
                  "resolved", 185);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1311255318U), "fileTimeLo",
                  "fileTimeLo", 185);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 185);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 185);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 185);
  sf_mex_assign(&c7_rhs185, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs185, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs185), "rhs", "rhs",
                  185);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs185), "lhs", "lhs",
                  185);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemv.p"),
                  "context", "context", 186);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 186);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 186);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 186);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1389307920U), "fileTimeLo",
                  "fileTimeLo", 186);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 186);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 186);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 186);
  sf_mex_assign(&c7_rhs186, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs186, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs186), "rhs", "rhs",
                  186);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs186), "lhs", "lhs",
                  186);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemv.p"),
                  "context", "context", 187);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.refblas.xgemv"),
                  "name", "name", 187);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 187);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgemv.p"),
                  "resolved", "resolved", 187);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 187);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 187);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 187);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 187);
  sf_mex_assign(&c7_rhs187, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs187, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs187), "rhs", "rhs",
                  187);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs187), "lhs", "lhs",
                  187);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgemv.p"),
                  "context", "context", 188);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexMinus"),
                  "name", "name", 188);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 188);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexMinus.m"),
                  "resolved", "resolved", 188);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 188);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 188);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 188);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 188);
  sf_mex_assign(&c7_rhs188, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs188, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs188), "rhs", "rhs",
                  188);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs188), "lhs", "lhs",
                  188);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgemv.p"),
                  "context", "context", 189);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexTimes"),
                  "name", "name", 189);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 189);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexTimes.m"),
                  "resolved", "resolved", 189);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 189);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 189);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 189);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 189);
  sf_mex_assign(&c7_rhs189, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs189, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs189), "rhs", "rhs",
                  189);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs189), "lhs", "lhs",
                  189);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgemv.p"),
                  "context", "context", 190);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 190);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 190);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 190);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 190);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 190);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 190);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 190);
  sf_mex_assign(&c7_rhs190, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs190, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs190), "rhs", "rhs",
                  190);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs190), "lhs", "lhs",
                  190);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgemv.p"),
                  "context", "context", 191);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 191);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 191);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 191);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 191);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 191);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 191);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 191);
  sf_mex_assign(&c7_rhs191, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs191, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs191), "rhs", "rhs",
                  191);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs191), "lhs", "lhs",
                  191);
  sf_mex_destroy(&c7_rhs128);
  sf_mex_destroy(&c7_lhs128);
  sf_mex_destroy(&c7_rhs129);
  sf_mex_destroy(&c7_lhs129);
  sf_mex_destroy(&c7_rhs130);
  sf_mex_destroy(&c7_lhs130);
  sf_mex_destroy(&c7_rhs131);
  sf_mex_destroy(&c7_lhs131);
  sf_mex_destroy(&c7_rhs132);
  sf_mex_destroy(&c7_lhs132);
  sf_mex_destroy(&c7_rhs133);
  sf_mex_destroy(&c7_lhs133);
  sf_mex_destroy(&c7_rhs134);
  sf_mex_destroy(&c7_lhs134);
  sf_mex_destroy(&c7_rhs135);
  sf_mex_destroy(&c7_lhs135);
  sf_mex_destroy(&c7_rhs136);
  sf_mex_destroy(&c7_lhs136);
  sf_mex_destroy(&c7_rhs137);
  sf_mex_destroy(&c7_lhs137);
  sf_mex_destroy(&c7_rhs138);
  sf_mex_destroy(&c7_lhs138);
  sf_mex_destroy(&c7_rhs139);
  sf_mex_destroy(&c7_lhs139);
  sf_mex_destroy(&c7_rhs140);
  sf_mex_destroy(&c7_lhs140);
  sf_mex_destroy(&c7_rhs141);
  sf_mex_destroy(&c7_lhs141);
  sf_mex_destroy(&c7_rhs142);
  sf_mex_destroy(&c7_lhs142);
  sf_mex_destroy(&c7_rhs143);
  sf_mex_destroy(&c7_lhs143);
  sf_mex_destroy(&c7_rhs144);
  sf_mex_destroy(&c7_lhs144);
  sf_mex_destroy(&c7_rhs145);
  sf_mex_destroy(&c7_lhs145);
  sf_mex_destroy(&c7_rhs146);
  sf_mex_destroy(&c7_lhs146);
  sf_mex_destroy(&c7_rhs147);
  sf_mex_destroy(&c7_lhs147);
  sf_mex_destroy(&c7_rhs148);
  sf_mex_destroy(&c7_lhs148);
  sf_mex_destroy(&c7_rhs149);
  sf_mex_destroy(&c7_lhs149);
  sf_mex_destroy(&c7_rhs150);
  sf_mex_destroy(&c7_lhs150);
  sf_mex_destroy(&c7_rhs151);
  sf_mex_destroy(&c7_lhs151);
  sf_mex_destroy(&c7_rhs152);
  sf_mex_destroy(&c7_lhs152);
  sf_mex_destroy(&c7_rhs153);
  sf_mex_destroy(&c7_lhs153);
  sf_mex_destroy(&c7_rhs154);
  sf_mex_destroy(&c7_lhs154);
  sf_mex_destroy(&c7_rhs155);
  sf_mex_destroy(&c7_lhs155);
  sf_mex_destroy(&c7_rhs156);
  sf_mex_destroy(&c7_lhs156);
  sf_mex_destroy(&c7_rhs157);
  sf_mex_destroy(&c7_lhs157);
  sf_mex_destroy(&c7_rhs158);
  sf_mex_destroy(&c7_lhs158);
  sf_mex_destroy(&c7_rhs159);
  sf_mex_destroy(&c7_lhs159);
  sf_mex_destroy(&c7_rhs160);
  sf_mex_destroy(&c7_lhs160);
  sf_mex_destroy(&c7_rhs161);
  sf_mex_destroy(&c7_lhs161);
  sf_mex_destroy(&c7_rhs162);
  sf_mex_destroy(&c7_lhs162);
  sf_mex_destroy(&c7_rhs163);
  sf_mex_destroy(&c7_lhs163);
  sf_mex_destroy(&c7_rhs164);
  sf_mex_destroy(&c7_lhs164);
  sf_mex_destroy(&c7_rhs165);
  sf_mex_destroy(&c7_lhs165);
  sf_mex_destroy(&c7_rhs166);
  sf_mex_destroy(&c7_lhs166);
  sf_mex_destroy(&c7_rhs167);
  sf_mex_destroy(&c7_lhs167);
  sf_mex_destroy(&c7_rhs168);
  sf_mex_destroy(&c7_lhs168);
  sf_mex_destroy(&c7_rhs169);
  sf_mex_destroy(&c7_lhs169);
  sf_mex_destroy(&c7_rhs170);
  sf_mex_destroy(&c7_lhs170);
  sf_mex_destroy(&c7_rhs171);
  sf_mex_destroy(&c7_lhs171);
  sf_mex_destroy(&c7_rhs172);
  sf_mex_destroy(&c7_lhs172);
  sf_mex_destroy(&c7_rhs173);
  sf_mex_destroy(&c7_lhs173);
  sf_mex_destroy(&c7_rhs174);
  sf_mex_destroy(&c7_lhs174);
  sf_mex_destroy(&c7_rhs175);
  sf_mex_destroy(&c7_lhs175);
  sf_mex_destroy(&c7_rhs176);
  sf_mex_destroy(&c7_lhs176);
  sf_mex_destroy(&c7_rhs177);
  sf_mex_destroy(&c7_lhs177);
  sf_mex_destroy(&c7_rhs178);
  sf_mex_destroy(&c7_lhs178);
  sf_mex_destroy(&c7_rhs179);
  sf_mex_destroy(&c7_lhs179);
  sf_mex_destroy(&c7_rhs180);
  sf_mex_destroy(&c7_lhs180);
  sf_mex_destroy(&c7_rhs181);
  sf_mex_destroy(&c7_lhs181);
  sf_mex_destroy(&c7_rhs182);
  sf_mex_destroy(&c7_lhs182);
  sf_mex_destroy(&c7_rhs183);
  sf_mex_destroy(&c7_lhs183);
  sf_mex_destroy(&c7_rhs184);
  sf_mex_destroy(&c7_lhs184);
  sf_mex_destroy(&c7_rhs185);
  sf_mex_destroy(&c7_lhs185);
  sf_mex_destroy(&c7_rhs186);
  sf_mex_destroy(&c7_lhs186);
  sf_mex_destroy(&c7_rhs187);
  sf_mex_destroy(&c7_lhs187);
  sf_mex_destroy(&c7_rhs188);
  sf_mex_destroy(&c7_lhs188);
  sf_mex_destroy(&c7_rhs189);
  sf_mex_destroy(&c7_lhs189);
  sf_mex_destroy(&c7_rhs190);
  sf_mex_destroy(&c7_lhs190);
  sf_mex_destroy(&c7_rhs191);
  sf_mex_destroy(&c7_lhs191);
}

static void c7_d_info_helper(const mxArray **c7_info)
{
  const mxArray *c7_rhs192 = NULL;
  const mxArray *c7_lhs192 = NULL;
  const mxArray *c7_rhs193 = NULL;
  const mxArray *c7_lhs193 = NULL;
  const mxArray *c7_rhs194 = NULL;
  const mxArray *c7_lhs194 = NULL;
  const mxArray *c7_rhs195 = NULL;
  const mxArray *c7_lhs195 = NULL;
  const mxArray *c7_rhs196 = NULL;
  const mxArray *c7_lhs196 = NULL;
  const mxArray *c7_rhs197 = NULL;
  const mxArray *c7_lhs197 = NULL;
  const mxArray *c7_rhs198 = NULL;
  const mxArray *c7_lhs198 = NULL;
  const mxArray *c7_rhs199 = NULL;
  const mxArray *c7_lhs199 = NULL;
  const mxArray *c7_rhs200 = NULL;
  const mxArray *c7_lhs200 = NULL;
  const mxArray *c7_rhs201 = NULL;
  const mxArray *c7_lhs201 = NULL;
  const mxArray *c7_rhs202 = NULL;
  const mxArray *c7_lhs202 = NULL;
  const mxArray *c7_rhs203 = NULL;
  const mxArray *c7_lhs203 = NULL;
  const mxArray *c7_rhs204 = NULL;
  const mxArray *c7_lhs204 = NULL;
  const mxArray *c7_rhs205 = NULL;
  const mxArray *c7_lhs205 = NULL;
  const mxArray *c7_rhs206 = NULL;
  const mxArray *c7_lhs206 = NULL;
  const mxArray *c7_rhs207 = NULL;
  const mxArray *c7_lhs207 = NULL;
  const mxArray *c7_rhs208 = NULL;
  const mxArray *c7_lhs208 = NULL;
  const mxArray *c7_rhs209 = NULL;
  const mxArray *c7_lhs209 = NULL;
  const mxArray *c7_rhs210 = NULL;
  const mxArray *c7_lhs210 = NULL;
  const mxArray *c7_rhs211 = NULL;
  const mxArray *c7_lhs211 = NULL;
  const mxArray *c7_rhs212 = NULL;
  const mxArray *c7_lhs212 = NULL;
  const mxArray *c7_rhs213 = NULL;
  const mxArray *c7_lhs213 = NULL;
  const mxArray *c7_rhs214 = NULL;
  const mxArray *c7_lhs214 = NULL;
  const mxArray *c7_rhs215 = NULL;
  const mxArray *c7_lhs215 = NULL;
  const mxArray *c7_rhs216 = NULL;
  const mxArray *c7_lhs216 = NULL;
  const mxArray *c7_rhs217 = NULL;
  const mxArray *c7_lhs217 = NULL;
  const mxArray *c7_rhs218 = NULL;
  const mxArray *c7_lhs218 = NULL;
  const mxArray *c7_rhs219 = NULL;
  const mxArray *c7_lhs219 = NULL;
  const mxArray *c7_rhs220 = NULL;
  const mxArray *c7_lhs220 = NULL;
  const mxArray *c7_rhs221 = NULL;
  const mxArray *c7_lhs221 = NULL;
  const mxArray *c7_rhs222 = NULL;
  const mxArray *c7_lhs222 = NULL;
  const mxArray *c7_rhs223 = NULL;
  const mxArray *c7_lhs223 = NULL;
  const mxArray *c7_rhs224 = NULL;
  const mxArray *c7_lhs224 = NULL;
  const mxArray *c7_rhs225 = NULL;
  const mxArray *c7_lhs225 = NULL;
  const mxArray *c7_rhs226 = NULL;
  const mxArray *c7_lhs226 = NULL;
  const mxArray *c7_rhs227 = NULL;
  const mxArray *c7_lhs227 = NULL;
  const mxArray *c7_rhs228 = NULL;
  const mxArray *c7_lhs228 = NULL;
  const mxArray *c7_rhs229 = NULL;
  const mxArray *c7_lhs229 = NULL;
  const mxArray *c7_rhs230 = NULL;
  const mxArray *c7_lhs230 = NULL;
  const mxArray *c7_rhs231 = NULL;
  const mxArray *c7_lhs231 = NULL;
  const mxArray *c7_rhs232 = NULL;
  const mxArray *c7_lhs232 = NULL;
  const mxArray *c7_rhs233 = NULL;
  const mxArray *c7_lhs233 = NULL;
  const mxArray *c7_rhs234 = NULL;
  const mxArray *c7_lhs234 = NULL;
  const mxArray *c7_rhs235 = NULL;
  const mxArray *c7_lhs235 = NULL;
  const mxArray *c7_rhs236 = NULL;
  const mxArray *c7_lhs236 = NULL;
  const mxArray *c7_rhs237 = NULL;
  const mxArray *c7_lhs237 = NULL;
  const mxArray *c7_rhs238 = NULL;
  const mxArray *c7_lhs238 = NULL;
  const mxArray *c7_rhs239 = NULL;
  const mxArray *c7_lhs239 = NULL;
  const mxArray *c7_rhs240 = NULL;
  const mxArray *c7_lhs240 = NULL;
  const mxArray *c7_rhs241 = NULL;
  const mxArray *c7_lhs241 = NULL;
  const mxArray *c7_rhs242 = NULL;
  const mxArray *c7_lhs242 = NULL;
  const mxArray *c7_rhs243 = NULL;
  const mxArray *c7_lhs243 = NULL;
  const mxArray *c7_rhs244 = NULL;
  const mxArray *c7_lhs244 = NULL;
  const mxArray *c7_rhs245 = NULL;
  const mxArray *c7_lhs245 = NULL;
  const mxArray *c7_rhs246 = NULL;
  const mxArray *c7_lhs246 = NULL;
  const mxArray *c7_rhs247 = NULL;
  const mxArray *c7_lhs247 = NULL;
  const mxArray *c7_rhs248 = NULL;
  const mxArray *c7_lhs248 = NULL;
  const mxArray *c7_rhs249 = NULL;
  const mxArray *c7_lhs249 = NULL;
  const mxArray *c7_rhs250 = NULL;
  const mxArray *c7_lhs250 = NULL;
  const mxArray *c7_rhs251 = NULL;
  const mxArray *c7_lhs251 = NULL;
  const mxArray *c7_rhs252 = NULL;
  const mxArray *c7_lhs252 = NULL;
  const mxArray *c7_rhs253 = NULL;
  const mxArray *c7_lhs253 = NULL;
  const mxArray *c7_rhs254 = NULL;
  const mxArray *c7_lhs254 = NULL;
  const mxArray *c7_rhs255 = NULL;
  const mxArray *c7_lhs255 = NULL;
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgemv.p"),
                  "context", "context", 192);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 192);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 192);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 192);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1389307920U), "fileTimeLo",
                  "fileTimeLo", 192);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 192);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 192);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 192);
  sf_mex_assign(&c7_rhs192, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs192, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs192), "rhs", "rhs",
                  192);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs192), "lhs", "lhs",
                  192);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgemv.p"),
                  "context", "context", 193);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.conjtimes"),
                  "name", "name", 193);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 193);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/conjtimes.m"),
                  "resolved", "resolved", 193);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1360282586U), "fileTimeLo",
                  "fileTimeLo", 193);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 193);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 193);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 193);
  sf_mex_assign(&c7_rhs193, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs193, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs193), "rhs", "rhs",
                  193);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs193), "lhs", "lhs",
                  193);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zlarf.m"),
                  "context", "context", 194);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_xgerc"), "name", "name",
                  194);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 194);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgerc.m"),
                  "resolved", "resolved", 194);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1375980690U), "fileTimeLo",
                  "fileTimeLo", 194);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 194);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 194);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 194);
  sf_mex_assign(&c7_rhs194, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs194, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs194), "rhs", "rhs",
                  194);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs194), "lhs", "lhs",
                  194);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgerc.m"), "context",
                  "context", 195);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 195);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 195);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 195);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 195);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 195);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 195);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 195);
  sf_mex_assign(&c7_rhs195, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs195, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs195), "rhs", "rhs",
                  195);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs195), "lhs", "lhs",
                  195);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgerc.m"), "context",
                  "context", 196);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.blas.xgerc"),
                  "name", "name", 196);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 196);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgerc.p"),
                  "resolved", "resolved", 196);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 196);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 196);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 196);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 196);
  sf_mex_assign(&c7_rhs196, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs196, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs196), "rhs", "rhs",
                  196);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs196), "lhs", "lhs",
                  196);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgerc.p"),
                  "context", "context", 197);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.blas.xger"),
                  "name", "name", 197);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 197);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xger.p"),
                  "resolved", "resolved", 197);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 197);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 197);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 197);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 197);
  sf_mex_assign(&c7_rhs197, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs197, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs197), "rhs", "rhs",
                  197);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs197), "lhs", "lhs",
                  197);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xger.p"),
                  "context", "context", 198);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 198);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 198);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 198);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 198);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 198);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 198);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 198);
  sf_mex_assign(&c7_rhs198, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs198, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs198), "rhs", "rhs",
                  198);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs198), "lhs", "lhs",
                  198);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xger.p!below_threshold"),
                  "context", "context", 199);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.blas.threshold"),
                  "name", "name", 199);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 199);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 199);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 199);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 199);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 199);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 199);
  sf_mex_assign(&c7_rhs199, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs199, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs199), "rhs", "rhs",
                  199);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs199), "lhs", "lhs",
                  199);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xger.p!below_threshold"),
                  "context", "context", 200);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.blas.int"),
                  "name", "name", 200);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 200);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/int.p"),
                  "resolved", "resolved", 200);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 200);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 200);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 200);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 200);
  sf_mex_assign(&c7_rhs200, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs200, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs200), "rhs", "rhs",
                  200);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs200), "lhs", "lhs",
                  200);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xger.p!below_threshold"),
                  "context", "context", 201);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("intmax"), "name", "name", 201);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 201);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "resolved",
                  "resolved", 201);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1362261882U), "fileTimeLo",
                  "fileTimeLo", 201);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 201);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 201);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 201);
  sf_mex_assign(&c7_rhs201, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs201, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs201), "rhs", "rhs",
                  201);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs201), "lhs", "lhs",
                  201);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xger.p!below_threshold"),
                  "context", "context", 202);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("min"), "name", "name", 202);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 202);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m"), "resolved",
                  "resolved", 202);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1311255318U), "fileTimeLo",
                  "fileTimeLo", 202);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 202);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 202);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 202);
  sf_mex_assign(&c7_rhs202, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs202, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs202), "rhs", "rhs",
                  202);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs202), "lhs", "lhs",
                  202);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xger.p!below_threshold"),
                  "context", "context", 203);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("length"), "name", "name", 203);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 203);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m"), "resolved",
                  "resolved", 203);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1303146206U), "fileTimeLo",
                  "fileTimeLo", 203);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 203);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 203);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 203);
  sf_mex_assign(&c7_rhs203, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs203, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs203), "rhs", "rhs",
                  203);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs203), "lhs", "lhs",
                  203);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xger.p"),
                  "context", "context", 204);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.refblas.xger"),
                  "name", "name", 204);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 204);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xger.p"),
                  "resolved", "resolved", 204);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 204);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 204);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 204);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 204);
  sf_mex_assign(&c7_rhs204, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs204, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs204), "rhs", "rhs",
                  204);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs204), "lhs", "lhs",
                  204);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xger.p"),
                  "context", "context", 205);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.refblas.xgerx"),
                  "name", "name", 205);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 205);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgerx.p"),
                  "resolved", "resolved", 205);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 205);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 205);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 205);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 205);
  sf_mex_assign(&c7_rhs205, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs205, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs205), "rhs", "rhs",
                  205);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs205), "lhs", "lhs",
                  205);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgerx.p"),
                  "context", "context", 206);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("abs"), "name", "name", 206);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 206);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "resolved",
                  "resolved", 206);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363713852U), "fileTimeLo",
                  "fileTimeLo", 206);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 206);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 206);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 206);
  sf_mex_assign(&c7_rhs206, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs206, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs206), "rhs", "rhs",
                  206);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs206), "lhs", "lhs",
                  206);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgerx.p"),
                  "context", "context", 207);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexMinus"),
                  "name", "name", 207);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 207);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexMinus.m"),
                  "resolved", "resolved", 207);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 207);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 207);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 207);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 207);
  sf_mex_assign(&c7_rhs207, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs207, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs207), "rhs", "rhs",
                  207);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs207), "lhs", "lhs",
                  207);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgerx.p"),
                  "context", "context", 208);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 208);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 208);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 208);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 208);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 208);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 208);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 208);
  sf_mex_assign(&c7_rhs208, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs208, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs208), "rhs", "rhs",
                  208);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs208), "lhs", "lhs",
                  208);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgerx.p"),
                  "context", "context", 209);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 209);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 209);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 209);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 209);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 209);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 209);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 209);
  sf_mex_assign(&c7_rhs209, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs209, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs209), "rhs", "rhs",
                  209);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs209), "lhs", "lhs",
                  209);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgerx.p"),
                  "context", "context", 210);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 210);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 210);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 210);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 210);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 210);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 210);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 210);
  sf_mex_assign(&c7_rhs210, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs210, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs210), "rhs", "rhs",
                  210);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs210), "lhs", "lhs",
                  210);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/lapack/matlab/eml_matlab_zgeqp3.m"),
                  "context", "context", 211);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("abs"), "name", "name", 211);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 211);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "resolved",
                  "resolved", 211);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363713852U), "fileTimeLo",
                  "fileTimeLo", 211);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 211);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 211);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 211);
  sf_mex_assign(&c7_rhs211, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs211, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs211), "rhs", "rhs",
                  211);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs211), "lhs", "lhs",
                  211);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_qrsolve.m"), "context",
                  "context", 212);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("max"), "name", "name", 212);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 212);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/max.m"), "resolved",
                  "resolved", 212);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1311255316U), "fileTimeLo",
                  "fileTimeLo", 212);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 212);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 212);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 212);
  sf_mex_assign(&c7_rhs212, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs212, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs212), "rhs", "rhs",
                  212);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs212), "lhs", "lhs",
                  212);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/max.m"), "context",
                  "context", 213);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_min_or_max"), "name",
                  "name", 213);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 213);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"),
                  "resolved", "resolved", 213);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1378295984U), "fileTimeLo",
                  "fileTimeLo", 213);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 213);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 213);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 213);
  sf_mex_assign(&c7_rhs213, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs213, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs213), "rhs", "rhs",
                  213);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs213), "lhs", "lhs",
                  213);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_qrsolve.m"), "context",
                  "context", 214);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_xcabs1"), "name", "name",
                  214);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 214);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xcabs1.m"),
                  "resolved", "resolved", 214);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 214);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 214);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 214);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 214);
  sf_mex_assign(&c7_rhs214, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs214, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs214), "rhs", "rhs",
                  214);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs214), "lhs", "lhs",
                  214);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xcabs1.m"),
                  "context", "context", 215);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.refblas.xcabs1"),
                  "name", "name", 215);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 215);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xcabs1.p"),
                  "resolved", "resolved", 215);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1389307922U), "fileTimeLo",
                  "fileTimeLo", 215);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 215);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 215);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 215);
  sf_mex_assign(&c7_rhs215, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs215, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs215), "rhs", "rhs",
                  215);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs215), "lhs", "lhs",
                  215);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_qrsolve.m"), "context",
                  "context", 216);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eps"), "name", "name", 216);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 216);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m"), "resolved",
                  "resolved", 216);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1326727996U), "fileTimeLo",
                  "fileTimeLo", 216);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 216);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 216);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 216);
  sf_mex_assign(&c7_rhs216, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs216, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs216), "rhs", "rhs",
                  216);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs216), "lhs", "lhs",
                  216);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_qrsolve.m"), "context",
                  "context", 217);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 217);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 217);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 217);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 217);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 217);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 217);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 217);
  sf_mex_assign(&c7_rhs217, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs217, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs217), "rhs", "rhs",
                  217);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs217), "lhs", "lhs",
                  217);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_qrsolve.m"), "context",
                  "context", 218);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.conjtimes"),
                  "name", "name", 218);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 218);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/conjtimes.m"),
                  "resolved", "resolved", 218);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1360282586U), "fileTimeLo",
                  "fileTimeLo", 218);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 218);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 218);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 218);
  sf_mex_assign(&c7_rhs218, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs218, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs218), "rhs", "rhs",
                  218);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs218), "lhs", "lhs",
                  218);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_qrsolve.m"), "context",
                  "context", 219);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_div"), "name", "name", 219);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 219);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "resolved",
                  "resolved", 219);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 219);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 219);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 219);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 219);
  sf_mex_assign(&c7_rhs219, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs219, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs219), "rhs", "rhs",
                  219);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs219), "lhs", "lhs",
                  219);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/polyfun/polyfit.m"), "context",
                  "context", 220);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_warning"), "name", "name",
                  220);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 220);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_warning.m"), "resolved",
                  "resolved", 220);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1286818802U), "fileTimeLo",
                  "fileTimeLo", 220);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 220);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 220);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 220);
  sf_mex_assign(&c7_rhs220, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs220, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs220), "rhs", "rhs",
                  220);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs220), "lhs", "lhs",
                  220);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "context", "context", 221);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("polyval"), "name", "name", 221);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 221);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/polyfun/polyval.m"), "resolved",
                  "resolved", 221);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1305318002U), "fileTimeLo",
                  "fileTimeLo", 221);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 221);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 221);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 221);
  sf_mex_assign(&c7_rhs221, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs221, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs221), "rhs", "rhs",
                  221);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs221), "lhs", "lhs",
                  221);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/polyfun/polyval.m"), "context",
                  "context", 222);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 222);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 222);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 222);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 222);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 222);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 222);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 222);
  sf_mex_assign(&c7_rhs222, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs222, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs222), "rhs", "rhs",
                  222);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs222), "lhs", "lhs",
                  222);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "context",
                  "context", 223);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 223);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 223);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 223);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1389307920U), "fileTimeLo",
                  "fileTimeLo", 223);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 223);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 223);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 223);
  sf_mex_assign(&c7_rhs223, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs223, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs223), "rhs", "rhs",
                  223);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs223), "lhs", "lhs",
                  223);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "context", "context", 224);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("power"), "name", "name", 224);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 224);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m"), "resolved",
                  "resolved", 224);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363713880U), "fileTimeLo",
                  "fileTimeLo", 224);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 224);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 224);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 224);
  sf_mex_assign(&c7_rhs224, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs224, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs224), "rhs", "rhs",
                  224);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs224), "lhs", "lhs",
                  224);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m"), "context",
                  "context", 225);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 225);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 225);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 225);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 225);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 225);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 225);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 225);
  sf_mex_assign(&c7_rhs225, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs225, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs225), "rhs", "rhs",
                  225);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs225), "lhs", "lhs",
                  225);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m"), "context",
                  "context", 226);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 226);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 226);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 226);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 226);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 226);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 226);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 226);
  sf_mex_assign(&c7_rhs226, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs226, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs226), "rhs", "rhs",
                  226);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs226), "lhs", "lhs",
                  226);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower"), "context",
                  "context", 227);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 227);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 227);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 227);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 227);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 227);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 227);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 227);
  sf_mex_assign(&c7_rhs227, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs227, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs227), "rhs", "rhs",
                  227);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs227), "lhs", "lhs",
                  227);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower"), "context",
                  "context", 228);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_scalexp_alloc"), "name",
                  "name", 228);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 228);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "resolved", "resolved", 228);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 228);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 228);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 228);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 228);
  sf_mex_assign(&c7_rhs228, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs228, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs228), "rhs", "rhs",
                  228);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs228), "lhs", "lhs",
                  228);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "context", "context", 229);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.scalexpAlloc"),
                  "name", "name", 229);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 229);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalexpAlloc.p"),
                  "resolved", "resolved", 229);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1389307920U), "fileTimeLo",
                  "fileTimeLo", 229);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 229);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 229);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 229);
  sf_mex_assign(&c7_rhs229, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs229, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs229), "rhs", "rhs",
                  229);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs229), "lhs", "lhs",
                  229);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower"), "context",
                  "context", 230);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("floor"), "name", "name", 230);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 230);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "resolved",
                  "resolved", 230);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363713854U), "fileTimeLo",
                  "fileTimeLo", 230);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 230);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 230);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 230);
  sf_mex_assign(&c7_rhs230, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs230, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs230), "rhs", "rhs",
                  230);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs230), "lhs", "lhs",
                  230);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!scalar_float_power"),
                  "context", "context", 231);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 231);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 231);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 231);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 231);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 231);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 231);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 231);
  sf_mex_assign(&c7_rhs231, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs231, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs231), "rhs", "rhs",
                  231);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs231), "lhs", "lhs",
                  231);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "context", "context", 232);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("sqrt"), "name", "name", 232);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 232);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "resolved",
                  "resolved", 232);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1343830386U), "fileTimeLo",
                  "fileTimeLo", 232);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 232);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 232);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 232);
  sf_mex_assign(&c7_rhs232, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs232, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs232), "rhs", "rhs",
                  232);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs232), "lhs", "lhs",
                  232);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "context",
                  "context", 233);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_scalar_sqrt"), "name",
                  "name", 233);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 233);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sqrt.m"),
                  "resolved", "resolved", 233);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1286818738U), "fileTimeLo",
                  "fileTimeLo", 233);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 233);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 233);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 233);
  sf_mex_assign(&c7_rhs233, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs233, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs233), "rhs", "rhs",
                  233);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs233), "lhs", "lhs",
                  233);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "context", "context", 234);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("atan2d"), "name", "name", 234);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 234);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan2d.m"), "resolved",
                  "resolved", 234);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1343830372U), "fileTimeLo",
                  "fileTimeLo", 234);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 234);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 234);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 234);
  sf_mex_assign(&c7_rhs234, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs234, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs234), "rhs", "rhs",
                  234);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs234), "lhs", "lhs",
                  234);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan2d.m"), "context",
                  "context", 235);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 235);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 235);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 235);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 235);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 235);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 235);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 235);
  sf_mex_assign(&c7_rhs235, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs235, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs235), "rhs", "rhs",
                  235);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs235), "lhs", "lhs",
                  235);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan2d.m"), "context",
                  "context", 236);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_scalexp_alloc"), "name",
                  "name", 236);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 236);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "resolved", "resolved", 236);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 236);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 236);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 236);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 236);
  sf_mex_assign(&c7_rhs236, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs236, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs236), "rhs", "rhs",
                  236);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs236), "lhs", "lhs",
                  236);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalexpAlloc.p!equalsize"),
                  "context", "context", 237);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("max"), "name", "name", 237);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 237);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/max.m"), "resolved",
                  "resolved", 237);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1311255316U), "fileTimeLo",
                  "fileTimeLo", 237);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 237);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 237);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 237);
  sf_mex_assign(&c7_rhs237, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs237, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs237), "rhs", "rhs",
                  237);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs237), "lhs", "lhs",
                  237);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan2d.m"), "context",
                  "context", 238);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_scalar_atan2"), "name",
                  "name", 238);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 238);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_atan2.m"),
                  "resolved", "resolved", 238);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1286818720U), "fileTimeLo",
                  "fileTimeLo", 238);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 238);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 238);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 238);
  sf_mex_assign(&c7_rhs238, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs238, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs238), "rhs", "rhs",
                  238);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs238), "lhs", "lhs",
                  238);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan2d.m"), "context",
                  "context", 239);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_mtimes_helper"), "name",
                  "name", 239);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 239);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "resolved", "resolved", 239);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1383877294U), "fileTimeLo",
                  "fileTimeLo", 239);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 239);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 239);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 239);
  sf_mex_assign(&c7_rhs239, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs239, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs239), "rhs", "rhs",
                  239);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs239), "lhs", "lhs",
                  239);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "context", "context", 240);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_li_find"), "name", "name",
                  240);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 240);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_li_find.m"), "resolved",
                  "resolved", 240);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1286818786U), "fileTimeLo",
                  "fileTimeLo", 240);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 240);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 240);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 240);
  sf_mex_assign(&c7_rhs240, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs240, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs240), "rhs", "rhs",
                  240);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs240), "lhs", "lhs",
                  240);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_li_find.m"), "context",
                  "context", 241);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 241);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 241);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 241);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1323170578U), "fileTimeLo",
                  "fileTimeLo", 241);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 241);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 241);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 241);
  sf_mex_assign(&c7_rhs241, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs241, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs241), "rhs", "rhs",
                  241);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs241), "lhs", "lhs",
                  241);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_li_find.m!compute_nones"),
                  "context", "context", 242);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 242);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 242);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 242);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1323170578U), "fileTimeLo",
                  "fileTimeLo", 242);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 242);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 242);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 242);
  sf_mex_assign(&c7_rhs242, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs242, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs242), "rhs", "rhs",
                  242);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs242), "lhs", "lhs",
                  242);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_li_find.m!compute_nones"),
                  "context", "context", 243);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 243);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 243);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 243);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 243);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 243);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 243);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 243);
  sf_mex_assign(&c7_rhs243, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs243, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs243), "rhs", "rhs",
                  243);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs243), "lhs", "lhs",
                  243);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_li_find.m!compute_nones"),
                  "context", "context", 244);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 244);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 244);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 244);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1372582416U), "fileTimeLo",
                  "fileTimeLo", 244);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 244);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 244);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 244);
  sf_mex_assign(&c7_rhs244, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs244, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs244), "rhs", "rhs",
                  244);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs244), "lhs", "lhs",
                  244);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_li_find.m"), "context",
                  "context", 245);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 245);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 245);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 245);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 245);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 245);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 245);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 245);
  sf_mex_assign(&c7_rhs245, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs245, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs245), "rhs", "rhs",
                  245);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs245), "lhs", "lhs",
                  245);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_li_find.m"), "context",
                  "context", 246);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 246);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 246);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 246);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1372582416U), "fileTimeLo",
                  "fileTimeLo", 246);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 246);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 246);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 246);
  sf_mex_assign(&c7_rhs246, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs246, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs246), "rhs", "rhs",
                  246);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs246), "lhs", "lhs",
                  246);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "context", "context", 247);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("sort"), "name", "name", 247);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 247);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sort.m"), "resolved",
                  "resolved", 247);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363713856U), "fileTimeLo",
                  "fileTimeLo", 247);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 247);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 247);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 247);
  sf_mex_assign(&c7_rhs247, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs247, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs247), "rhs", "rhs",
                  247);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs247), "lhs", "lhs",
                  247);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sort.m"), "context",
                  "context", 248);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 248);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 248);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 248);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 248);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 248);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 248);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 248);
  sf_mex_assign(&c7_rhs248, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs248, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs248), "rhs", "rhs",
                  248);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs248), "lhs", "lhs",
                  248);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sort.m"), "context",
                  "context", 249);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_sort"), "name", "name",
                  249);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 249);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort.m"), "resolved",
                  "resolved", 249);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1314736612U), "fileTimeLo",
                  "fileTimeLo", 249);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 249);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 249);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 249);
  sf_mex_assign(&c7_rhs249, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs249, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs249), "rhs", "rhs",
                  249);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs249), "lhs", "lhs",
                  249);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort.m"), "context",
                  "context", 250);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_nonsingleton_dim"), "name",
                  "name", 250);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 250);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_nonsingleton_dim.m"),
                  "resolved", "resolved", 250);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1307651242U), "fileTimeLo",
                  "fileTimeLo", 250);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 250);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 250);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 250);
  sf_mex_assign(&c7_rhs250, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs250, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs250), "rhs", "rhs",
                  250);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs250), "lhs", "lhs",
                  250);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_nonsingleton_dim.m"),
                  "context", "context", 251);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 251);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 251);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 251);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1323170578U), "fileTimeLo",
                  "fileTimeLo", 251);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 251);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 251);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 251);
  sf_mex_assign(&c7_rhs251, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs251, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs251), "rhs", "rhs",
                  251);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs251), "lhs", "lhs",
                  251);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort.m"), "context",
                  "context", 252);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_sort"), "name", "name",
                  252);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 252);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort.m"), "resolved",
                  "resolved", 252);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1314736612U), "fileTimeLo",
                  "fileTimeLo", 252);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 252);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 252);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 252);
  sf_mex_assign(&c7_rhs252, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs252, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs252), "rhs", "rhs",
                  252);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs252), "lhs", "lhs",
                  252);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort.m"), "context",
                  "context", 253);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_assert_valid_dim"), "name",
                  "name", 253);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 253);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_dim.m"),
                  "resolved", "resolved", 253);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1372582416U), "fileTimeLo",
                  "fileTimeLo", 253);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 253);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 253);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 253);
  sf_mex_assign(&c7_rhs253, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs253, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs253), "rhs", "rhs",
                  253);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs253), "lhs", "lhs",
                  253);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_dim.m"),
                  "context", "context", 254);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.assertValidDim"),
                  "name", "name", 254);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 254);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assertValidDim.m"),
                  "resolved", "resolved", 254);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1372583160U), "fileTimeLo",
                  "fileTimeLo", 254);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 254);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 254);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 254);
  sf_mex_assign(&c7_rhs254, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs254, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs254), "rhs", "rhs",
                  254);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs254), "lhs", "lhs",
                  254);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assertValidDim.m"),
                  "context", "context", 255);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 255);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 255);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 255);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 255);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 255);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 255);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 255);
  sf_mex_assign(&c7_rhs255, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs255, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs255), "rhs", "rhs",
                  255);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs255), "lhs", "lhs",
                  255);
  sf_mex_destroy(&c7_rhs192);
  sf_mex_destroy(&c7_lhs192);
  sf_mex_destroy(&c7_rhs193);
  sf_mex_destroy(&c7_lhs193);
  sf_mex_destroy(&c7_rhs194);
  sf_mex_destroy(&c7_lhs194);
  sf_mex_destroy(&c7_rhs195);
  sf_mex_destroy(&c7_lhs195);
  sf_mex_destroy(&c7_rhs196);
  sf_mex_destroy(&c7_lhs196);
  sf_mex_destroy(&c7_rhs197);
  sf_mex_destroy(&c7_lhs197);
  sf_mex_destroy(&c7_rhs198);
  sf_mex_destroy(&c7_lhs198);
  sf_mex_destroy(&c7_rhs199);
  sf_mex_destroy(&c7_lhs199);
  sf_mex_destroy(&c7_rhs200);
  sf_mex_destroy(&c7_lhs200);
  sf_mex_destroy(&c7_rhs201);
  sf_mex_destroy(&c7_lhs201);
  sf_mex_destroy(&c7_rhs202);
  sf_mex_destroy(&c7_lhs202);
  sf_mex_destroy(&c7_rhs203);
  sf_mex_destroy(&c7_lhs203);
  sf_mex_destroy(&c7_rhs204);
  sf_mex_destroy(&c7_lhs204);
  sf_mex_destroy(&c7_rhs205);
  sf_mex_destroy(&c7_lhs205);
  sf_mex_destroy(&c7_rhs206);
  sf_mex_destroy(&c7_lhs206);
  sf_mex_destroy(&c7_rhs207);
  sf_mex_destroy(&c7_lhs207);
  sf_mex_destroy(&c7_rhs208);
  sf_mex_destroy(&c7_lhs208);
  sf_mex_destroy(&c7_rhs209);
  sf_mex_destroy(&c7_lhs209);
  sf_mex_destroy(&c7_rhs210);
  sf_mex_destroy(&c7_lhs210);
  sf_mex_destroy(&c7_rhs211);
  sf_mex_destroy(&c7_lhs211);
  sf_mex_destroy(&c7_rhs212);
  sf_mex_destroy(&c7_lhs212);
  sf_mex_destroy(&c7_rhs213);
  sf_mex_destroy(&c7_lhs213);
  sf_mex_destroy(&c7_rhs214);
  sf_mex_destroy(&c7_lhs214);
  sf_mex_destroy(&c7_rhs215);
  sf_mex_destroy(&c7_lhs215);
  sf_mex_destroy(&c7_rhs216);
  sf_mex_destroy(&c7_lhs216);
  sf_mex_destroy(&c7_rhs217);
  sf_mex_destroy(&c7_lhs217);
  sf_mex_destroy(&c7_rhs218);
  sf_mex_destroy(&c7_lhs218);
  sf_mex_destroy(&c7_rhs219);
  sf_mex_destroy(&c7_lhs219);
  sf_mex_destroy(&c7_rhs220);
  sf_mex_destroy(&c7_lhs220);
  sf_mex_destroy(&c7_rhs221);
  sf_mex_destroy(&c7_lhs221);
  sf_mex_destroy(&c7_rhs222);
  sf_mex_destroy(&c7_lhs222);
  sf_mex_destroy(&c7_rhs223);
  sf_mex_destroy(&c7_lhs223);
  sf_mex_destroy(&c7_rhs224);
  sf_mex_destroy(&c7_lhs224);
  sf_mex_destroy(&c7_rhs225);
  sf_mex_destroy(&c7_lhs225);
  sf_mex_destroy(&c7_rhs226);
  sf_mex_destroy(&c7_lhs226);
  sf_mex_destroy(&c7_rhs227);
  sf_mex_destroy(&c7_lhs227);
  sf_mex_destroy(&c7_rhs228);
  sf_mex_destroy(&c7_lhs228);
  sf_mex_destroy(&c7_rhs229);
  sf_mex_destroy(&c7_lhs229);
  sf_mex_destroy(&c7_rhs230);
  sf_mex_destroy(&c7_lhs230);
  sf_mex_destroy(&c7_rhs231);
  sf_mex_destroy(&c7_lhs231);
  sf_mex_destroy(&c7_rhs232);
  sf_mex_destroy(&c7_lhs232);
  sf_mex_destroy(&c7_rhs233);
  sf_mex_destroy(&c7_lhs233);
  sf_mex_destroy(&c7_rhs234);
  sf_mex_destroy(&c7_lhs234);
  sf_mex_destroy(&c7_rhs235);
  sf_mex_destroy(&c7_lhs235);
  sf_mex_destroy(&c7_rhs236);
  sf_mex_destroy(&c7_lhs236);
  sf_mex_destroy(&c7_rhs237);
  sf_mex_destroy(&c7_lhs237);
  sf_mex_destroy(&c7_rhs238);
  sf_mex_destroy(&c7_lhs238);
  sf_mex_destroy(&c7_rhs239);
  sf_mex_destroy(&c7_lhs239);
  sf_mex_destroy(&c7_rhs240);
  sf_mex_destroy(&c7_lhs240);
  sf_mex_destroy(&c7_rhs241);
  sf_mex_destroy(&c7_lhs241);
  sf_mex_destroy(&c7_rhs242);
  sf_mex_destroy(&c7_lhs242);
  sf_mex_destroy(&c7_rhs243);
  sf_mex_destroy(&c7_lhs243);
  sf_mex_destroy(&c7_rhs244);
  sf_mex_destroy(&c7_lhs244);
  sf_mex_destroy(&c7_rhs245);
  sf_mex_destroy(&c7_lhs245);
  sf_mex_destroy(&c7_rhs246);
  sf_mex_destroy(&c7_lhs246);
  sf_mex_destroy(&c7_rhs247);
  sf_mex_destroy(&c7_lhs247);
  sf_mex_destroy(&c7_rhs248);
  sf_mex_destroy(&c7_lhs248);
  sf_mex_destroy(&c7_rhs249);
  sf_mex_destroy(&c7_lhs249);
  sf_mex_destroy(&c7_rhs250);
  sf_mex_destroy(&c7_lhs250);
  sf_mex_destroy(&c7_rhs251);
  sf_mex_destroy(&c7_lhs251);
  sf_mex_destroy(&c7_rhs252);
  sf_mex_destroy(&c7_lhs252);
  sf_mex_destroy(&c7_rhs253);
  sf_mex_destroy(&c7_lhs253);
  sf_mex_destroy(&c7_rhs254);
  sf_mex_destroy(&c7_lhs254);
  sf_mex_destroy(&c7_rhs255);
  sf_mex_destroy(&c7_lhs255);
}

static void c7_e_info_helper(const mxArray **c7_info)
{
  const mxArray *c7_rhs256 = NULL;
  const mxArray *c7_lhs256 = NULL;
  const mxArray *c7_rhs257 = NULL;
  const mxArray *c7_lhs257 = NULL;
  const mxArray *c7_rhs258 = NULL;
  const mxArray *c7_lhs258 = NULL;
  const mxArray *c7_rhs259 = NULL;
  const mxArray *c7_lhs259 = NULL;
  const mxArray *c7_rhs260 = NULL;
  const mxArray *c7_lhs260 = NULL;
  const mxArray *c7_rhs261 = NULL;
  const mxArray *c7_lhs261 = NULL;
  const mxArray *c7_rhs262 = NULL;
  const mxArray *c7_lhs262 = NULL;
  const mxArray *c7_rhs263 = NULL;
  const mxArray *c7_lhs263 = NULL;
  const mxArray *c7_rhs264 = NULL;
  const mxArray *c7_lhs264 = NULL;
  const mxArray *c7_rhs265 = NULL;
  const mxArray *c7_lhs265 = NULL;
  const mxArray *c7_rhs266 = NULL;
  const mxArray *c7_lhs266 = NULL;
  const mxArray *c7_rhs267 = NULL;
  const mxArray *c7_lhs267 = NULL;
  const mxArray *c7_rhs268 = NULL;
  const mxArray *c7_lhs268 = NULL;
  const mxArray *c7_rhs269 = NULL;
  const mxArray *c7_lhs269 = NULL;
  const mxArray *c7_rhs270 = NULL;
  const mxArray *c7_lhs270 = NULL;
  const mxArray *c7_rhs271 = NULL;
  const mxArray *c7_lhs271 = NULL;
  const mxArray *c7_rhs272 = NULL;
  const mxArray *c7_lhs272 = NULL;
  const mxArray *c7_rhs273 = NULL;
  const mxArray *c7_lhs273 = NULL;
  const mxArray *c7_rhs274 = NULL;
  const mxArray *c7_lhs274 = NULL;
  const mxArray *c7_rhs275 = NULL;
  const mxArray *c7_lhs275 = NULL;
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assertValidDim.m"),
                  "context", "context", 256);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("floor"), "name", "name", 256);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 256);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "resolved",
                  "resolved", 256);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363713854U), "fileTimeLo",
                  "fileTimeLo", 256);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 256);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 256);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 256);
  sf_mex_assign(&c7_rhs256, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs256, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs256), "rhs", "rhs",
                  256);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs256), "lhs", "lhs",
                  256);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "context",
                  "context", 257);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 257);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 257);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 257);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 257);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 257);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 257);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 257);
  sf_mex_assign(&c7_rhs257, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs257, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs257), "rhs", "rhs",
                  257);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs257), "lhs", "lhs",
                  257);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "context",
                  "context", 258);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_scalar_floor"), "name",
                  "name", 258);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 258);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m"),
                  "resolved", "resolved", 258);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1286818726U), "fileTimeLo",
                  "fileTimeLo", 258);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 258);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 258);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 258);
  sf_mex_assign(&c7_rhs258, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs258, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs258), "rhs", "rhs",
                  258);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs258), "lhs", "lhs",
                  258);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assertValidDim.m"),
                  "context", "context", 259);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("intmax"), "name", "name", 259);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 259);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "resolved",
                  "resolved", 259);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1362261882U), "fileTimeLo",
                  "fileTimeLo", 259);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 259);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 259);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 259);
  sf_mex_assign(&c7_rhs259, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs259, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs259), "rhs", "rhs",
                  259);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs259), "lhs", "lhs",
                  259);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort.m"), "context",
                  "context", 260);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 260);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 260);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 260);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 260);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 260);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 260);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 260);
  sf_mex_assign(&c7_rhs260, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs260, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs260), "rhs", "rhs",
                  260);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs260), "lhs", "lhs",
                  260);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort.m"), "context",
                  "context", 261);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_sort_idx"), "name", "name",
                  261);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 261);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort_idx.m"), "resolved",
                  "resolved", 261);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1305318004U), "fileTimeLo",
                  "fileTimeLo", 261);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 261);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 261);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 261);
  sf_mex_assign(&c7_rhs261, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs261, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs261), "rhs", "rhs",
                  261);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs261), "lhs", "lhs",
                  261);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort_idx.m"), "context",
                  "context", 262);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 262);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 262);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 262);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1323170578U), "fileTimeLo",
                  "fileTimeLo", 262);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 262);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 262);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 262);
  sf_mex_assign(&c7_rhs262, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs262, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs262), "rhs", "rhs",
                  262);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs262), "lhs", "lhs",
                  262);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort_idx.m"), "context",
                  "context", 263);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_size_ispow2"), "name",
                  "name", 263);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 263);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_size_ispow2.m"),
                  "resolved", "resolved", 263);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363713870U), "fileTimeLo",
                  "fileTimeLo", 263);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 263);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 263);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 263);
  sf_mex_assign(&c7_rhs263, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs263, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs263), "rhs", "rhs",
                  263);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs263), "lhs", "lhs",
                  263);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_size_ispow2.m"),
                  "context", "context", 264);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_unsigned_class"), "name",
                  "name", 264);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 264);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_unsigned_class.m"),
                  "resolved", "resolved", 264);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 264);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 264);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 264);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 264);
  sf_mex_assign(&c7_rhs264, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs264, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs264), "rhs", "rhs",
                  264);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs264), "lhs", "lhs",
                  264);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort_idx.m"), "context",
                  "context", 265);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 265);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 265);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 265);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1372582416U), "fileTimeLo",
                  "fileTimeLo", 265);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 265);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 265);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 265);
  sf_mex_assign(&c7_rhs265, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs265, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs265), "rhs", "rhs",
                  265);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs265), "lhs", "lhs",
                  265);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort_idx.m"), "context",
                  "context", 266);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 266);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 266);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 266);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1375980688U), "fileTimeLo",
                  "fileTimeLo", 266);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 266);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 266);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 266);
  sf_mex_assign(&c7_rhs266, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs266, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs266), "rhs", "rhs",
                  266);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs266), "lhs", "lhs",
                  266);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort_idx.m"), "context",
                  "context", 267);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_index_minus"), "name",
                  "name", 267);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 267);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m"),
                  "resolved", "resolved", 267);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1372582416U), "fileTimeLo",
                  "fileTimeLo", 267);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 267);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 267);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 267);
  sf_mex_assign(&c7_rhs267, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs267, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs267), "rhs", "rhs",
                  267);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs267), "lhs", "lhs",
                  267);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort_idx.m"), "context",
                  "context", 268);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 268);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 268);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 268);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1372582416U), "fileTimeLo",
                  "fileTimeLo", 268);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 268);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 268);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 268);
  sf_mex_assign(&c7_rhs268, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs268, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs268), "rhs", "rhs",
                  268);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs268), "lhs", "lhs",
                  268);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort_idx.m"), "context",
                  "context", 269);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_sort_le"), "name", "name",
                  269);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 269);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort_le.m"), "resolved",
                  "resolved", 269);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1292190510U), "fileTimeLo",
                  "fileTimeLo", 269);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 269);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 269);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 269);
  sf_mex_assign(&c7_rhs269, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs269, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs269), "rhs", "rhs",
                  269);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs269), "lhs", "lhs",
                  269);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort_le.m!eml_sort_descending_le"),
                  "context", "context", 270);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_relop"), "name", "name",
                  270);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("function_handle"),
                  "dominantType", "dominantType", 270);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_relop.m"), "resolved",
                  "resolved", 270);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1342451182U), "fileTimeLo",
                  "fileTimeLo", 270);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 270);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 270);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 270);
  sf_mex_assign(&c7_rhs270, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs270, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs270), "rhs", "rhs",
                  270);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs270), "lhs", "lhs",
                  270);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort_le.m!eml_sort_descending_le"),
                  "context", "context", 271);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("isnan"), "name", "name", 271);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 271);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m"), "resolved",
                  "resolved", 271);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363713858U), "fileTimeLo",
                  "fileTimeLo", 271);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 271);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 271);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 271);
  sf_mex_assign(&c7_rhs271, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs271, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs271), "rhs", "rhs",
                  271);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs271), "lhs", "lhs",
                  271);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m"), "context",
                  "context", 272);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 272);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 272);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 272);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 272);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 272);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 272);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 272);
  sf_mex_assign(&c7_rhs272, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs272, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs272), "rhs", "rhs",
                  272);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs272), "lhs", "lhs",
                  272);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort_idx.m"), "context",
                  "context", 273);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_index_times"), "name",
                  "name", 273);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 273);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m"),
                  "resolved", "resolved", 273);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1372582416U), "fileTimeLo",
                  "fileTimeLo", 273);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 273);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 273);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 273);
  sf_mex_assign(&c7_rhs273, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs273, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs273), "rhs", "rhs",
                  273);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs273), "lhs", "lhs",
                  273);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort_idx.m"), "context",
                  "context", 274);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("eml_index_minus"), "name",
                  "name", 274);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 274);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m"),
                  "resolved", "resolved", 274);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1372582416U), "fileTimeLo",
                  "fileTimeLo", 274);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 274);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 274);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 274);
  sf_mex_assign(&c7_rhs274, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs274, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs274), "rhs", "rhs",
                  274);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs274), "lhs", "lhs",
                  274);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(""), "context", "context", 275);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("length"), "name", "name", 275);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut("single"), "dominantType",
                  "dominantType", 275);
  sf_mex_addfield(*c7_info, c7_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m"), "resolved",
                  "resolved", 275);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(1303146206U), "fileTimeLo",
                  "fileTimeLo", 275);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 275);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 275);
  sf_mex_addfield(*c7_info, c7_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 275);
  sf_mex_assign(&c7_rhs275, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c7_lhs275, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_rhs275), "rhs", "rhs",
                  275);
  sf_mex_addfield(*c7_info, sf_mex_duplicatearraysafe(&c7_lhs275), "lhs", "lhs",
                  275);
  sf_mex_destroy(&c7_rhs256);
  sf_mex_destroy(&c7_lhs256);
  sf_mex_destroy(&c7_rhs257);
  sf_mex_destroy(&c7_lhs257);
  sf_mex_destroy(&c7_rhs258);
  sf_mex_destroy(&c7_lhs258);
  sf_mex_destroy(&c7_rhs259);
  sf_mex_destroy(&c7_lhs259);
  sf_mex_destroy(&c7_rhs260);
  sf_mex_destroy(&c7_lhs260);
  sf_mex_destroy(&c7_rhs261);
  sf_mex_destroy(&c7_lhs261);
  sf_mex_destroy(&c7_rhs262);
  sf_mex_destroy(&c7_lhs262);
  sf_mex_destroy(&c7_rhs263);
  sf_mex_destroy(&c7_lhs263);
  sf_mex_destroy(&c7_rhs264);
  sf_mex_destroy(&c7_lhs264);
  sf_mex_destroy(&c7_rhs265);
  sf_mex_destroy(&c7_lhs265);
  sf_mex_destroy(&c7_rhs266);
  sf_mex_destroy(&c7_lhs266);
  sf_mex_destroy(&c7_rhs267);
  sf_mex_destroy(&c7_lhs267);
  sf_mex_destroy(&c7_rhs268);
  sf_mex_destroy(&c7_lhs268);
  sf_mex_destroy(&c7_rhs269);
  sf_mex_destroy(&c7_lhs269);
  sf_mex_destroy(&c7_rhs270);
  sf_mex_destroy(&c7_lhs270);
  sf_mex_destroy(&c7_rhs271);
  sf_mex_destroy(&c7_lhs271);
  sf_mex_destroy(&c7_rhs272);
  sf_mex_destroy(&c7_lhs272);
  sf_mex_destroy(&c7_rhs273);
  sf_mex_destroy(&c7_lhs273);
  sf_mex_destroy(&c7_rhs274);
  sf_mex_destroy(&c7_lhs274);
  sf_mex_destroy(&c7_rhs275);
  sf_mex_destroy(&c7_lhs275);
}

static real_T c7_cosd(SFc7_SimulationModelInstanceStruct *chartInstance, real_T
                      c7_x)
{
  real_T c7_b_x;
  c7_b_x = c7_x;
  c7_b_cosd(chartInstance, &c7_b_x);
  return c7_b_x;
}

static boolean_T c7_isfinite(SFc7_SimulationModelInstanceStruct *chartInstance,
  real_T c7_x)
{
  real_T c7_b_x;
  boolean_T c7_b_b;
  boolean_T c7_b0;
  real_T c7_c_x;
  boolean_T c7_c_b;
  boolean_T c7_b1;
  (void)chartInstance;
  c7_b_x = c7_x;
  c7_b_b = muDoubleScalarIsInf(c7_b_x);
  c7_b0 = !c7_b_b;
  c7_c_x = c7_x;
  c7_c_b = muDoubleScalarIsNaN(c7_c_x);
  c7_b1 = !c7_c_b;
  return c7_b0 && c7_b1;
}

static void c7_eml_scalar_eg(SFc7_SimulationModelInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static real_T c7_sind(SFc7_SimulationModelInstanceStruct *chartInstance, real_T
                      c7_x)
{
  real_T c7_b_x;
  c7_b_x = c7_x;
  c7_b_sind(chartInstance, &c7_b_x);
  return c7_b_x;
}

static void c7_isincamera(SFc7_SimulationModelInstanceStruct *chartInstance,
  real_T c7_p[2], real_T c7_l[2], real32_T c7_x[8], real32_T c7_y[8], real32_T
  c7_dist[8], real32_T c7_bearing[8])
{
  uint32_T c7_debug_family_var_map[11];
  real_T c7_c1[2];
  real_T c7_c2[2];
  boolean_T c7_isvisible[8];
  real_T c7_nargin = 4.0;
  real_T c7_nargout = 2.0;
  real_T c7_A;
  real_T c7_b_x;
  real_T c7_c_x;
  real_T c7_d_x;
  real_T c7_b_y;
  real_T c7_b_A;
  real_T c7_e_x;
  real_T c7_f_x;
  real_T c7_g_x;
  real_T c7_c_y;
  int32_T c7_i75;
  real_T c7_b_p[2];
  real_T c7_d_y[2];
  real_T c7_dv4[2];
  int32_T c7_i76;
  real_T c7_c_A;
  real_T c7_h_x;
  real_T c7_i_x;
  real_T c7_j_x;
  real_T c7_e_y;
  real_T c7_d_A;
  real_T c7_k_x;
  real_T c7_l_x;
  real_T c7_m_x;
  real_T c7_f_y;
  int32_T c7_i77;
  real_T c7_c_p[2];
  real_T c7_g_y[2];
  real_T c7_dv5[2];
  int32_T c7_i78;
  real_T c7_d_p;
  int32_T c7_i79;
  boolean_T c7_bv1[8];
  real_T c7_e_p;
  int32_T c7_i80;
  boolean_T c7_bv2[8];
  int32_T c7_i81;
  int32_T c7_i82;
  real_T c7_b_c1[2];
  int32_T c7_i83;
  real32_T c7_n_x[8];
  real32_T c7_h_y[8];
  int32_T c7_i84;
  int32_T c7_i85;
  real_T c7_b_c2[2];
  int32_T c7_i86;
  real32_T c7_o_x[8];
  int32_T c7_i87;
  boolean_T c7_bv3[8];
  int32_T c7_i88;
  int32_T c7_i89;
  int32_T c7_i90;
  real32_T c7_p_x[8];
  real32_T c7_q_x[8];
  int32_T c7_i91;
  real32_T c7_i_y[8];
  int32_T c7_i92;
  int32_T c7_i93;
  int32_T c7_k;
  real_T c7_b_k;
  int32_T c7_c_k;
  real32_T c7_r_x;
  real32_T c7_s_x;
  int32_T c7_i94;
  int32_T c7_i95;
  int32_T c7_d_k;
  real_T c7_e_k;
  real32_T c7_j_y;
  real32_T c7_t_x;
  real32_T c7_r;
  real32_T c7_b;
  real32_T c7_k_y;
  real32_T c7_b_r[8];
  int32_T c7_i96;
  int32_T c7_i97;
  boolean_T c7_b_isvisible[8];
  int32_T c7_tmp_sizes;
  int32_T c7_tmp_data[8];
  int32_T c7_loop_ub;
  int32_T c7_i98;
  int32_T c7_i99;
  boolean_T c7_c_isvisible[8];
  int32_T c7_b_loop_ub;
  int32_T c7_i100;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 11U, 11U, c7_c_debug_family_names,
    c7_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c7_c1, 0U, c7_b_sf_marshallOut,
    c7_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c7_c2, 1U, c7_b_sf_marshallOut,
    c7_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c7_isvisible, 2U, c7_g_sf_marshallOut,
    c7_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_nargin, 3U, c7_d_sf_marshallOut,
    c7_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c7_nargout, 4U, c7_d_sf_marshallOut,
    c7_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c7_p, 5U, c7_b_sf_marshallOut,
    c7_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c7_l, 6U, c7_b_sf_marshallOut,
    c7_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c7_x, 7U, c7_f_sf_marshallOut,
    c7_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c7_y, 8U, c7_f_sf_marshallOut,
    c7_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c7_dist, 9U, c7_f_sf_marshallOut,
    c7_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c7_bearing, 10U, c7_f_sf_marshallOut,
    c7_e_sf_marshallIn);
  CV_EML_FCN(0, 4);
  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 166U);
  c7_A = -c7_l[0];
  c7_b_x = c7_A;
  c7_c_x = c7_b_x;
  c7_d_x = c7_c_x;
  c7_b_y = c7_d_x / 2.0;
  c7_b_A = -c7_l[1];
  c7_e_x = c7_b_A;
  c7_f_x = c7_e_x;
  c7_g_x = c7_f_x;
  c7_c_y = c7_g_x / 2.0;
  for (c7_i75 = 0; c7_i75 < 2; c7_i75++) {
    c7_b_p[c7_i75] = c7_p[c7_i75];
  }

  c7_d_y[0] = c7_b_y;
  c7_d_y[1] = c7_c_y;
  c7_polyfit(chartInstance, c7_b_p, c7_d_y, c7_dv4);
  for (c7_i76 = 0; c7_i76 < 2; c7_i76++) {
    c7_c1[c7_i76] = c7_dv4[c7_i76];
  }

  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 167U);
  c7_c_A = c7_l[0];
  c7_h_x = c7_c_A;
  c7_i_x = c7_h_x;
  c7_j_x = c7_i_x;
  c7_e_y = c7_j_x / 2.0;
  c7_d_A = c7_l[1];
  c7_k_x = c7_d_A;
  c7_l_x = c7_k_x;
  c7_m_x = c7_l_x;
  c7_f_y = c7_m_x / 2.0;
  for (c7_i77 = 0; c7_i77 < 2; c7_i77++) {
    c7_c_p[c7_i77] = c7_p[c7_i77];
  }

  c7_g_y[0] = c7_e_y;
  c7_g_y[1] = c7_f_y;
  c7_polyfit(chartInstance, c7_c_p, c7_g_y, c7_dv5);
  for (c7_i78 = 0; c7_i78 < 2; c7_i78++) {
    c7_c2[c7_i78] = c7_dv5[c7_i78];
  }

  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 177U);
  c7_d_p = c7_p[0];
  for (c7_i79 = 0; c7_i79 < 8; c7_i79++) {
    c7_bv1[c7_i79] = ((real_T)c7_x[c7_i79] >= c7_d_p);
  }

  c7_e_p = c7_p[1];
  for (c7_i80 = 0; c7_i80 < 8; c7_i80++) {
    c7_bv2[c7_i80] = ((real_T)c7_x[c7_i80] <= c7_e_p);
  }

  for (c7_i81 = 0; c7_i81 < 8; c7_i81++) {
    c7_bv1[c7_i81] = (c7_bv1[c7_i81] && c7_bv2[c7_i81]);
  }

  for (c7_i82 = 0; c7_i82 < 2; c7_i82++) {
    c7_b_c1[c7_i82] = c7_c1[c7_i82];
  }

  for (c7_i83 = 0; c7_i83 < 8; c7_i83++) {
    c7_n_x[c7_i83] = c7_x[c7_i83];
  }

  c7_polyval(chartInstance, c7_b_c1, c7_n_x, c7_h_y);
  for (c7_i84 = 0; c7_i84 < 8; c7_i84++) {
    c7_bv2[c7_i84] = (c7_y[c7_i84] >= c7_h_y[c7_i84]);
  }

  for (c7_i85 = 0; c7_i85 < 2; c7_i85++) {
    c7_b_c2[c7_i85] = c7_c2[c7_i85];
  }

  for (c7_i86 = 0; c7_i86 < 8; c7_i86++) {
    c7_o_x[c7_i86] = c7_x[c7_i86];
  }

  c7_polyval(chartInstance, c7_b_c2, c7_o_x, c7_h_y);
  for (c7_i87 = 0; c7_i87 < 8; c7_i87++) {
    c7_bv3[c7_i87] = (c7_y[c7_i87] <= c7_h_y[c7_i87]);
  }

  for (c7_i88 = 0; c7_i88 < 8; c7_i88++) {
    c7_bv2[c7_i88] = (c7_bv2[c7_i88] && c7_bv3[c7_i88]);
  }

  for (c7_i89 = 0; c7_i89 < 8; c7_i89++) {
    c7_isvisible[c7_i89] = (c7_bv1[c7_i89] && c7_bv2[c7_i89]);
  }

  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 180U);
  for (c7_i90 = 0; c7_i90 < 8; c7_i90++) {
    c7_p_x[c7_i90] = c7_x[c7_i90];
  }

  c7_power(chartInstance, c7_p_x, c7_q_x);
  for (c7_i91 = 0; c7_i91 < 8; c7_i91++) {
    c7_i_y[c7_i91] = c7_y[c7_i91];
  }

  c7_power(chartInstance, c7_i_y, c7_h_y);
  for (c7_i92 = 0; c7_i92 < 8; c7_i92++) {
    c7_q_x[c7_i92] += c7_h_y[c7_i92];
  }

  for (c7_i93 = 0; c7_i93 < 8; c7_i93++) {
    c7_dist[c7_i93] = c7_q_x[c7_i93];
  }

  for (c7_k = 0; c7_k < 8; c7_k++) {
    c7_b_k = 1.0 + (real_T)c7_k;
    if (c7_dist[(int32_T)c7_b_k - 1] < 0.0F) {
      c7_eml_error(chartInstance);
    }
  }

  for (c7_c_k = 0; c7_c_k < 8; c7_c_k++) {
    c7_b_k = 1.0 + (real_T)c7_c_k;
    c7_r_x = c7_dist[(int32_T)c7_b_k - 1];
    c7_s_x = c7_r_x;
    c7_s_x = muSingleScalarSqrt(c7_s_x);
    c7_dist[(int32_T)c7_b_k - 1] = c7_s_x;
  }

  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 182U);
  for (c7_i94 = 0; c7_i94 < 8; c7_i94++) {
    c7_h_y[c7_i94] = c7_x[c7_i94];
  }

  for (c7_i95 = 0; c7_i95 < 8; c7_i95++) {
    c7_q_x[c7_i95] = -c7_y[c7_i95];
  }

  for (c7_d_k = 0; c7_d_k < 8; c7_d_k++) {
    c7_e_k = 1.0 + (real_T)c7_d_k;
    c7_j_y = c7_h_y[(int32_T)c7_e_k - 1];
    c7_t_x = c7_q_x[(int32_T)c7_e_k - 1];
    c7_r = muSingleScalarAtan2(c7_j_y, c7_t_x);
    c7_b = c7_r;
    c7_k_y = 57.2957802F * c7_b;
    c7_b_r[(int32_T)c7_e_k - 1] = c7_k_y;
  }

  for (c7_i96 = 0; c7_i96 < 8; c7_i96++) {
    c7_bearing[c7_i96] = c7_b_r[c7_i96] - 90.0F;
  }

  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 185U);
  for (c7_i97 = 0; c7_i97 < 8; c7_i97++) {
    c7_b_isvisible[c7_i97] = !c7_isvisible[c7_i97];
  }

  c7_eml_li_find(chartInstance, c7_b_isvisible, c7_tmp_data, &c7_tmp_sizes);
  c7_loop_ub = c7_tmp_sizes - 1;
  for (c7_i98 = 0; c7_i98 <= c7_loop_ub; c7_i98++) {
    c7_dist[c7_tmp_data[c7_i98] - 1] = 0.0F;
  }

  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, 186U);
  for (c7_i99 = 0; c7_i99 < 8; c7_i99++) {
    c7_c_isvisible[c7_i99] = !c7_isvisible[c7_i99];
  }

  c7_eml_li_find(chartInstance, c7_c_isvisible, c7_tmp_data, &c7_tmp_sizes);
  c7_b_loop_ub = c7_tmp_sizes - 1;
  for (c7_i100 = 0; c7_i100 <= c7_b_loop_ub; c7_i100++) {
    c7_bearing[c7_tmp_data[c7_i100] - 1] = 0.0F;
  }

  _SFD_EML_CALL(0U, chartInstance->c7_sfEvent, -186);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c7_polyfit(SFc7_SimulationModelInstanceStruct *chartInstance, real_T
  c7_x[2], real_T c7_y[2], real_T c7_p[2])
{
  int32_T c7_k;
  real_T c7_b_k;
  real_T c7_V[4];
  int32_T c7_c_k;
  int32_T c7_i101;
  real_T c7_b_V[4];
  int32_T c7_i102;
  real_T c7_b_y[2];
  real_T c7_rr;
  real_T c7_p1[2];
  real_T c7_b_rr;
  int32_T c7_i103;
  for (c7_k = 0; c7_k < 2; c7_k++) {
    c7_b_k = 1.0 + (real_T)c7_k;
    c7_V[(int32_T)c7_b_k + 1] = 1.0;
  }

  for (c7_c_k = 0; c7_c_k < 2; c7_c_k++) {
    c7_b_k = 1.0 + (real_T)c7_c_k;
    c7_V[(int32_T)c7_b_k - 1] = c7_x[(int32_T)c7_b_k - 1];
  }

  for (c7_i101 = 0; c7_i101 < 4; c7_i101++) {
    c7_b_V[c7_i101] = c7_V[c7_i101];
  }

  for (c7_i102 = 0; c7_i102 < 2; c7_i102++) {
    c7_b_y[c7_i102] = c7_y[c7_i102];
  }

  c7_eml_qrsolve(chartInstance, c7_b_V, c7_b_y, c7_p1, &c7_rr);
  c7_b_rr = c7_rr;
  if (c7_b_rr <= 1.0) {
    c7_eml_warning(chartInstance);
  }

  for (c7_i103 = 0; c7_i103 < 2; c7_i103++) {
    c7_p[c7_i103] = c7_p1[c7_i103];
  }
}

static void c7_b_eml_scalar_eg(SFc7_SimulationModelInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c7_eml_switch_helper(SFc7_SimulationModelInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c7_eps(SFc7_SimulationModelInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static real_T c7_sqrt(SFc7_SimulationModelInstanceStruct *chartInstance, real_T
                      c7_x)
{
  real_T c7_b_x;
  c7_b_x = c7_x;
  c7_b_sqrt(chartInstance, &c7_b_x);
  return c7_b_x;
}

static void c7_eml_error(SFc7_SimulationModelInstanceStruct *chartInstance)
{
  int32_T c7_i104;
  static char_T c7_cv0[30] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'E', 'l', 'F', 'u', 'n', 'D', 'o', 'm', 'a', 'i', 'n',
    'E', 'r', 'r', 'o', 'r' };

  char_T c7_u[30];
  const mxArray *c7_y = NULL;
  int32_T c7_i105;
  static char_T c7_cv1[4] = { 's', 'q', 'r', 't' };

  char_T c7_b_u[4];
  const mxArray *c7_b_y = NULL;
  (void)chartInstance;
  for (c7_i104 = 0; c7_i104 < 30; c7_i104++) {
    c7_u[c7_i104] = c7_cv0[c7_i104];
  }

  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_create("y", c7_u, 10, 0U, 1U, 0U, 2, 1, 30), false);
  for (c7_i105 = 0; c7_i105 < 4; c7_i105++) {
    c7_b_u[c7_i105] = c7_cv1[c7_i105];
  }

  c7_b_y = NULL;
  sf_mex_assign(&c7_b_y, sf_mex_create("y", c7_b_u, 10, 0U, 1U, 0U, 2, 1, 4),
                false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    2U, 14, c7_y, 14, c7_b_y));
}

static void c7_threshold(SFc7_SimulationModelInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c7_realmin(SFc7_SimulationModelInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c7_eml_qrsolve(SFc7_SimulationModelInstanceStruct *chartInstance,
  real_T c7_A[4], real_T c7_B[2], real_T c7_Y[2], real_T *c7_rankR)
{
  int32_T c7_i106;
  int32_T c7_jpvt[2];
  int32_T c7_i107;
  real_T c7_work[2];
  int32_T c7_k;
  int32_T c7_j;
  int32_T c7_b_j;
  int32_T c7_i108;
  real_T c7_b_A[4];
  real_T c7_vn1[2];
  real_T c7_vn2[2];
  int32_T c7_a;
  int32_T c7_b_a;
  int32_T c7_i;
  int32_T c7_b_i;
  int32_T c7_c_a;
  int32_T c7_d_a;
  int32_T c7_im1;
  int32_T c7_e_a;
  int32_T c7_f_a;
  int32_T c7_ip1;
  int32_T c7_g_a;
  int32_T c7_h_a;
  int32_T c7_c;
  int32_T c7_i_a;
  int32_T c7_b;
  int32_T c7_j_a;
  int32_T c7_b_b;
  int32_T c7_i_i;
  int32_T c7_c_b;
  int32_T c7_d_b;
  int32_T c7_nmi;
  int32_T c7_e_b;
  int32_T c7_f_b;
  int32_T c7_mmi;
  int32_T c7_g_b;
  int32_T c7_h_b;
  int32_T c7_mmip1;
  int32_T c7_i_b;
  int32_T c7_j_b;
  int32_T c7_nmip1;
  int32_T c7_k_a;
  int32_T c7_i109;
  real_T c7_b_vn1[2];
  int32_T c7_k_b;
  int32_T c7_l_a;
  int32_T c7_l_b;
  int32_T c7_pvt;
  int32_T c7_m_a;
  int32_T c7_n_a;
  int32_T c7_b_c;
  int32_T c7_m_b;
  int32_T c7_n_b;
  int32_T c7_c_c;
  int32_T c7_o_b;
  int32_T c7_p_b;
  int32_T c7_pvtcol;
  int32_T c7_q_b;
  int32_T c7_r_b;
  int32_T c7_d_c;
  int32_T c7_s_b;
  int32_T c7_t_b;
  int32_T c7_mcol;
  int32_T c7_itemp;
  real_T c7_atmp;
  int32_T c7_o_a;
  int32_T c7_p_a;
  int32_T c7_e_c;
  real_T c7_b_atmp;
  real_T c7_d1;
  real_T c7_tau[2];
  real_T c7_c_atmp;
  real_T c7_d2;
  real_T c7_d3;
  int32_T c7_m;
  int32_T c7_n;
  int32_T c7_iv0;
  real_T c7_b_tau;
  int32_T c7_lastv;
  int32_T c7_q_a;
  int32_T c7_r_a;
  int32_T c7_f_c;
  int32_T c7_u_b;
  int32_T c7_v_b;
  int32_T c7_g_c;
  int32_T c7_s_a;
  int32_T c7_w_b;
  int32_T c7_t_a;
  int32_T c7_x_b;
  int32_T c7_c_i;
  int32_T c7_u_a;
  int32_T c7_v_a;
  int32_T c7_w_a;
  int32_T c7_x_a;
  int32_T c7_b_m;
  int32_T c7_b_n;
  int32_T c7_lastc;
  int32_T c7_y_a;
  int32_T c7_ab_a;
  int32_T c7_h_c;
  int32_T c7_y_b;
  int32_T c7_ab_b;
  int32_T c7_colbottom;
  int32_T c7_b_colbottom;
  int32_T c7_bb_b;
  int32_T c7_cb_b;
  boolean_T c7_overflow;
  int32_T c7_ia;
  int32_T c7_b_ia;
  int32_T c7_i110;
  int32_T c7_i111;
  real_T c7_c_A[4];
  int32_T c7_i112;
  real_T c7_d_A[4];
  int32_T c7_i113;
  int32_T c7_i114;
  real_T c7_b_work[2];
  int32_T c7_b_ip1;
  int32_T c7_bb_a;
  int32_T c7_cb_a;
  int32_T c7_c_j;
  int32_T c7_db_a;
  int32_T c7_eb_a;
  int32_T c7_i_j;
  real_T c7_x;
  real_T c7_b_x;
  real_T c7_y;
  real_T c7_temp1;
  real_T c7_temp2;
  int32_T c7_fb_a;
  int32_T c7_gb_a;
  int32_T c7_i_c;
  int32_T c7_c_n;
  int32_T c7_ix0;
  int32_T c7_d_n;
  int32_T c7_b_ix0;
  int32_T c7_e_n;
  int32_T c7_c_ix0;
  real_T c7_b_y;
  real_T c7_c_x;
  real_T c7_d_x;
  real_T c7_e_x;
  real_T c7_f_x;
  real_T c7_g_x;
  real_T c7_h_x;
  real_T c7_i_x;
  real_T c7_j_x;
  real_T c7_c_y;
  real_T c7_k_x;
  real_T c7_l_x;
  real_T c7_d_y;
  real_T c7_d;
  real_T c7_tol;
  int32_T c7_b_k;
  real_T c7_c_k;
  real_T c7_m_x;
  real_T c7_n_x;
  real_T c7_o_x;
  real_T c7_p_x;
  real_T c7_e_y;
  real_T c7_q_x;
  real_T c7_r_x;
  real_T c7_f_y;
  real_T c7_b_d;
  int32_T c7_i115;
  int32_T c7_d_j;
  real_T c7_e_j;
  real_T c7_tauj;
  real_T c7_wj;
  real_T c7_d4;
  int32_T c7_i116;
  int32_T c7_d_i;
  real_T c7_e_i;
  real_T c7_hb_a;
  real_T c7_db_b;
  real_T c7_z;
  real_T c7_d5;
  int32_T c7_i117;
  int32_T c7_f_i;
  int32_T c7_g_i;
  int32_T c7_f_j;
  int32_T c7_pj;
  real_T c7_s_x;
  real_T c7_g_y;
  real_T c7_t_x;
  real_T c7_h_y;
  real_T c7_b_z;
  real_T c7_d6;
  int32_T c7_i118;
  int32_T c7_h_i;
  boolean_T exitg1;
  int32_T exitg2;
  boolean_T exitg3;
  boolean_T exitg4;
  c7_b_eml_scalar_eg(chartInstance);
  c7_eml_switch_helper(chartInstance);
  c7_eml_switch_helper(chartInstance);
  for (c7_i106 = 0; c7_i106 < 2; c7_i106++) {
    c7_jpvt[c7_i106] = 1 + c7_i106;
  }

  c7_b_eml_scalar_eg(chartInstance);
  for (c7_i107 = 0; c7_i107 < 2; c7_i107++) {
    c7_work[c7_i107] = 0.0;
  }

  c7_eps(chartInstance);
  c7_sqrt(chartInstance, 2.2204460492503131E-16);
  c7_b_eml_scalar_eg(chartInstance);
  c7_k = 1;
  for (c7_j = 1; c7_j < 3; c7_j++) {
    c7_b_j = c7_j - 1;
    for (c7_i108 = 0; c7_i108 < 4; c7_i108++) {
      c7_b_A[c7_i108] = c7_A[c7_i108];
    }

    c7_vn1[c7_b_j] = c7_eml_xnrm2(chartInstance, c7_b_A, c7_k);
    c7_vn2[c7_b_j] = c7_vn1[c7_b_j];
    c7_a = c7_k;
    c7_b_a = c7_a + 2;
    c7_k = c7_b_a;
  }

  for (c7_i = 1; c7_i < 3; c7_i++) {
    c7_b_i = c7_i;
    c7_c_a = c7_b_i;
    c7_d_a = c7_c_a - 1;
    c7_im1 = c7_d_a;
    c7_e_a = c7_b_i;
    c7_f_a = c7_e_a;
    c7_ip1 = c7_f_a;
    c7_g_a = c7_im1;
    c7_h_a = c7_g_a;
    c7_c = c7_h_a << 1;
    c7_i_a = c7_b_i;
    c7_b = c7_c;
    c7_j_a = c7_i_a;
    c7_b_b = c7_b;
    c7_i_i = (c7_j_a + c7_b_b) - 1;
    c7_c_b = c7_b_i;
    c7_d_b = c7_c_b;
    c7_nmi = 2 - c7_d_b;
    c7_e_b = c7_b_i;
    c7_f_b = c7_e_b;
    c7_mmi = 2 - c7_f_b;
    c7_g_b = c7_mmi;
    c7_h_b = c7_g_b + 1;
    c7_mmip1 = c7_h_b;
    c7_i_b = c7_nmi;
    c7_j_b = c7_i_b;
    c7_nmip1 = c7_j_b;
    c7_k_a = c7_im1;
    for (c7_i109 = 0; c7_i109 < 2; c7_i109++) {
      c7_b_vn1[c7_i109] = c7_vn1[c7_i109];
    }

    c7_k_b = c7_eml_ixamax(chartInstance, c7_nmip1 + 1, c7_b_vn1, c7_b_i);
    c7_l_a = c7_k_a;
    c7_l_b = c7_k_b;
    c7_pvt = c7_l_a + c7_l_b;
    if (c7_pvt != c7_b_i) {
      c7_m_a = c7_pvt;
      c7_n_a = c7_m_a;
      c7_b_c = c7_n_a;
      c7_m_b = c7_b_c - 1;
      c7_n_b = c7_m_b;
      c7_c_c = c7_n_b << 1;
      c7_o_b = c7_c_c;
      c7_p_b = c7_o_b;
      c7_pvtcol = c7_p_b;
      c7_q_b = c7_im1;
      c7_r_b = c7_q_b;
      c7_d_c = c7_r_b << 1;
      c7_s_b = c7_d_c;
      c7_t_b = c7_s_b;
      c7_mcol = c7_t_b;
      c7_b_eml_xswap(chartInstance, c7_A, c7_pvtcol + 1, c7_mcol + 1);
      c7_itemp = c7_jpvt[_SFD_EML_ARRAY_BOUNDS_CHECK("", c7_pvt, 1, 2, 1, 0) - 1];
      c7_jpvt[c7_pvt - 1] = c7_jpvt[c7_b_i - 1];
      c7_jpvt[c7_b_i - 1] = c7_itemp;
      c7_vn1[c7_pvt - 1] = c7_vn1[c7_b_i - 1];
      c7_vn2[c7_pvt - 1] = c7_vn2[c7_b_i - 1];
    }

    c7_atmp = c7_A[c7_i_i];
    if (c7_b_i < 2) {
      c7_o_a = c7_i_i + 1;
      c7_p_a = c7_o_a;
      c7_e_c = c7_p_a;
      c7_b_atmp = c7_atmp;
      c7_d1 = c7_c_eml_matlab_zlarfg(chartInstance, c7_mmip1, &c7_b_atmp, c7_A,
        c7_e_c + 1);
      c7_atmp = c7_b_atmp;
      c7_tau[0] = c7_d1;
    } else {
      c7_c_atmp = c7_atmp;
      c7_d2 = c7_A[c7_i_i];
      c7_d3 = c7_d_eml_matlab_zlarfg(chartInstance, &c7_c_atmp, &c7_d2);
      c7_atmp = c7_c_atmp;
      c7_A[c7_i_i] = c7_d2;
      c7_tau[1] = c7_d3;
    }

    c7_A[c7_i_i] = c7_atmp;
    if (c7_b_i < 2) {
      c7_atmp = c7_A[c7_i_i];
      c7_A[c7_i_i] = 1.0;
      c7_m = c7_mmip1;
      c7_n = c7_nmi;
      c7_iv0 = c7_i_i + 1;
      c7_b_tau = c7_tau[0];
      if (c7_b_tau != 0.0) {
        c7_lastv = c7_m;
        c7_q_a = c7_lastv;
        c7_r_a = c7_q_a;
        c7_f_c = c7_r_a;
        c7_u_b = c7_f_c - 1;
        c7_v_b = c7_u_b;
        c7_g_c = c7_v_b;
        c7_s_a = c7_iv0;
        c7_w_b = c7_g_c;
        c7_t_a = c7_s_a;
        c7_x_b = c7_w_b;
        c7_c_i = c7_t_a + c7_x_b;
        exitg4 = false;
        while ((exitg4 == false) && (c7_lastv > 0)) {
          if (c7_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", c7_c_i, 1, 4, 1, 0) - 1] ==
              0.0) {
            c7_u_a = c7_lastv;
            c7_v_a = c7_u_a - 1;
            c7_lastv = c7_v_a;
            c7_w_a = c7_c_i;
            c7_x_a = c7_w_a - 1;
            c7_c_i = c7_x_a;
          } else {
            exitg4 = true;
          }
        }

        c7_b_m = c7_lastv;
        c7_b_n = c7_n;
        c7_lastc = c7_b_n;
        exitg3 = false;
        while ((exitg3 == false) && (c7_lastc > 0)) {
          c7_y_a = c7_b_m;
          c7_ab_a = c7_y_a;
          c7_h_c = c7_ab_a;
          c7_y_b = c7_h_c - 1;
          c7_ab_b = c7_y_b;
          c7_colbottom = c7_ab_b;
          c7_b_colbottom = c7_colbottom + 3;
          c7_bb_b = c7_b_colbottom;
          c7_cb_b = c7_bb_b;
          if (3 > c7_cb_b) {
            c7_overflow = false;
          } else {
            c7_eml_switch_helper(chartInstance);
            c7_overflow = (c7_cb_b > 2147483646);
          }

          if (c7_overflow) {
            c7_check_forloop_overflow_error(chartInstance, true);
          }

          c7_ia = 3;
          do {
            exitg2 = 0;
            if (c7_ia <= c7_b_colbottom) {
              c7_b_ia = c7_ia;
              if (c7_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", c7_b_ia, 1, 4, 1, 0) - 1]
                  != 0.0) {
                exitg2 = 1;
              } else {
                c7_ia++;
              }
            } else {
              c7_lastc = 0;
              exitg2 = 2;
            }
          } while (exitg2 == 0);

          if (exitg2 == 1) {
            exitg3 = true;
          }
        }
      } else {
        c7_lastv = 0;
        c7_lastc = 0;
      }

      if (c7_lastv > 0) {
        c7_i110 = 3;
        for (c7_i111 = 0; c7_i111 < 4; c7_i111++) {
          c7_c_A[c7_i111] = c7_A[c7_i111];
        }

        for (c7_i112 = 0; c7_i112 < 4; c7_i112++) {
          c7_d_A[c7_i112] = c7_A[c7_i112];
        }

        c7_b_eml_xgemv(chartInstance, c7_lastv, c7_lastc, c7_c_A, c7_i110,
                       c7_d_A, c7_iv0, c7_work);
        c7_i113 = 3;
        for (c7_i114 = 0; c7_i114 < 2; c7_i114++) {
          c7_b_work[c7_i114] = c7_work[c7_i114];
        }

        c7_b_eml_xgerc(chartInstance, c7_lastv, c7_lastc, -c7_b_tau, c7_iv0,
                       c7_b_work, c7_A, c7_i113);
      }

      c7_A[c7_i_i] = c7_atmp;
    }

    c7_b_ip1 = c7_ip1 + 1;
    c7_bb_a = c7_b_ip1;
    c7_cb_a = c7_bb_a;
    if (c7_cb_a > 2) {
    } else {
      c7_eml_switch_helper(chartInstance);
    }

    c7_c_j = c7_b_ip1;
    while (c7_c_j < 3) {
      c7_db_a = c7_b_i;
      c7_eb_a = c7_db_a;
      c7_i_j = c7_eb_a;
      if (c7_vn1[1] != 0.0) {
        c7_x = c7_A[c7_b_i + 1];
        c7_b_x = c7_x;
        c7_y = muDoubleScalarAbs(c7_b_x);
        c7_temp1 = c7_y / c7_vn1[1];
        c7_temp1 = 1.0 - c7_temp1 * c7_temp1;
        if (c7_temp1 < 0.0) {
          c7_temp1 = 0.0;
        }

        c7_temp2 = c7_vn1[1] / c7_vn2[1];
        c7_temp2 = c7_temp1 * (c7_temp2 * c7_temp2);
        if (c7_temp2 <= 1.4901161193847656E-8) {
          if (c7_b_i < 2) {
            c7_fb_a = c7_i_j + 2;
            c7_gb_a = c7_fb_a;
            c7_i_c = c7_gb_a;
            c7_c_n = c7_mmi;
            c7_ix0 = c7_i_c + 1;
            c7_d_n = c7_c_n;
            c7_b_ix0 = c7_ix0;
            c7_threshold(chartInstance);
            c7_e_n = c7_d_n;
            c7_c_ix0 = c7_b_ix0;
            c7_b_y = 0.0;
            if (c7_e_n < 1) {
            } else {
              c7_c_x = c7_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", c7_c_ix0, 1, 4, 1, 0)
                - 1];
              c7_d_x = c7_c_x;
              c7_b_y = muDoubleScalarAbs(c7_d_x);
            }

            c7_vn1[1] = c7_b_y;
            c7_vn2[1] = c7_vn1[1];
          } else {
            c7_vn1[1] = 0.0;
            c7_vn2[1] = 0.0;
          }
        } else {
          c7_e_x = c7_temp1;
          c7_f_x = c7_e_x;
          c7_f_x = muDoubleScalarSqrt(c7_f_x);
          c7_vn1[1] *= c7_f_x;
        }
      }

      c7_c_j = 3;
    }
  }

  *c7_rankR = 0.0;
  c7_eps(chartInstance);
  c7_g_x = c7_A[0];
  c7_h_x = c7_g_x;
  c7_i_x = c7_h_x;
  c7_j_x = c7_i_x;
  c7_c_y = muDoubleScalarAbs(c7_j_x);
  c7_k_x = 0.0;
  c7_l_x = c7_k_x;
  c7_d_y = muDoubleScalarAbs(c7_l_x);
  c7_d = c7_c_y + c7_d_y;
  c7_tol = 2.0 * c7_d * 2.2204460492503131E-16;
  c7_b_k = 0;
  exitg1 = false;
  while ((exitg1 == false) && (c7_b_k < 2)) {
    c7_c_k = 1.0 + (real_T)c7_b_k;
    c7_m_x = c7_A[((int32_T)c7_c_k + (((int32_T)c7_c_k - 1) << 1)) - 1];
    c7_n_x = c7_m_x;
    c7_o_x = c7_n_x;
    c7_p_x = c7_o_x;
    c7_e_y = muDoubleScalarAbs(c7_p_x);
    c7_q_x = 0.0;
    c7_r_x = c7_q_x;
    c7_f_y = muDoubleScalarAbs(c7_r_x);
    c7_b_d = c7_e_y + c7_f_y;
    if (c7_b_d <= c7_tol) {
      exitg1 = true;
    } else {
      (*c7_rankR)++;
      c7_b_k++;
    }
  }

  for (c7_i115 = 0; c7_i115 < 2; c7_i115++) {
    c7_Y[c7_i115] = 0.0;
  }

  for (c7_d_j = 0; c7_d_j < 2; c7_d_j++) {
    c7_e_j = 1.0 + (real_T)c7_d_j;
    c7_tauj = c7_tau[(int32_T)c7_e_j - 1];
    if (c7_tauj != 0.0) {
      c7_wj = c7_B[(int32_T)c7_e_j - 1];
      c7_d4 = c7_e_j + 1.0;
      c7_i116 = (int32_T)(2.0 + (1.0 - c7_d4)) - 1;
      c7_d_i = 0;
      while (c7_d_i <= c7_i116) {
        c7_e_i = c7_d4;
        (real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)c7_e_i, 1, 2, 1, 0);
        (real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)c7_e_i, 1, 2, 1, 0);
        c7_hb_a = c7_A[1 + (((int32_T)c7_e_j - 1) << 1)];
        c7_db_b = c7_B[1];
        c7_z = c7_hb_a * c7_db_b;
        c7_wj += c7_z;
        c7_d_i = 1;
      }

      c7_wj *= c7_tauj;
      if (c7_wj != 0.0) {
        c7_B[(int32_T)c7_e_j - 1] -= c7_wj;
        c7_d5 = c7_e_j + 1.0;
        c7_i117 = (int32_T)(2.0 + (1.0 - c7_d5)) - 1;
        c7_f_i = 0;
        while (c7_f_i <= c7_i117) {
          c7_e_i = c7_d5;
          (real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)c7_e_i, 1, 2, 1, 0);
          (real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)c7_e_i, 1, 2, 1, 0);
          (real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)c7_e_i, 1, 2, 1, 0);
          c7_B[1] -= c7_A[1 + (((int32_T)c7_e_j - 1) << 1)] * c7_wj;
          c7_f_i = 1;
        }
      }
    }
  }

  for (c7_g_i = 0; c7_g_i < 2; c7_g_i++) {
    c7_e_i = 1.0 + (real_T)c7_g_i;
    c7_Y[c7_jpvt[(int32_T)c7_e_i - 1] - 1] = c7_B[(int32_T)c7_e_i - 1];
  }

  for (c7_f_j = 0; c7_f_j < 2; c7_f_j++) {
    c7_e_j = 2.0 + -(real_T)c7_f_j;
    c7_pj = c7_jpvt[(int32_T)c7_e_j - 1] - 1;
    c7_s_x = c7_Y[c7_pj];
    c7_g_y = c7_A[((int32_T)c7_e_j + (((int32_T)c7_e_j - 1) << 1)) - 1];
    c7_t_x = c7_s_x;
    c7_h_y = c7_g_y;
    c7_b_z = c7_t_x / c7_h_y;
    c7_Y[c7_pj] = c7_b_z;
    c7_d6 = c7_e_j - 1.0;
    c7_i118 = (int32_T)c7_d6 - 1;
    c7_h_i = 0;
    while (c7_h_i <= c7_i118) {
      c7_Y[c7_jpvt[0] - 1] -= c7_Y[c7_pj] * c7_A[((int32_T)c7_e_j - 1) << 1];
      c7_h_i = 1;
    }
  }
}

static real_T c7_eml_xnrm2(SFc7_SimulationModelInstanceStruct *chartInstance,
  real_T c7_x[4], int32_T c7_ix0)
{
  real_T c7_y;
  int32_T c7_b_ix0;
  int32_T c7_c_ix0;
  real_T c7_scale;
  int32_T c7_kstart;
  int32_T c7_a;
  int32_T c7_kend;
  int32_T c7_b_kstart;
  int32_T c7_b_kend;
  int32_T c7_b_a;
  int32_T c7_b;
  int32_T c7_c_a;
  int32_T c7_b_b;
  boolean_T c7_overflow;
  int32_T c7_k;
  int32_T c7_b_k;
  real_T c7_b_x;
  real_T c7_c_x;
  real_T c7_absxk;
  real_T c7_t;
  c7_b_ix0 = c7_ix0;
  c7_threshold(chartInstance);
  c7_c_ix0 = c7_b_ix0;
  c7_y = 0.0;
  c7_realmin(chartInstance);
  c7_scale = 2.2250738585072014E-308;
  c7_kstart = c7_c_ix0;
  c7_a = c7_kstart;
  c7_kend = c7_a;
  c7_b_kstart = c7_kstart;
  c7_b_kend = c7_kend + 1;
  c7_b_a = c7_b_kstart;
  c7_b = c7_b_kend;
  c7_c_a = c7_b_a;
  c7_b_b = c7_b;
  if (c7_c_a > c7_b_b) {
    c7_overflow = false;
  } else {
    c7_eml_switch_helper(chartInstance);
    c7_overflow = (c7_b_b > 2147483646);
  }

  if (c7_overflow) {
    c7_check_forloop_overflow_error(chartInstance, true);
  }

  for (c7_k = c7_b_kstart; c7_k <= c7_b_kend; c7_k++) {
    c7_b_k = c7_k;
    c7_b_x = c7_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", c7_b_k, 1, 4, 1, 0) - 1];
    c7_c_x = c7_b_x;
    c7_absxk = muDoubleScalarAbs(c7_c_x);
    if (c7_absxk > c7_scale) {
      c7_t = c7_scale / c7_absxk;
      c7_y = 1.0 + c7_y * c7_t * c7_t;
      c7_scale = c7_absxk;
    } else {
      c7_t = c7_absxk / c7_scale;
      c7_y += c7_t * c7_t;
    }
  }

  return c7_scale * muDoubleScalarSqrt(c7_y);
}

static void c7_check_forloop_overflow_error(SFc7_SimulationModelInstanceStruct
  *chartInstance, boolean_T c7_overflow)
{
  int32_T c7_i119;
  static char_T c7_cv2[34] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'i', 'n', 't', '_', 'f', 'o', 'r', 'l', 'o', 'o', 'p',
    '_', 'o', 'v', 'e', 'r', 'f', 'l', 'o', 'w' };

  char_T c7_u[34];
  const mxArray *c7_y = NULL;
  int32_T c7_i120;
  static char_T c7_cv3[23] = { 'c', 'o', 'd', 'e', 'r', '.', 'i', 'n', 't', 'e',
    'r', 'n', 'a', 'l', '.', 'i', 'n', 'd', 'e', 'x', 'I', 'n', 't' };

  char_T c7_b_u[23];
  const mxArray *c7_b_y = NULL;
  (void)chartInstance;
  (void)c7_overflow;
  for (c7_i119 = 0; c7_i119 < 34; c7_i119++) {
    c7_u[c7_i119] = c7_cv2[c7_i119];
  }

  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_create("y", c7_u, 10, 0U, 1U, 0U, 2, 1, 34), false);
  for (c7_i120 = 0; c7_i120 < 23; c7_i120++) {
    c7_b_u[c7_i120] = c7_cv3[c7_i120];
  }

  c7_b_y = NULL;
  sf_mex_assign(&c7_b_y, sf_mex_create("y", c7_b_u, 10, 0U, 1U, 0U, 2, 1, 23),
                false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    2U, 14, c7_y, 14, c7_b_y));
}

static int32_T c7_eml_ixamax(SFc7_SimulationModelInstanceStruct *chartInstance,
  int32_T c7_n, real_T c7_x[2], int32_T c7_ix0)
{
  int32_T c7_idxmax;
  int32_T c7_b_n;
  int32_T c7_b_ix0;
  int32_T c7_c_n;
  int32_T c7_c_ix0;
  int32_T c7_ix;
  real_T c7_b_x;
  real_T c7_c_x;
  real_T c7_d_x;
  real_T c7_y;
  real_T c7_e_x;
  real_T c7_f_x;
  real_T c7_b_y;
  real_T c7_smax;
  int32_T c7_k;
  int32_T c7_a;
  real_T c7_g_x;
  real_T c7_h_x;
  real_T c7_i_x;
  real_T c7_c_y;
  real_T c7_j_x;
  real_T c7_k_x;
  real_T c7_d_y;
  real_T c7_s;
  c7_b_n = c7_n;
  c7_b_ix0 = c7_ix0;
  c7_c_n = c7_b_n;
  c7_c_ix0 = c7_b_ix0;
  c7_idxmax = 1;
  if (c7_c_n > 1) {
    c7_ix = c7_c_ix0;
    c7_b_x = c7_x[c7_ix - 1];
    c7_c_x = c7_b_x;
    c7_d_x = c7_c_x;
    c7_y = muDoubleScalarAbs(c7_d_x);
    c7_e_x = 0.0;
    c7_f_x = c7_e_x;
    c7_b_y = muDoubleScalarAbs(c7_f_x);
    c7_smax = c7_y + c7_b_y;
    c7_eml_switch_helper(chartInstance);
    c7_k = 2;
    while (c7_k <= 2) {
      c7_a = c7_ix + 1;
      c7_ix = c7_a;
      c7_g_x = c7_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", c7_ix, 1, 2, 1, 0) - 1];
      c7_h_x = c7_g_x;
      c7_i_x = c7_h_x;
      c7_c_y = muDoubleScalarAbs(c7_i_x);
      c7_j_x = 0.0;
      c7_k_x = c7_j_x;
      c7_d_y = muDoubleScalarAbs(c7_k_x);
      c7_s = c7_c_y + c7_d_y;
      if (c7_s > c7_smax) {
        c7_idxmax = 2;
        c7_smax = c7_s;
      }

      c7_k = 3;
    }
  }

  return c7_idxmax;
}

static void c7_eml_xswap(SFc7_SimulationModelInstanceStruct *chartInstance,
  real_T c7_x[4], int32_T c7_ix0, int32_T c7_iy0, real_T c7_b_x[4])
{
  int32_T c7_i121;
  for (c7_i121 = 0; c7_i121 < 4; c7_i121++) {
    c7_b_x[c7_i121] = c7_x[c7_i121];
  }

  c7_b_eml_xswap(chartInstance, c7_b_x, c7_ix0, c7_iy0);
}

static void c7_b_threshold(SFc7_SimulationModelInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c7_eml_matlab_zlarfg(SFc7_SimulationModelInstanceStruct
  *chartInstance, int32_T c7_n, real_T c7_alpha1, real_T c7_x[4], int32_T c7_ix0,
  real_T *c7_b_alpha1, real_T c7_b_x[4], real_T *c7_tau)
{
  int32_T c7_i122;
  *c7_b_alpha1 = c7_alpha1;
  for (c7_i122 = 0; c7_i122 < 4; c7_i122++) {
    c7_b_x[c7_i122] = c7_x[c7_i122];
  }

  *c7_tau = c7_c_eml_matlab_zlarfg(chartInstance, c7_n, c7_b_alpha1, c7_b_x,
    c7_ix0);
}

static real_T c7_b_eml_xnrm2(SFc7_SimulationModelInstanceStruct *chartInstance,
  int32_T c7_n, real_T c7_x[4], int32_T c7_ix0)
{
  real_T c7_y;
  int32_T c7_b_n;
  int32_T c7_b_ix0;
  int32_T c7_c_n;
  int32_T c7_c_ix0;
  real_T c7_b_x;
  real_T c7_c_x;
  c7_b_n = c7_n;
  c7_b_ix0 = c7_ix0;
  c7_threshold(chartInstance);
  c7_c_n = c7_b_n;
  c7_c_ix0 = c7_b_ix0;
  c7_y = 0.0;
  if (c7_c_n < 1) {
  } else {
    c7_b_x = c7_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", c7_c_ix0, 1, 4, 1, 0) - 1];
    c7_c_x = c7_b_x;
    c7_y = muDoubleScalarAbs(c7_c_x);
  }

  return c7_y;
}

static void c7_eml_xscal(SFc7_SimulationModelInstanceStruct *chartInstance,
  int32_T c7_n, real_T c7_a, real_T c7_x[4], int32_T c7_ix0, real_T c7_b_x[4])
{
  int32_T c7_i123;
  for (c7_i123 = 0; c7_i123 < 4; c7_i123++) {
    c7_b_x[c7_i123] = c7_x[c7_i123];
  }

  c7_c_eml_xscal(chartInstance, c7_n, c7_a, c7_b_x, c7_ix0);
}

static void c7_c_threshold(SFc7_SimulationModelInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c7_b_eml_matlab_zlarfg(SFc7_SimulationModelInstanceStruct
  *chartInstance, real_T c7_alpha1, real_T c7_x, real_T *c7_b_alpha1, real_T
  *c7_b_x, real_T *c7_tau)
{
  *c7_b_alpha1 = c7_alpha1;
  *c7_b_x = c7_x;
  *c7_tau = c7_d_eml_matlab_zlarfg(chartInstance, c7_b_alpha1, c7_b_x);
}

static void c7_c_eml_xnrm2(SFc7_SimulationModelInstanceStruct *chartInstance)
{
  c7_threshold(chartInstance);
}

static real_T c7_b_eml_xscal(SFc7_SimulationModelInstanceStruct *chartInstance,
  real_T c7_x)
{
  real_T c7_b_x;
  c7_b_x = c7_x;
  c7_d_eml_xscal(chartInstance, &c7_b_x);
  return c7_b_x;
}

static void c7_eml_xgemv(SFc7_SimulationModelInstanceStruct *chartInstance,
  int32_T c7_m, int32_T c7_n, real_T c7_A[4], int32_T c7_ia0, real_T c7_x[4],
  int32_T c7_ix0, real_T c7_y[2], real_T c7_b_y[2])
{
  int32_T c7_i124;
  int32_T c7_i125;
  real_T c7_b_A[4];
  int32_T c7_i126;
  real_T c7_b_x[4];
  for (c7_i124 = 0; c7_i124 < 2; c7_i124++) {
    c7_b_y[c7_i124] = c7_y[c7_i124];
  }

  for (c7_i125 = 0; c7_i125 < 4; c7_i125++) {
    c7_b_A[c7_i125] = c7_A[c7_i125];
  }

  for (c7_i126 = 0; c7_i126 < 4; c7_i126++) {
    c7_b_x[c7_i126] = c7_x[c7_i126];
  }

  c7_b_eml_xgemv(chartInstance, c7_m, c7_n, c7_b_A, c7_ia0, c7_b_x, c7_ix0,
                 c7_b_y);
}

static void c7_eml_xgerc(SFc7_SimulationModelInstanceStruct *chartInstance,
  int32_T c7_m, int32_T c7_n, real_T c7_alpha1, int32_T c7_ix0, real_T c7_y[2],
  real_T c7_A[4], int32_T c7_ia0, real_T c7_b_A[4])
{
  int32_T c7_i127;
  int32_T c7_i128;
  real_T c7_b_y[2];
  for (c7_i127 = 0; c7_i127 < 4; c7_i127++) {
    c7_b_A[c7_i127] = c7_A[c7_i127];
  }

  for (c7_i128 = 0; c7_i128 < 2; c7_i128++) {
    c7_b_y[c7_i128] = c7_y[c7_i128];
  }

  c7_b_eml_xgerc(chartInstance, c7_m, c7_n, c7_alpha1, c7_ix0, c7_b_y, c7_b_A,
                 c7_ia0);
}

static void c7_eml_warning(SFc7_SimulationModelInstanceStruct *chartInstance)
{
  int32_T c7_i129;
  static char_T c7_varargin_1[44] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A',
    'T', 'L', 'A', 'B', ':', 'p', 'o', 'l', 'y', 'f', 'i', 't', '_', 'R', 'e',
    'p', 'e', 'a', 't', 'e', 'd', 'P', 'o', 'i', 'n', 't', 's', 'O', 'r', 'R',
    'e', 's', 'c', 'a', 'l', 'e' };

  char_T c7_u[44];
  const mxArray *c7_y = NULL;
  (void)chartInstance;
  for (c7_i129 = 0; c7_i129 < 44; c7_i129++) {
    c7_u[c7_i129] = c7_varargin_1[c7_i129];
  }

  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_create("y", c7_u, 10, 0U, 1U, 0U, 2, 1, 44), false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "warning", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    1U, 14, c7_y));
}

static void c7_polyval(SFc7_SimulationModelInstanceStruct *chartInstance, real_T
  c7_p[2], real32_T c7_x[8], real32_T c7_y[8])
{
  int32_T c7_i130;
  real32_T c7_b_p;
  int32_T c7_i131;
  (void)chartInstance;
  for (c7_i130 = 0; c7_i130 < 8; c7_i130++) {
    c7_y[c7_i130] = (real32_T)c7_p[0];
  }

  c7_b_p = (real32_T)c7_p[1];
  for (c7_i131 = 0; c7_i131 < 8; c7_i131++) {
    c7_y[c7_i131] = c7_x[c7_i131] * c7_y[c7_i131] + c7_b_p;
  }
}

static void c7_power(SFc7_SimulationModelInstanceStruct *chartInstance, real32_T
                     c7_a[8], real32_T c7_y[8])
{
  int32_T c7_k;
  real_T c7_b_k;
  real32_T c7_ak;
  real32_T c7_b_a;
  real32_T c7_b_y;
  (void)chartInstance;
  for (c7_k = 0; c7_k < 8; c7_k++) {
    c7_b_k = 1.0 + (real_T)c7_k;
    c7_ak = c7_a[(int32_T)c7_b_k - 1];
    c7_b_a = c7_ak;
    c7_b_y = c7_b_a * c7_b_a;
    c7_y[(int32_T)c7_b_k - 1] = c7_b_y;
  }
}

static void c7_eml_li_find(SFc7_SimulationModelInstanceStruct *chartInstance,
  boolean_T c7_x[8], int32_T c7_y_data[], int32_T *c7_y_sizes)
{
  int32_T c7_k;
  int32_T c7_i;
  int32_T c7_b_i;
  int32_T c7_a;
  int32_T c7_b_a;
  const mxArray *c7_y = NULL;
  int32_T c7_tmp_sizes;
  int32_T c7_loop_ub;
  int32_T c7_i132;
  int32_T c7_tmp_data[8];
  int32_T c7_j;
  int32_T c7_c_i;
  int32_T c7_d_i;
  int32_T c7_c_a;
  int32_T c7_d_a;
  (void)chartInstance;
  c7_k = 0;
  for (c7_i = 1; c7_i < 9; c7_i++) {
    c7_b_i = c7_i - 1;
    if (c7_x[c7_b_i]) {
      c7_a = c7_k;
      c7_b_a = c7_a + 1;
      c7_k = c7_b_a;
    }
  }

  if (c7_k <= 8) {
  } else {
    c7_y = NULL;
    sf_mex_assign(&c7_y, sf_mex_create("y", "Assertion failed.", 15, 0U, 0U, 0U,
      2, 1, strlen("Assertion failed.")), false);
    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14, c7_y);
  }

  c7_tmp_sizes = (int32_T)_SFD_NON_NEGATIVE_CHECK("", (real_T)c7_k);
  c7_loop_ub = (int32_T)_SFD_NON_NEGATIVE_CHECK("", (real_T)c7_k) - 1;
  for (c7_i132 = 0; c7_i132 <= c7_loop_ub; c7_i132++) {
    c7_tmp_data[c7_i132] = 0;
  }

  *c7_y_sizes = c7_tmp_sizes;
  c7_j = 1;
  for (c7_c_i = 1; c7_c_i < 9; c7_c_i++) {
    c7_d_i = c7_c_i;
    if (c7_x[c7_d_i - 1]) {
      c7_y_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c7_j, 1, *c7_y_sizes, 1, 0) - 1]
        = c7_d_i;
      c7_c_a = c7_j;
      c7_d_a = c7_c_a + 1;
      c7_j = c7_d_a;
    }
  }
}

static void c7_eml_sort(SFc7_SimulationModelInstanceStruct *chartInstance,
  real32_T c7_x[8], real32_T c7_y[8], int32_T c7_idx[8])
{
  int32_T c7_i133;
  real32_T c7_b_x[8];
  for (c7_i133 = 0; c7_i133 < 8; c7_i133++) {
    c7_b_x[c7_i133] = c7_x[c7_i133];
  }

  c7_b_eml_sort(chartInstance, c7_b_x, c7_y, c7_idx);
}

static void c7_b_eml_sort(SFc7_SimulationModelInstanceStruct *chartInstance,
  real32_T c7_x[8], real32_T c7_y[8], int32_T c7_idx[8])
{
  int32_T c7_i134;
  real32_T c7_b_x[8];
  for (c7_i134 = 0; c7_i134 < 8; c7_i134++) {
    c7_b_x[c7_i134] = c7_x[c7_i134];
  }

  c7_eml_sort_idx(chartInstance, c7_b_x, c7_idx, c7_y);
}

static void c7_eml_sort_idx(SFc7_SimulationModelInstanceStruct *chartInstance,
  real32_T c7_x[8], int32_T c7_idx[8], real32_T c7_y[8])
{
  int32_T c7_k;
  int32_T c7_b_k;
  int32_T c7_c_k;
  int32_T c7_a;
  int32_T c7_b_a;
  int32_T c7_c;
  int32_T c7_irow1;
  int32_T c7_irow2;
  real32_T c7_c_a;
  real32_T c7_b;
  real32_T c7_d_a;
  real32_T c7_b_b;
  boolean_T c7_p;
  real32_T c7_b_x;
  boolean_T c7_c_b;
  boolean_T c7_b2;
  boolean_T c7_b_p;
  int32_T c7_e_a;
  int32_T c7_f_a;
  int32_T c7_b_c;
  int32_T c7_g_a;
  int32_T c7_h_a;
  int32_T c7_c_c;
  int32_T c7_i135;
  int32_T c7_idx0[8];
  int32_T c7_i;
  int32_T c7_i_a;
  int32_T c7_j_a;
  int32_T c7_i2;
  int32_T c7_j;
  int32_T c7_d_b;
  int32_T c7_e_b;
  int32_T c7_pEnd;
  int32_T c7_c_p;
  int32_T c7_q;
  int32_T c7_k_a;
  int32_T c7_f_b;
  int32_T c7_l_a;
  int32_T c7_g_b;
  int32_T c7_qEnd;
  int32_T c7_m_a;
  int32_T c7_h_b;
  int32_T c7_n_a;
  int32_T c7_i_b;
  int32_T c7_kEnd;
  int32_T c7_b_irow1;
  int32_T c7_b_irow2;
  real32_T c7_o_a;
  real32_T c7_j_b;
  real32_T c7_p_a;
  real32_T c7_k_b;
  boolean_T c7_d_p;
  real32_T c7_c_x;
  boolean_T c7_l_b;
  boolean_T c7_b3;
  boolean_T c7_e_p;
  int32_T c7_q_a;
  int32_T c7_r_a;
  int32_T c7_s_a;
  int32_T c7_t_a;
  int32_T c7_u_a;
  int32_T c7_v_a;
  int32_T c7_w_a;
  int32_T c7_x_a;
  int32_T c7_y_a;
  int32_T c7_ab_a;
  int32_T c7_bb_a;
  int32_T c7_cb_a;
  int32_T c7_db_a;
  int32_T c7_eb_a;
  int32_T c7_fb_a;
  int32_T c7_gb_a;
  int32_T c7_b_kEnd;
  int32_T c7_m_b;
  int32_T c7_n_b;
  boolean_T c7_overflow;
  int32_T c7_d_k;
  int32_T c7_hb_a;
  int32_T c7_o_b;
  int32_T c7_ib_a;
  int32_T c7_p_b;
  int32_T c7_d_c;
  int32_T c7_jb_a;
  int32_T c7_q_b;
  int32_T c7_kb_a;
  int32_T c7_r_b;
  int32_T c7_e_k;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  for (c7_k = 1; c7_k < 9; c7_k++) {
    c7_b_k = c7_k;
    c7_idx[c7_b_k - 1] = c7_b_k;
  }

  for (c7_c_k = 1; c7_c_k < 8; c7_c_k += 2) {
    c7_b_k = c7_c_k;
    c7_a = c7_b_k;
    c7_b_a = c7_a;
    c7_c = c7_b_a;
    c7_irow1 = c7_b_k;
    c7_irow2 = c7_c + 1;
    c7_c_a = c7_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", c7_irow1, 1, 8, 1, 0) - 1];
    c7_b = c7_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", c7_irow2, 1, 8, 1, 0) - 1];
    c7_d_a = c7_c_a;
    c7_b_b = c7_b;
    c7_p = (c7_d_a >= c7_b_b);
    guard2 = false;
    if (c7_p) {
      guard2 = true;
    } else {
      c7_b_x = c7_c_a;
      c7_c_b = muSingleScalarIsNaN(c7_b_x);
      if (c7_c_b) {
        guard2 = true;
      } else {
        c7_b2 = false;
      }
    }

    if (guard2 == true) {
      c7_b2 = true;
    }

    c7_b_p = c7_b2;
    if (c7_b_p) {
    } else {
      c7_e_a = c7_b_k;
      c7_f_a = c7_e_a;
      c7_b_c = c7_f_a;
      c7_idx[c7_b_k - 1] = c7_b_c + 1;
      c7_g_a = c7_b_k;
      c7_h_a = c7_g_a;
      c7_c_c = c7_h_a;
      c7_idx[_SFD_EML_ARRAY_BOUNDS_CHECK("", c7_c_c + 1, 1, 8, 1, 0) - 1] =
        c7_b_k;
    }
  }

  for (c7_i135 = 0; c7_i135 < 8; c7_i135++) {
    c7_idx0[c7_i135] = 1;
  }

  c7_i = 2;
  while (c7_i < 8) {
    c7_i_a = c7_i;
    c7_j_a = c7_i_a;
    c7_i2 = c7_j_a << 1;
    c7_j = 1;
    c7_d_b = c7_i;
    c7_e_b = c7_d_b + 1;
    for (c7_pEnd = c7_e_b; c7_pEnd < 9; c7_pEnd = c7_kb_a + c7_r_b) {
      c7_c_p = c7_j;
      c7_q = c7_pEnd;
      c7_k_a = c7_j;
      c7_f_b = c7_i2;
      c7_l_a = c7_k_a;
      c7_g_b = c7_f_b;
      c7_qEnd = c7_l_a + c7_g_b;
      c7_b_k = 1;
      c7_m_a = c7_qEnd;
      c7_h_b = c7_j;
      c7_n_a = c7_m_a;
      c7_i_b = c7_h_b;
      c7_kEnd = c7_n_a - c7_i_b;
      while (c7_b_k <= c7_kEnd) {
        c7_b_irow1 = c7_idx[_SFD_EML_ARRAY_BOUNDS_CHECK("", c7_c_p, 1, 8, 1, 0)
          - 1];
        c7_b_irow2 = c7_idx[_SFD_EML_ARRAY_BOUNDS_CHECK("", c7_q, 1, 8, 1, 0) -
          1];
        c7_o_a = c7_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", c7_b_irow1, 1, 8, 1, 0) -
          1];
        c7_j_b = c7_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", c7_b_irow2, 1, 8, 1, 0) -
          1];
        c7_p_a = c7_o_a;
        c7_k_b = c7_j_b;
        c7_d_p = (c7_p_a >= c7_k_b);
        guard1 = false;
        if (c7_d_p) {
          guard1 = true;
        } else {
          c7_c_x = c7_o_a;
          c7_l_b = muSingleScalarIsNaN(c7_c_x);
          if (c7_l_b) {
            guard1 = true;
          } else {
            c7_b3 = false;
          }
        }

        if (guard1 == true) {
          c7_b3 = true;
        }

        c7_e_p = c7_b3;
        if (c7_e_p) {
          c7_idx0[_SFD_EML_ARRAY_BOUNDS_CHECK("", c7_b_k, 1, 8, 1, 0) - 1] =
            c7_idx[c7_c_p - 1];
          c7_q_a = c7_c_p;
          c7_r_a = c7_q_a + 1;
          c7_c_p = c7_r_a;
          if (c7_c_p == c7_pEnd) {
            while (c7_q < c7_qEnd) {
              c7_s_a = c7_b_k;
              c7_t_a = c7_s_a + 1;
              c7_b_k = c7_t_a;
              c7_idx0[_SFD_EML_ARRAY_BOUNDS_CHECK("", c7_b_k, 1, 8, 1, 0) - 1] =
                c7_idx[_SFD_EML_ARRAY_BOUNDS_CHECK("", c7_q, 1, 8, 1, 0) - 1];
              c7_u_a = c7_q;
              c7_v_a = c7_u_a + 1;
              c7_q = c7_v_a;
            }
          }
        } else {
          c7_idx0[_SFD_EML_ARRAY_BOUNDS_CHECK("", c7_b_k, 1, 8, 1, 0) - 1] =
            c7_idx[c7_q - 1];
          c7_w_a = c7_q;
          c7_x_a = c7_w_a + 1;
          c7_q = c7_x_a;
          if (c7_q == c7_qEnd) {
            while (c7_c_p < c7_pEnd) {
              c7_y_a = c7_b_k;
              c7_ab_a = c7_y_a + 1;
              c7_b_k = c7_ab_a;
              c7_idx0[_SFD_EML_ARRAY_BOUNDS_CHECK("", c7_b_k, 1, 8, 1, 0) - 1] =
                c7_idx[_SFD_EML_ARRAY_BOUNDS_CHECK("", c7_c_p, 1, 8, 1, 0) - 1];
              c7_bb_a = c7_c_p;
              c7_cb_a = c7_bb_a + 1;
              c7_c_p = c7_cb_a;
            }
          }
        }

        c7_db_a = c7_b_k;
        c7_eb_a = c7_db_a + 1;
        c7_b_k = c7_eb_a;
      }

      c7_fb_a = c7_j;
      c7_gb_a = c7_fb_a;
      c7_c_p = c7_gb_a;
      c7_b_kEnd = c7_kEnd;
      c7_m_b = c7_b_kEnd;
      c7_n_b = c7_m_b;
      if (1 > c7_n_b) {
        c7_overflow = false;
      } else {
        c7_eml_switch_helper(chartInstance);
        c7_overflow = (c7_n_b > 2147483646);
      }

      if (c7_overflow) {
        c7_check_forloop_overflow_error(chartInstance, true);
      }

      for (c7_d_k = 1; c7_d_k <= c7_b_kEnd; c7_d_k++) {
        c7_b_k = c7_d_k;
        c7_hb_a = c7_c_p - 1;
        c7_o_b = c7_b_k;
        c7_ib_a = c7_hb_a;
        c7_p_b = c7_o_b;
        c7_d_c = c7_ib_a + c7_p_b;
        c7_idx[_SFD_EML_ARRAY_BOUNDS_CHECK("", c7_d_c, 1, 8, 1, 0) - 1] =
          c7_idx0[_SFD_EML_ARRAY_BOUNDS_CHECK("", c7_b_k, 1, 8, 1, 0) - 1];
      }

      c7_j = c7_qEnd;
      c7_jb_a = c7_j;
      c7_q_b = c7_i;
      c7_kb_a = c7_jb_a;
      c7_r_b = c7_q_b;
    }

    c7_i = c7_i2;
  }

  for (c7_e_k = 1; c7_e_k < 9; c7_e_k++) {
    c7_b_k = c7_e_k - 1;
    c7_y[c7_b_k] = c7_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", c7_idx[c7_b_k], 1, 8, 1,
      0) - 1];
  }
}

static const mxArray *c7_h_sf_marshallOut(void *chartInstanceVoid, void
  *c7_inData)
{
  const mxArray *c7_mxArrayOutData = NULL;
  int32_T c7_u;
  const mxArray *c7_y = NULL;
  SFc7_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc7_SimulationModelInstanceStruct *)chartInstanceVoid;
  c7_mxArrayOutData = NULL;
  c7_u = *(int32_T *)c7_inData;
  c7_y = NULL;
  sf_mex_assign(&c7_y, sf_mex_create("y", &c7_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c7_mxArrayOutData, c7_y, false);
  return c7_mxArrayOutData;
}

static int32_T c7_h_emlrt_marshallIn(SFc7_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId)
{
  int32_T c7_y;
  int32_T c7_i136;
  (void)chartInstance;
  sf_mex_import(c7_parentId, sf_mex_dup(c7_u), &c7_i136, 1, 6, 0U, 0, 0U, 0);
  c7_y = c7_i136;
  sf_mex_destroy(&c7_u);
  return c7_y;
}

static void c7_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c7_mxArrayInData, const char_T *c7_varName, void *c7_outData)
{
  const mxArray *c7_b_sfEvent;
  const char_T *c7_identifier;
  emlrtMsgIdentifier c7_thisId;
  int32_T c7_y;
  SFc7_SimulationModelInstanceStruct *chartInstance;
  chartInstance = (SFc7_SimulationModelInstanceStruct *)chartInstanceVoid;
  c7_b_sfEvent = sf_mex_dup(c7_mxArrayInData);
  c7_identifier = c7_varName;
  c7_thisId.fIdentifier = c7_identifier;
  c7_thisId.fParent = NULL;
  c7_y = c7_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c7_b_sfEvent),
    &c7_thisId);
  sf_mex_destroy(&c7_b_sfEvent);
  *(int32_T *)c7_outData = c7_y;
  sf_mex_destroy(&c7_mxArrayInData);
}

static uint8_T c7_i_emlrt_marshallIn(SFc7_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c7_b_is_active_c7_SimulationModel, const char_T
  *c7_identifier)
{
  uint8_T c7_y;
  emlrtMsgIdentifier c7_thisId;
  c7_thisId.fIdentifier = c7_identifier;
  c7_thisId.fParent = NULL;
  c7_y = c7_j_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c7_b_is_active_c7_SimulationModel), &c7_thisId);
  sf_mex_destroy(&c7_b_is_active_c7_SimulationModel);
  return c7_y;
}

static uint8_T c7_j_emlrt_marshallIn(SFc7_SimulationModelInstanceStruct
  *chartInstance, const mxArray *c7_u, const emlrtMsgIdentifier *c7_parentId)
{
  uint8_T c7_y;
  uint8_T c7_u0;
  (void)chartInstance;
  sf_mex_import(c7_parentId, sf_mex_dup(c7_u), &c7_u0, 1, 3, 0U, 0, 0U, 0);
  c7_y = c7_u0;
  sf_mex_destroy(&c7_u);
  return c7_y;
}

static void c7_b_cosd(SFc7_SimulationModelInstanceStruct *chartInstance, real_T *
                      c7_x)
{
  real_T c7_b_x;
  real_T c7_c_x;
  real_T c7_d_x;
  real_T c7_e_x;
  real_T c7_xk;
  real_T c7_f_x;
  real_T c7_g_x;
  real_T c7_absx;
  real_T c7_h_x;
  real_T c7_i_x;
  int8_T c7_n;
  int8_T c7_b_n;
  c7_b_x = *c7_x;
  if (!c7_isfinite(chartInstance, c7_b_x)) {
    *c7_x = rtNaN;
  } else {
    c7_c_x = c7_b_x;
    c7_d_x = c7_c_x;
    c7_e_x = c7_d_x;
    c7_eml_scalar_eg(chartInstance);
    c7_xk = c7_e_x;
    c7_d_x = muDoubleScalarRem(c7_xk, 360.0);
    c7_f_x = c7_d_x;
    c7_g_x = c7_f_x;
    c7_absx = muDoubleScalarAbs(c7_g_x);
    if (c7_absx > 180.0) {
      if (c7_d_x > 0.0) {
        c7_d_x -= 360.0;
      } else {
        c7_d_x += 360.0;
      }

      c7_h_x = c7_d_x;
      c7_i_x = c7_h_x;
      c7_absx = muDoubleScalarAbs(c7_i_x);
    }

    if (c7_absx <= 45.0) {
      c7_d_x *= 0.017453292519943295;
      c7_n = 0;
    } else if (c7_absx <= 135.0) {
      if (c7_d_x > 0.0) {
        c7_d_x = 0.017453292519943295 * (c7_d_x - 90.0);
        c7_n = 1;
      } else {
        c7_d_x = 0.017453292519943295 * (c7_d_x + 90.0);
        c7_n = -1;
      }
    } else if (c7_d_x > 0.0) {
      c7_d_x = 0.017453292519943295 * (c7_d_x - 180.0);
      c7_n = 2;
    } else {
      c7_d_x = 0.017453292519943295 * (c7_d_x + 180.0);
      c7_n = -2;
    }

    c7_b_n = c7_n;
    c7_b_x = c7_d_x;
    if ((real_T)c7_b_n == 0.0) {
      *c7_x = muDoubleScalarCos(c7_b_x);
    } else if ((real_T)c7_b_n == 1.0) {
      *c7_x = -muDoubleScalarSin(c7_b_x);
    } else if ((real_T)c7_b_n == -1.0) {
      *c7_x = muDoubleScalarSin(c7_b_x);
    } else {
      *c7_x = -muDoubleScalarCos(c7_b_x);
    }
  }
}

static void c7_b_sind(SFc7_SimulationModelInstanceStruct *chartInstance, real_T *
                      c7_x)
{
  real_T c7_b_x;
  real_T c7_d7;
  real_T c7_c_x;
  real_T c7_d_x;
  real_T c7_e_x;
  real_T c7_xk;
  real_T c7_f_x;
  real_T c7_g_x;
  real_T c7_absx;
  real_T c7_h_x;
  real_T c7_i_x;
  int8_T c7_n;
  int8_T c7_b_n;
  c7_b_x = *c7_x;
  if (!c7_isfinite(chartInstance, c7_b_x)) {
    c7_d7 = rtNaN;
  } else {
    c7_c_x = c7_b_x;
    c7_d_x = c7_c_x;
    c7_e_x = c7_d_x;
    c7_eml_scalar_eg(chartInstance);
    c7_xk = c7_e_x;
    c7_d_x = muDoubleScalarRem(c7_xk, 360.0);
    c7_f_x = c7_d_x;
    c7_g_x = c7_f_x;
    c7_absx = muDoubleScalarAbs(c7_g_x);
    if (c7_absx > 180.0) {
      if (c7_d_x > 0.0) {
        c7_d_x -= 360.0;
      } else {
        c7_d_x += 360.0;
      }

      c7_h_x = c7_d_x;
      c7_i_x = c7_h_x;
      c7_absx = muDoubleScalarAbs(c7_i_x);
    }

    if (c7_absx <= 45.0) {
      c7_d_x *= 0.017453292519943295;
      c7_n = 0;
    } else if (c7_absx <= 135.0) {
      if (c7_d_x > 0.0) {
        c7_d_x = 0.017453292519943295 * (c7_d_x - 90.0);
        c7_n = 1;
      } else {
        c7_d_x = 0.017453292519943295 * (c7_d_x + 90.0);
        c7_n = -1;
      }
    } else if (c7_d_x > 0.0) {
      c7_d_x = 0.017453292519943295 * (c7_d_x - 180.0);
      c7_n = 2;
    } else {
      c7_d_x = 0.017453292519943295 * (c7_d_x + 180.0);
      c7_n = -2;
    }

    c7_b_n = c7_n;
    c7_b_x = c7_d_x;
    if ((real_T)c7_b_n == 0.0) {
      c7_d7 = muDoubleScalarSin(c7_b_x);
    } else if ((real_T)c7_b_n == 1.0) {
      c7_d7 = muDoubleScalarCos(c7_b_x);
    } else if ((real_T)c7_b_n == -1.0) {
      c7_d7 = -muDoubleScalarCos(c7_b_x);
    } else {
      c7_d7 = -muDoubleScalarSin(c7_b_x);
    }
  }

  *c7_x = c7_d7;
}

static void c7_b_sqrt(SFc7_SimulationModelInstanceStruct *chartInstance, real_T *
                      c7_x)
{
  (void)chartInstance;
  *c7_x = muDoubleScalarSqrt(*c7_x);
}

static void c7_b_eml_xswap(SFc7_SimulationModelInstanceStruct *chartInstance,
  real_T c7_x[4], int32_T c7_ix0, int32_T c7_iy0)
{
  int32_T c7_b_ix0;
  int32_T c7_b_iy0;
  int32_T c7_c_ix0;
  int32_T c7_c_iy0;
  int32_T c7_ix;
  int32_T c7_iy;
  int32_T c7_k;
  real_T c7_temp;
  int32_T c7_a;
  int32_T c7_b_a;
  c7_b_ix0 = c7_ix0;
  c7_b_iy0 = c7_iy0;
  c7_b_threshold(chartInstance);
  c7_c_ix0 = c7_b_ix0;
  c7_c_iy0 = c7_b_iy0;
  c7_ix = c7_c_ix0;
  c7_iy = c7_c_iy0;
  for (c7_k = 1; c7_k < 3; c7_k++) {
    c7_temp = c7_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", c7_ix, 1, 4, 1, 0) - 1];
    c7_x[c7_ix - 1] = c7_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", c7_iy, 1, 4, 1, 0) -
      1];
    c7_x[c7_iy - 1] = c7_temp;
    c7_a = c7_ix + 1;
    c7_ix = c7_a;
    c7_b_a = c7_iy + 1;
    c7_iy = c7_b_a;
  }
}

static real_T c7_c_eml_matlab_zlarfg(SFc7_SimulationModelInstanceStruct
  *chartInstance, int32_T c7_n, real_T *c7_alpha1, real_T c7_x[4], int32_T
  c7_ix0)
{
  real_T c7_tau;
  int32_T c7_nm1;
  int32_T c7_i137;
  int32_T c7_i138;
  int32_T c7_i139;
  real_T c7_b_x[4];
  real_T c7_xnorm;
  real_T c7_x1;
  real_T c7_x2;
  real_T c7_a;
  real_T c7_b;
  real_T c7_beta1;
  real_T c7_c_x;
  real_T c7_d_x;
  real_T c7_y;
  int32_T c7_knt;
  int32_T c7_b_a;
  int32_T c7_c_a;
  real_T c7_d8;
  real_T c7_e_x;
  real_T c7_f_x;
  real_T c7_b_y;
  int32_T c7_i140;
  int32_T c7_i141;
  int32_T c7_i142;
  real_T c7_g_x[4];
  real_T c7_b_x1;
  real_T c7_b_x2;
  real_T c7_d_a;
  real_T c7_b_b;
  real_T c7_h_x;
  real_T c7_c_y;
  real_T c7_i_x;
  real_T c7_d_y;
  real_T c7_e_y;
  real_T c7_f_y;
  int32_T c7_b_knt;
  int32_T c7_c_b;
  int32_T c7_d_b;
  boolean_T c7_overflow;
  int32_T c7_k;
  real_T c7_j_x;
  real_T c7_g_y;
  real_T c7_k_x;
  real_T c7_h_y;
  real_T c7_i_y;
  real_T c7_j_y;
  c7_tau = 0.0;
  c7_nm1 = c7_n - 1;
  c7_i137 = 0;
  for (c7_i138 = 0; c7_i138 < 2; c7_i138++) {
    for (c7_i139 = 0; c7_i139 < 2; c7_i139++) {
      c7_b_x[c7_i139 + c7_i137] = c7_x[c7_i139 + c7_i137];
    }

    c7_i137 += 2;
  }

  c7_xnorm = c7_b_eml_xnrm2(chartInstance, c7_nm1, c7_b_x, c7_ix0);
  if (c7_xnorm != 0.0) {
    c7_x1 = *c7_alpha1;
    c7_x2 = c7_xnorm;
    c7_a = c7_x1;
    c7_b = c7_x2;
    c7_beta1 = muDoubleScalarHypot(c7_a, c7_b);
    if (*c7_alpha1 >= 0.0) {
      c7_beta1 = -c7_beta1;
    }

    c7_realmin(chartInstance);
    c7_eps(chartInstance);
    c7_c_x = c7_beta1;
    c7_d_x = c7_c_x;
    c7_y = muDoubleScalarAbs(c7_d_x);
    if (c7_y < 1.0020841800044864E-292) {
      c7_knt = 0;
      do {
        c7_b_a = c7_knt;
        c7_c_a = c7_b_a + 1;
        c7_knt = c7_c_a;
        c7_d8 = 9.9792015476736E+291;
        c7_c_eml_xscal(chartInstance, c7_nm1, c7_d8, c7_x, c7_ix0);
        c7_beta1 *= 9.9792015476736E+291;
        *c7_alpha1 *= 9.9792015476736E+291;
        c7_e_x = c7_beta1;
        c7_f_x = c7_e_x;
        c7_b_y = muDoubleScalarAbs(c7_f_x);
      } while (!(c7_b_y >= 1.0020841800044864E-292));

      c7_i140 = 0;
      for (c7_i141 = 0; c7_i141 < 2; c7_i141++) {
        for (c7_i142 = 0; c7_i142 < 2; c7_i142++) {
          c7_g_x[c7_i142 + c7_i140] = c7_x[c7_i142 + c7_i140];
        }

        c7_i140 += 2;
      }

      c7_xnorm = c7_b_eml_xnrm2(chartInstance, c7_nm1, c7_g_x, c7_ix0);
      c7_b_x1 = *c7_alpha1;
      c7_b_x2 = c7_xnorm;
      c7_d_a = c7_b_x1;
      c7_b_b = c7_b_x2;
      c7_beta1 = muDoubleScalarHypot(c7_d_a, c7_b_b);
      if (*c7_alpha1 >= 0.0) {
        c7_beta1 = -c7_beta1;
      }

      c7_h_x = c7_beta1 - *c7_alpha1;
      c7_c_y = c7_beta1;
      c7_i_x = c7_h_x;
      c7_d_y = c7_c_y;
      c7_tau = c7_i_x / c7_d_y;
      c7_e_y = *c7_alpha1 - c7_beta1;
      c7_f_y = c7_e_y;
      *c7_alpha1 = 1.0 / c7_f_y;
      c7_c_eml_xscal(chartInstance, c7_nm1, *c7_alpha1, c7_x, c7_ix0);
      c7_b_knt = c7_knt;
      c7_c_b = c7_b_knt;
      c7_d_b = c7_c_b;
      if (1 > c7_d_b) {
        c7_overflow = false;
      } else {
        c7_eml_switch_helper(chartInstance);
        c7_overflow = (c7_d_b > 2147483646);
      }

      if (c7_overflow) {
        c7_check_forloop_overflow_error(chartInstance, true);
      }

      for (c7_k = 1; c7_k <= c7_b_knt; c7_k++) {
        c7_beta1 *= 1.0020841800044864E-292;
      }

      *c7_alpha1 = c7_beta1;
    } else {
      c7_j_x = c7_beta1 - *c7_alpha1;
      c7_g_y = c7_beta1;
      c7_k_x = c7_j_x;
      c7_h_y = c7_g_y;
      c7_tau = c7_k_x / c7_h_y;
      c7_i_y = *c7_alpha1 - c7_beta1;
      c7_j_y = c7_i_y;
      *c7_alpha1 = 1.0 / c7_j_y;
      c7_c_eml_xscal(chartInstance, c7_nm1, *c7_alpha1, c7_x, c7_ix0);
      *c7_alpha1 = c7_beta1;
    }
  }

  return c7_tau;
}

static void c7_c_eml_xscal(SFc7_SimulationModelInstanceStruct *chartInstance,
  int32_T c7_n, real_T c7_a, real_T c7_x[4], int32_T c7_ix0)
{
  int32_T c7_b_n;
  real_T c7_b_a;
  int32_T c7_b_ix0;
  int32_T c7_c_n;
  real_T c7_c_a;
  int32_T c7_c_ix0;
  int32_T c7_d_ix0;
  int32_T c7_d_a;
  int32_T c7_c;
  int32_T c7_b;
  int32_T c7_b_c;
  int32_T c7_e_a;
  int32_T c7_b_b;
  int32_T c7_i143;
  int32_T c7_f_a;
  int32_T c7_c_b;
  int32_T c7_g_a;
  int32_T c7_d_b;
  boolean_T c7_overflow;
  int32_T c7_k;
  int32_T c7_b_k;
  c7_b_n = c7_n;
  c7_b_a = c7_a;
  c7_b_ix0 = c7_ix0;
  c7_c_threshold(chartInstance);
  c7_c_n = c7_b_n;
  c7_c_a = c7_b_a;
  c7_c_ix0 = c7_b_ix0;
  c7_d_ix0 = c7_c_ix0;
  c7_d_a = c7_c_n;
  c7_c = c7_d_a;
  c7_b = c7_c - 1;
  c7_b_c = c7_b;
  c7_e_a = c7_c_ix0;
  c7_b_b = c7_b_c;
  c7_i143 = c7_e_a + c7_b_b;
  c7_f_a = c7_d_ix0;
  c7_c_b = c7_i143;
  c7_g_a = c7_f_a;
  c7_d_b = c7_c_b;
  if (c7_g_a > c7_d_b) {
    c7_overflow = false;
  } else {
    c7_eml_switch_helper(chartInstance);
    c7_overflow = (c7_d_b > 2147483646);
  }

  if (c7_overflow) {
    c7_check_forloop_overflow_error(chartInstance, true);
  }

  for (c7_k = c7_d_ix0; c7_k <= c7_i143; c7_k++) {
    c7_b_k = c7_k;
    c7_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", c7_b_k, 1, 4, 1, 0) - 1] = c7_c_a *
      c7_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", c7_b_k, 1, 4, 1, 0) - 1];
  }
}

static real_T c7_d_eml_matlab_zlarfg(SFc7_SimulationModelInstanceStruct
  *chartInstance, real_T *c7_alpha1, real_T *c7_x)
{
  real_T c7_tau;
  (void)c7_alpha1;
  (void)c7_x;
  c7_tau = 0.0;
  c7_c_eml_xnrm2(chartInstance);
  return c7_tau;
}

static void c7_d_eml_xscal(SFc7_SimulationModelInstanceStruct *chartInstance,
  real_T *c7_x)
{
  (void)c7_x;
  c7_c_threshold(chartInstance);
}

static void c7_b_eml_xgemv(SFc7_SimulationModelInstanceStruct *chartInstance,
  int32_T c7_m, int32_T c7_n, real_T c7_A[4], int32_T c7_ia0, real_T c7_x[4],
  int32_T c7_ix0, real_T c7_y[2])
{
  int32_T c7_b_m;
  int32_T c7_b_n;
  int32_T c7_b_ix0;
  int32_T c7_c_m;
  int32_T c7_c_n;
  int32_T c7_c_ix0;
  int32_T c7_a;
  int32_T c7_mm1;
  int32_T c7_iy;
  int32_T c7_b_iy;
  int32_T c7_iac;
  int32_T c7_ix;
  real_T c7_c;
  int32_T c7_b;
  int32_T c7_i144;
  int32_T c7_b_b;
  int32_T c7_c_b;
  boolean_T c7_overflow;
  int32_T c7_ia;
  int32_T c7_b_ia;
  real_T c7_b_a;
  real_T c7_d_b;
  real_T c7_z;
  int32_T c7_c_a;
  int32_T c7_d_a;
  (void)c7_ia0;
  c7_b_m = c7_m;
  c7_b_n = c7_n;
  c7_b_ix0 = c7_ix0;
  c7_c_m = c7_b_m;
  c7_c_n = c7_b_n;
  c7_c_ix0 = c7_b_ix0;
  if (c7_c_n == 0) {
  } else {
    c7_a = c7_c_m;
    c7_mm1 = c7_a;
    c7_eml_switch_helper(chartInstance);
    c7_iy = 1;
    while (c7_iy <= 1) {
      c7_y[0] = 0.0;
      c7_iy = 2;
    }

    c7_b_iy = 1;
    c7_eml_switch_helper(chartInstance);
    c7_iac = 3;
    while (c7_iac <= 3) {
      c7_ix = c7_c_ix0;
      c7_c = 0.0;
      c7_b = c7_mm1 + 2;
      c7_i144 = c7_b;
      c7_b_b = c7_i144;
      c7_c_b = c7_b_b;
      if (3 > c7_c_b) {
        c7_overflow = false;
      } else {
        c7_eml_switch_helper(chartInstance);
        c7_overflow = (c7_c_b > 2147483646);
      }

      if (c7_overflow) {
        c7_check_forloop_overflow_error(chartInstance, true);
      }

      for (c7_ia = 3; c7_ia <= c7_i144; c7_ia++) {
        c7_b_ia = c7_ia - 1;
        c7_b_a = c7_A[c7_b_ia];
        c7_d_b = c7_x[_SFD_EML_ARRAY_BOUNDS_CHECK("", c7_ix, 1, 4, 1, 0) - 1];
        c7_z = c7_b_a * c7_d_b;
        c7_c += c7_z;
        c7_c_a = c7_ix + 1;
        c7_ix = c7_c_a;
      }

      c7_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", c7_b_iy, 1, 2, 1, 0) - 1] =
        c7_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", c7_b_iy, 1, 2, 1, 0) - 1] + c7_c;
      c7_d_a = c7_b_iy + 1;
      c7_b_iy = c7_d_a;
      c7_iac = 5;
    }
  }
}

static void c7_b_eml_xgerc(SFc7_SimulationModelInstanceStruct *chartInstance,
  int32_T c7_m, int32_T c7_n, real_T c7_alpha1, int32_T c7_ix0, real_T c7_y[2],
  real_T c7_A[4], int32_T c7_ia0)
{
  int32_T c7_b_m;
  int32_T c7_b_n;
  real_T c7_b_alpha1;
  int32_T c7_b_ix0;
  int32_T c7_c_m;
  int32_T c7_c_n;
  real_T c7_c_alpha1;
  int32_T c7_c_ix0;
  int32_T c7_d_m;
  int32_T c7_d_n;
  real_T c7_d_alpha1;
  int32_T c7_d_ix0;
  int32_T c7_e_m;
  int32_T c7_e_n;
  real_T c7_e_alpha1;
  int32_T c7_e_ix0;
  int32_T c7_ixstart;
  int32_T c7_jA;
  int32_T c7_jy;
  int32_T c7_f_n;
  int32_T c7_b;
  int32_T c7_b_b;
  boolean_T c7_overflow;
  int32_T c7_j;
  real_T c7_yjy;
  real_T c7_temp;
  int32_T c7_ix;
  int32_T c7_c_b;
  int32_T c7_i145;
  int32_T c7_a;
  int32_T c7_d_b;
  int32_T c7_i146;
  int32_T c7_b_a;
  int32_T c7_e_b;
  int32_T c7_c_a;
  int32_T c7_f_b;
  boolean_T c7_b_overflow;
  int32_T c7_ijA;
  int32_T c7_b_ijA;
  int32_T c7_d_a;
  int32_T c7_e_a;
  int32_T c7_f_a;
  (void)c7_ia0;
  c7_b_m = c7_m;
  c7_b_n = c7_n;
  c7_b_alpha1 = c7_alpha1;
  c7_b_ix0 = c7_ix0;
  c7_c_m = c7_b_m;
  c7_c_n = c7_b_n;
  c7_c_alpha1 = c7_b_alpha1;
  c7_c_ix0 = c7_b_ix0;
  c7_d_m = c7_c_m;
  c7_d_n = c7_c_n;
  c7_d_alpha1 = c7_c_alpha1;
  c7_d_ix0 = c7_c_ix0;
  c7_e_m = c7_d_m;
  c7_e_n = c7_d_n;
  c7_e_alpha1 = c7_d_alpha1;
  c7_e_ix0 = c7_d_ix0;
  if (c7_e_alpha1 == 0.0) {
  } else {
    c7_ixstart = c7_e_ix0;
    c7_jA = 2;
    c7_jy = 1;
    c7_f_n = c7_e_n;
    c7_b = c7_f_n;
    c7_b_b = c7_b;
    if (1 > c7_b_b) {
      c7_overflow = false;
    } else {
      c7_eml_switch_helper(chartInstance);
      c7_overflow = (c7_b_b > 2147483646);
    }

    if (c7_overflow) {
      c7_check_forloop_overflow_error(chartInstance, true);
    }

    c7_j = 1;
    while (c7_j <= c7_f_n) {
      c7_yjy = c7_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", c7_jy, 1, 2, 1, 0) - 1];
      if (c7_yjy != 0.0) {
        c7_temp = c7_yjy * c7_e_alpha1;
        c7_ix = c7_ixstart;
        c7_c_b = c7_jA + 1;
        c7_i145 = c7_c_b;
        c7_a = c7_e_m;
        c7_d_b = c7_jA;
        c7_i146 = c7_a + c7_d_b;
        c7_b_a = c7_i145;
        c7_e_b = c7_i146;
        c7_c_a = c7_b_a;
        c7_f_b = c7_e_b;
        if (c7_c_a > c7_f_b) {
          c7_b_overflow = false;
        } else {
          c7_eml_switch_helper(chartInstance);
          c7_b_overflow = (c7_f_b > 2147483646);
        }

        if (c7_b_overflow) {
          c7_check_forloop_overflow_error(chartInstance, true);
        }

        for (c7_ijA = c7_i145; c7_ijA <= c7_i146; c7_ijA++) {
          c7_b_ijA = c7_ijA;
          c7_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", c7_b_ijA, 1, 4, 1, 0) - 1] =
            c7_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", c7_b_ijA, 1, 4, 1, 0) - 1] +
            c7_A[_SFD_EML_ARRAY_BOUNDS_CHECK("", c7_ix, 1, 4, 1, 0) - 1] *
            c7_temp;
          c7_d_a = c7_ix + 1;
          c7_ix = c7_d_a;
        }
      }

      c7_e_a = c7_jy + 1;
      c7_jy = c7_e_a;
      c7_f_a = c7_jA + 2;
      c7_jA = c7_f_a;
      c7_j = 2;
    }
  }
}

static void init_dsm_address_info(SFc7_SimulationModelInstanceStruct
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

void sf_c7_SimulationModel_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1974679525U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(1661552579U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3743820749U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3794445873U);
}

mxArray *sf_c7_SimulationModel_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("4mVVsT5d2mOLKYXUfuIU7F");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,3,3,dataFields);

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
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
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
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,2,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(6);
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
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c7_SimulationModel_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c7_SimulationModel_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

static const mxArray *sf_get_sim_state_info_c7_SimulationModel(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x3'type','srcId','name','auxInfo'{{M[1],M[6],T\"bear\",},{M[1],M[5],T\"dist\",},{M[8],M[0],T\"is_active_c7_SimulationModel\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 3, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c7_SimulationModel_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc7_SimulationModelInstanceStruct *chartInstance;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    chartInstance = (SFc7_SimulationModelInstanceStruct *)
      chartInfo->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _SimulationModelMachineNumber_,
           7,
           1,
           1,
           0,
           7,
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
          _SFD_SET_DATA_PROPS(1,1,1,0,"robotTheta");
          _SFD_SET_DATA_PROPS(2,2,0,1,"dist");
          _SFD_SET_DATA_PROPS(3,2,0,1,"bear");
          _SFD_SET_DATA_PROPS(4,1,1,0,"targetsPos");
          _SFD_SET_DATA_PROPS(5,10,0,0,"pcam");
          _SFD_SET_DATA_PROPS(6,10,0,0,"lcam");
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
        _SFD_CV_INIT_EML(0,1,5,1,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,1428);
        _SFD_CV_INIT_EML_FCN(0,1,"camera",1428,-1,2492);
        _SFD_CV_INIT_EML_FCN(0,2,"cameraAnalytic",2492,-1,3665);
        _SFD_CV_INIT_EML_FCN(0,3,"world2robot",3665,-1,4799);
        _SFD_CV_INIT_EML_FCN(0,4,"isincamera",4799,-1,6179);
        _SFD_CV_INIT_EML_IF(0,1,0,1243,1264,1366,1425);

        {
          unsigned int dimVector[2];
          dimVector[0]= 1;
          dimVector[1]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c7_b_sf_marshallOut,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c7_d_sf_marshallOut,(MexInFcnForType)NULL);

        {
          unsigned int dimVector[1];
          dimVector[0]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_SINGLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c7_sf_marshallOut,(MexInFcnForType)
            c7_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 6;
          _SFD_SET_DATA_COMPILED_PROPS(3,SF_SINGLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c7_sf_marshallOut,(MexInFcnForType)
            c7_sf_marshallIn);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 8;
          dimVector[1]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(4,SF_SINGLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c7_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 1;
          dimVector[1]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c7_b_sf_marshallOut,(MexInFcnForType)
            c7_b_sf_marshallIn);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 1;
          dimVector[1]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c7_b_sf_marshallOut,(MexInFcnForType)
            c7_b_sf_marshallIn);
        }

        {
          real_T *c7_robotTheta;
          real_T (*c7_robotPos)[2];
          real32_T (*c7_dist)[6];
          real32_T (*c7_bear)[6];
          real32_T (*c7_targetsPos)[16];
          c7_targetsPos = (real32_T (*)[16])ssGetInputPortSignal
            (chartInstance->S, 2);
          c7_bear = (real32_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 2);
          c7_dist = (real32_T (*)[6])ssGetOutputPortSignal(chartInstance->S, 1);
          c7_robotTheta = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
          c7_robotPos = (real_T (*)[2])ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, *c7_robotPos);
          _SFD_SET_DATA_VALUE_PTR(1U, c7_robotTheta);
          _SFD_SET_DATA_VALUE_PTR(2U, *c7_dist);
          _SFD_SET_DATA_VALUE_PTR(3U, *c7_bear);
          _SFD_SET_DATA_VALUE_PTR(4U, *c7_targetsPos);
          _SFD_SET_DATA_VALUE_PTR(5U, chartInstance->c7_pcam);
          _SFD_SET_DATA_VALUE_PTR(6U, chartInstance->c7_lcam);
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
  return "1ZtBlEZkBziQt1QzED0QVG";
}

static void sf_opaque_initialize_c7_SimulationModel(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc7_SimulationModelInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c7_SimulationModel((SFc7_SimulationModelInstanceStruct*)
    chartInstanceVar);
  initialize_c7_SimulationModel((SFc7_SimulationModelInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c7_SimulationModel(void *chartInstanceVar)
{
  enable_c7_SimulationModel((SFc7_SimulationModelInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c7_SimulationModel(void *chartInstanceVar)
{
  disable_c7_SimulationModel((SFc7_SimulationModelInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c7_SimulationModel(void *chartInstanceVar)
{
  sf_gateway_c7_SimulationModel((SFc7_SimulationModelInstanceStruct*)
    chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c7_SimulationModel(SimStruct* S)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c7_SimulationModel
    ((SFc7_SimulationModelInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c7_SimulationModel();/* state var info */
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

extern void sf_internal_set_sim_state_c7_SimulationModel(SimStruct* S, const
  mxArray *st)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[3];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxDuplicateArray(st);      /* high level simctx */
  prhs[2] = (mxArray*) sf_get_sim_state_info_c7_SimulationModel();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 3, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c7_SimulationModel((SFc7_SimulationModelInstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c7_SimulationModel(SimStruct* S)
{
  return sf_internal_get_sim_state_c7_SimulationModel(S);
}

static void sf_opaque_set_sim_state_c7_SimulationModel(SimStruct* S, const
  mxArray *st)
{
  sf_internal_set_sim_state_c7_SimulationModel(S, st);
}

static void sf_opaque_terminate_c7_SimulationModel(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc7_SimulationModelInstanceStruct*) chartInstanceVar)->S;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_SimulationModel_optimization_info();
    }

    finalize_c7_SimulationModel((SFc7_SimulationModelInstanceStruct*)
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
  initSimStructsc7_SimulationModel((SFc7_SimulationModelInstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c7_SimulationModel(SimStruct *S)
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
    initialize_params_c7_SimulationModel((SFc7_SimulationModelInstanceStruct*)
      (chartInfo->chartInstance));
  }
}

static void mdlSetWorkWidths_c7_SimulationModel(SimStruct *S)
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
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,7);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(sf_get_instance_specialization(),
                infoStruct,7,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,7,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(sf_get_instance_specialization(),infoStruct,7);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,7,3);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,7,2);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=2; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 3; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,7);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(1983105440U));
  ssSetChecksum1(S,(1427598992U));
  ssSetChecksum2(S,(1370403361U));
  ssSetChecksum3(S,(1669136662U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c7_SimulationModel(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c7_SimulationModel(SimStruct *S)
{
  SFc7_SimulationModelInstanceStruct *chartInstance;
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)utMalloc(sizeof
    (ChartRunTimeInfo));
  chartInstance = (SFc7_SimulationModelInstanceStruct *)utMalloc(sizeof
    (SFc7_SimulationModelInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc7_SimulationModelInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c7_SimulationModel;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c7_SimulationModel;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c7_SimulationModel;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c7_SimulationModel;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c7_SimulationModel;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c7_SimulationModel;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c7_SimulationModel;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c7_SimulationModel;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c7_SimulationModel;
  chartInstance->chartInfo.mdlStart = mdlStart_c7_SimulationModel;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c7_SimulationModel;
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

void c7_SimulationModel_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c7_SimulationModel(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c7_SimulationModel(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c7_SimulationModel(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c7_SimulationModel_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
