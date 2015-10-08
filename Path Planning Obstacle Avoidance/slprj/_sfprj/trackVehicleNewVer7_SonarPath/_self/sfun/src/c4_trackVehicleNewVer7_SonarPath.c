/* Include files */

#include <stddef.h>
#include "blas.h"
#include "trackVehicleNewVer7_SonarPath_sfun.h"
#include "c4_trackVehicleNewVer7_SonarPath.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "trackVehicleNewVer7_SonarPath_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c_with_debugger(S, sfGlobalDebugInstanceStruct);

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization);
static void chart_debug_initialize_data_addresses(SimStruct *S);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static real_T _sfTime_;
static const char * c4_debug_family_names[25] = { "ix", "iy", "goalnum", "gx",
  "gy", "cellpath", "cellpathlen", "step", "tx", "ty", "goalcell", "mindist",
  "goalindex", "tmpdist", "nargin", "nargout", "x", "y", "heading", "goals",
  "OccGrid", "cellsize", "gridorig", "path", "pathlen" };

static const char * c4_b_debug_family_names[8] = { "nargin", "nargout", "x", "y",
  "cellsize", "gridorig", "ix", "iy" };

static const char * c4_c_debug_family_names[7] = { "nargin", "nargout", "x1",
  "y1", "x2", "y2", "dist" };

static const char * c4_d_debug_family_names[29] = { "came_from", "open",
  "g_matrix", "f_matrix", "closed", "loop_counter", "goal_x", "goal_y", "i", "j",
  "xStart", "yStart", "xNode", "yNode", "counter", "camefrom_index",
  "cf_counter", "index", "cfi", "row", "k", "temp_x", "temp_y", "new_g", "path",
  "ix", "iy", "OccGrid", "pathlen" };

static const char * c4_e_debug_family_names[8] = { "nargin", "nargout", "ix",
  "iy", "cellsize", "gridorig", "x", "y" };

/* Function Declarations */
static void initialize_c4_trackVehicleNewVer7_SonarPath
  (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance);
static void initialize_params_c4_trackVehicleNewVer7_SonarPath
  (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance);
static void enable_c4_trackVehicleNewVer7_SonarPath
  (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance);
static void disable_c4_trackVehicleNewVer7_SonarPath
  (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance);
static void c4_update_debugger_state_c4_trackVehicleNewVer7_SonarPath
  (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c4_trackVehicleNewVer7_SonarPath
  (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance);
static void set_sim_state_c4_trackVehicleNewVer7_SonarPath
  (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance, const
   mxArray *c4_st);
static void finalize_c4_trackVehicleNewVer7_SonarPath
  (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance);
static void sf_gateway_c4_trackVehicleNewVer7_SonarPath
  (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance);
static void mdl_start_c4_trackVehicleNewVer7_SonarPath
  (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance);
static void c4_chartstep_c4_trackVehicleNewVer7_SonarPath
  (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance);
static void initSimStructsc4_trackVehicleNewVer7_SonarPath
  (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c4_machineNumber, uint32_T
  c4_chartNumber, uint32_T c4_instanceNumber);
static const mxArray *c4_sf_marshallOut(void *chartInstanceVoid, void *c4_inData);
static real_T c4_emlrt_marshallIn
  (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance, const
   mxArray *c4_b_pathlen, const char_T *c4_identifier);
static real_T c4_b_emlrt_marshallIn
  (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance, const
   mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId);
static void c4_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData);
static const mxArray *c4_b_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData);
static void c4_c_emlrt_marshallIn
  (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance, const
   mxArray *c4_b_path, const char_T *c4_identifier, real_T c4_b_y[1800]);
static void c4_d_emlrt_marshallIn
  (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance, const
   mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId, real_T c4_b_y[1800]);
static void c4_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData);
static const mxArray *c4_c_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData);
static const mxArray *c4_d_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData);
static void c4_e_emlrt_marshallIn
  (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance, const
   mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId, real_T c4_b_y[2]);
static void c4_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData);
static const mxArray *c4_e_sf_marshallOut(void *chartInstanceVoid, real_T
  c4_inData_data[], int32_T c4_inData_sizes[2]);
static void c4_f_emlrt_marshallIn
  (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance, const
   mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId, real_T c4_y_data[],
   int32_T c4_y_sizes[2]);
static void c4_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, real_T c4_outData_data[], int32_T
  c4_outData_sizes[2]);
static void c4_g_emlrt_marshallIn
  (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance, const
   mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId, real_T c4_b_y[900]);
static void c4_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData);
static void c4_pos2cell(SFc4_trackVehicleNewVer7_SonarPathInstanceStruct
  *chartInstance, real_T c4_b_x, real_T c4_b_y, real_T c4_b_cellsize[2], real_T
  c4_b_gridorig[2], real_T *c4_ix, real_T *c4_iy);
static void c4_eml_scalar_eg(SFc4_trackVehicleNewVer7_SonarPathInstanceStruct
  *chartInstance);
static void c4_dimagree(SFc4_trackVehicleNewVer7_SonarPathInstanceStruct
  *chartInstance);
static void c4_planPath(SFc4_trackVehicleNewVer7_SonarPathInstanceStruct
  *chartInstance, real_T c4_ix, real_T c4_iy, real_T c4_b_OccGrid[900], real_T
  c4_path_data[], int32_T c4_path_sizes[2], real_T *c4_b_pathlen);
static real_T c4_distance(SFc4_trackVehicleNewVer7_SonarPathInstanceStruct
  *chartInstance, real_T c4_x1, real_T c4_y1, real_T c4_x2, real_T c4_y2);
static real_T c4_mpower(SFc4_trackVehicleNewVer7_SonarPathInstanceStruct
  *chartInstance, real_T c4_a);
static real_T c4_sqrt(SFc4_trackVehicleNewVer7_SonarPathInstanceStruct
                      *chartInstance, real_T c4_b_x);
static void c4_eml_error(SFc4_trackVehicleNewVer7_SonarPathInstanceStruct
  *chartInstance);
static void c4_eml_switch_helper
  (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance);
static boolean_T c4_allinrange(SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *
  chartInstance, int32_T c4_b_x, real_T c4_lo, int32_T c4_hi);
static void c4_flipud(SFc4_trackVehicleNewVer7_SonarPathInstanceStruct
                      *chartInstance, real_T c4_x_data[], int32_T c4_x_sizes[2],
                      real_T c4_b_x_data[], int32_T c4_b_x_sizes[2]);
static void c4_cell2pos(SFc4_trackVehicleNewVer7_SonarPathInstanceStruct
  *chartInstance, real_T c4_ix, real_T c4_iy, real_T c4_b_cellsize[2], real_T
  c4_b_gridorig[2], real_T *c4_b_x, real_T *c4_b_y);
static const mxArray *c4_emlrt_marshallOut
  (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance, const real_T
   c4_u_data[], const int32_T c4_u_sizes[2]);
static const mxArray *c4_b_emlrt_marshallOut
  (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance, const real_T
   c4_u[12]);
static const mxArray *c4_c_emlrt_marshallOut
  (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance, const real_T
   c4_u);
static const mxArray *c4_d_emlrt_marshallOut
  (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance, const char_T
   c4_u[7]);
static const mxArray *c4_e_emlrt_marshallOut
  (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance, const char_T
   c4_u[22]);
static const mxArray *c4_f_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData);
static int32_T c4_h_emlrt_marshallIn
  (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance, const
   mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId);
static void c4_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData);
static uint8_T c4_i_emlrt_marshallIn
  (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance, const
   mxArray *c4_b_is_active_c4_trackVehicleNewVer7_SonarPath, const char_T
   *c4_identifier);
static uint8_T c4_j_emlrt_marshallIn
  (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance, const
   mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId);
static void c4_b_sqrt(SFc4_trackVehicleNewVer7_SonarPathInstanceStruct
                      *chartInstance, real_T *c4_b_x);
static void c4_b_flipud(SFc4_trackVehicleNewVer7_SonarPathInstanceStruct
  *chartInstance, real_T c4_x_data[], int32_T c4_x_sizes[2]);
static int32_T c4_div_s32(SFc4_trackVehicleNewVer7_SonarPathInstanceStruct
  *chartInstance, int32_T c4_numerator, int32_T c4_denominator);
static void init_dsm_address_info
  (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance);
static void init_simulink_io_address
  (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance);

/* Function Definitions */
static void initialize_c4_trackVehicleNewVer7_SonarPath
  (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance)
{
  if (sf_is_first_init_cond(chartInstance->S)) {
    initSimStructsc4_trackVehicleNewVer7_SonarPath(chartInstance);
    chart_debug_initialize_data_addresses(chartInstance->S);
  }

  chartInstance->c4_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c4_is_active_c4_trackVehicleNewVer7_SonarPath = 0U;
}

static void initialize_params_c4_trackVehicleNewVer7_SonarPath
  (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void enable_c4_trackVehicleNewVer7_SonarPath
  (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c4_trackVehicleNewVer7_SonarPath
  (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c4_update_debugger_state_c4_trackVehicleNewVer7_SonarPath
  (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c4_trackVehicleNewVer7_SonarPath
  (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance)
{
  const mxArray *c4_st;
  const mxArray *c4_b_y = NULL;
  const mxArray *c4_c_y = NULL;
  real_T c4_hoistedGlobal;
  real_T c4_u;
  const mxArray *c4_d_y = NULL;
  uint8_T c4_b_hoistedGlobal;
  uint8_T c4_b_u;
  const mxArray *c4_e_y = NULL;
  c4_st = NULL;
  c4_st = NULL;
  c4_b_y = NULL;
  sf_mex_assign(&c4_b_y, sf_mex_createcellmatrix(3, 1), false);
  c4_c_y = NULL;
  sf_mex_assign(&c4_c_y, sf_mex_create("y", *chartInstance->c4_path, 0, 0U, 1U,
    0U, 2, 900, 2), false);
  sf_mex_setcell(c4_b_y, 0, c4_c_y);
  c4_hoistedGlobal = *chartInstance->c4_pathlen;
  c4_u = c4_hoistedGlobal;
  c4_d_y = NULL;
  sf_mex_assign(&c4_d_y, sf_mex_create("y", &c4_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c4_b_y, 1, c4_d_y);
  c4_b_hoistedGlobal =
    chartInstance->c4_is_active_c4_trackVehicleNewVer7_SonarPath;
  c4_b_u = c4_b_hoistedGlobal;
  c4_e_y = NULL;
  sf_mex_assign(&c4_e_y, sf_mex_create("y", &c4_b_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c4_b_y, 2, c4_e_y);
  sf_mex_assign(&c4_st, c4_b_y, false);
  return c4_st;
}

static void set_sim_state_c4_trackVehicleNewVer7_SonarPath
  (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance, const
   mxArray *c4_st)
{
  const mxArray *c4_u;
  real_T c4_dv0[1800];
  int32_T c4_i0;
  chartInstance->c4_doneDoubleBufferReInit = true;
  c4_u = sf_mex_dup(c4_st);
  c4_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell("path", c4_u, 0)),
                        "path", c4_dv0);
  for (c4_i0 = 0; c4_i0 < 1800; c4_i0++) {
    (*chartInstance->c4_path)[c4_i0] = c4_dv0[c4_i0];
  }

  *chartInstance->c4_pathlen = c4_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell("pathlen", c4_u, 1)), "pathlen");
  chartInstance->c4_is_active_c4_trackVehicleNewVer7_SonarPath =
    c4_i_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(
    "is_active_c4_trackVehicleNewVer7_SonarPath", c4_u, 2)),
    "is_active_c4_trackVehicleNewVer7_SonarPath");
  sf_mex_destroy(&c4_u);
  c4_update_debugger_state_c4_trackVehicleNewVer7_SonarPath(chartInstance);
  sf_mex_destroy(&c4_st);
}

static void finalize_c4_trackVehicleNewVer7_SonarPath
  (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c4_trackVehicleNewVer7_SonarPath
  (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance)
{
  int32_T c4_i1;
  int32_T c4_i2;
  int32_T c4_i3;
  int32_T c4_i4;
  int32_T c4_i5;
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 3U, chartInstance->c4_sfEvent);
  for (c4_i1 = 0; c4_i1 < 2; c4_i1++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c4_gridorig)[c4_i1], 6U, 1U, 0U,
                          chartInstance->c4_sfEvent, false);
  }

  for (c4_i2 = 0; c4_i2 < 2; c4_i2++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c4_cellsize)[c4_i2], 5U, 1U, 0U,
                          chartInstance->c4_sfEvent, false);
  }

  for (c4_i3 = 0; c4_i3 < 900; c4_i3++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c4_OccGrid)[c4_i3], 4U, 1U, 0U,
                          chartInstance->c4_sfEvent, false);
  }

  for (c4_i4 = 0; c4_i4 < 2; c4_i4++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c4_goals)[c4_i4], 3U, 1U, 0U,
                          chartInstance->c4_sfEvent, false);
  }

  _SFD_DATA_RANGE_CHECK(*chartInstance->c4_heading, 2U, 1U, 0U,
                        chartInstance->c4_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c4_y, 1U, 1U, 0U,
                        chartInstance->c4_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c4_x, 0U, 1U, 0U,
                        chartInstance->c4_sfEvent, false);
  chartInstance->c4_sfEvent = CALL_EVENT;
  c4_chartstep_c4_trackVehicleNewVer7_SonarPath(chartInstance);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY
    (_trackVehicleNewVer7_SonarPathMachineNumber_, chartInstance->chartNumber,
     chartInstance->instanceNumber);
  for (c4_i5 = 0; c4_i5 < 1800; c4_i5++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c4_path)[c4_i5], 7U, 1U, 0U,
                          chartInstance->c4_sfEvent, false);
  }

  _SFD_DATA_RANGE_CHECK(*chartInstance->c4_pathlen, 8U, 1U, 0U,
                        chartInstance->c4_sfEvent, false);
}

static void mdl_start_c4_trackVehicleNewVer7_SonarPath
  (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c4_chartstep_c4_trackVehicleNewVer7_SonarPath
  (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance)
{
  real_T c4_hoistedGlobal;
  real_T c4_b_hoistedGlobal;
  real_T c4_c_hoistedGlobal;
  real_T c4_b_x;
  real_T c4_b_y;
  real_T c4_b_heading;
  int32_T c4_i6;
  real_T c4_b_goals[2];
  int32_T c4_i7;
  real_T c4_b_OccGrid[900];
  int32_T c4_i8;
  real_T c4_b_cellsize[2];
  int32_T c4_i9;
  real_T c4_b_gridorig[2];
  uint32_T c4_debug_family_var_map[25];
  real_T c4_ix;
  real_T c4_iy;
  real_T c4_goalnum;
  real_T c4_gx;
  real_T c4_gy;
  int32_T c4_cellpath_sizes[2];
  real_T c4_cellpath_data[1800];
  real_T c4_cellpathlen;
  real_T c4_step;
  real_T c4_tx;
  real_T c4_ty;
  real_T c4_goalcell[2];
  real_T c4_mindist;
  real_T c4_goalindex;
  real_T c4_tmpdist;
  real_T c4_nargin = 7.0;
  real_T c4_nargout = 2.0;
  real_T c4_b_path[1800];
  real_T c4_b_pathlen;
  int32_T c4_i10;
  int32_T c4_i11;
  real_T c4_c_cellsize[2];
  int32_T c4_i12;
  real_T c4_c_gridorig[2];
  real_T c4_b_iy;
  real_T c4_b_ix;
  int32_T c4_i13;
  real_T c4_d_cellsize[2];
  int32_T c4_i14;
  real_T c4_d_gridorig[2];
  real_T c4_b_gy;
  real_T c4_b_gx;
  int32_T c4_i15;
  real_T c4_c_OccGrid[900];
  real_T c4_b_cellpathlen;
  int32_T c4_b_cellpath_sizes[2];
  real_T c4_b_cellpath_data[1800];
  int32_T c4_cellpath;
  int32_T c4_b_cellpath;
  int32_T c4_loop_ub;
  int32_T c4_i16;
  real_T c4_c_cellpathlen;
  int32_T c4_i17;
  int32_T c4_b_step;
  int32_T c4_i18;
  real_T c4_e_cellsize[2];
  int32_T c4_i19;
  real_T c4_e_gridorig[2];
  real_T c4_b_ty;
  real_T c4_b_tx;
  int32_T c4_c_pathlen;
  real_T c4_d0;
  int32_T c4_d_pathlen;
  boolean_T c4_b0;
  boolean_T c4_b1;
  boolean_T c4_b2;
  int32_T c4_i20;
  int32_T c4_path_sizes[2];
  int32_T c4_i21;
  int32_T c4_b_loop_ub;
  int32_T c4_i22;
  real_T c4_path_data[1798];
  int32_T c4_i23;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 3U, chartInstance->c4_sfEvent);
  c4_hoistedGlobal = *chartInstance->c4_x;
  c4_b_hoistedGlobal = *chartInstance->c4_y;
  c4_c_hoistedGlobal = *chartInstance->c4_heading;
  c4_b_x = c4_hoistedGlobal;
  c4_b_y = c4_b_hoistedGlobal;
  c4_b_heading = c4_c_hoistedGlobal;
  for (c4_i6 = 0; c4_i6 < 2; c4_i6++) {
    c4_b_goals[c4_i6] = (*chartInstance->c4_goals)[c4_i6];
  }

  for (c4_i7 = 0; c4_i7 < 900; c4_i7++) {
    c4_b_OccGrid[c4_i7] = (*chartInstance->c4_OccGrid)[c4_i7];
  }

  for (c4_i8 = 0; c4_i8 < 2; c4_i8++) {
    c4_b_cellsize[c4_i8] = (*chartInstance->c4_cellsize)[c4_i8];
  }

  for (c4_i9 = 0; c4_i9 < 2; c4_i9++) {
    c4_b_gridorig[c4_i9] = (*chartInstance->c4_gridorig)[c4_i9];
  }

  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 25U, 25U, c4_debug_family_names,
    c4_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_ix, 0U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_iy, 1U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c4_goalnum, 2U, c4_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_gx, 3U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_gy, 4U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_DYN_IMPORTABLE(c4_cellpath_data, (const int32_T *)
    &c4_cellpath_sizes, NULL, 0, 5, (void *)c4_e_sf_marshallOut, (void *)
    c4_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_cellpathlen, 6U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_step, 7U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_tx, 8U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_ty, 9U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c4_goalcell, 10U, c4_c_sf_marshallOut,
    c4_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_mindist, 11U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_goalindex, 12U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_tmpdist, 13U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_nargin, 14U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_nargout, 15U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c4_b_x, 16U, c4_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c4_b_y, 17U, c4_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c4_b_heading, 18U, c4_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c4_b_goals, 19U, c4_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c4_b_OccGrid, 20U, c4_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c4_b_cellsize, 21U, c4_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c4_b_gridorig, 22U, c4_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c4_b_path, 23U, c4_b_sf_marshallOut,
    c4_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_b_pathlen, 24U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 9);
  for (c4_i10 = 0; c4_i10 < 1800; c4_i10++) {
    c4_b_path[c4_i10] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 10);
  c4_b_pathlen = 0.0;
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 12);
  c4_ix = 0.0;
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 13);
  c4_iy = 0.0;
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 14);
  for (c4_i11 = 0; c4_i11 < 2; c4_i11++) {
    c4_c_cellsize[c4_i11] = c4_b_cellsize[c4_i11];
  }

  for (c4_i12 = 0; c4_i12 < 2; c4_i12++) {
    c4_c_gridorig[c4_i12] = c4_b_gridorig[c4_i12];
  }

  c4_pos2cell(chartInstance, c4_b_x, c4_b_y, c4_c_cellsize, c4_c_gridorig,
              &c4_b_ix, &c4_b_iy);
  c4_ix = c4_b_ix;
  c4_iy = c4_b_iy;
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 17);
  CV_RELATIONAL_EVAL(4U, 0U, 0, 1.0, 0.0, -1, 4U, 1);
  CV_EML_IF(0, 1, 0, true);
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 18);
  c4_goalnum = 1.0;
  CV_EML_FOR(0, 1, 0, 1);
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 19);
  for (c4_i13 = 0; c4_i13 < 2; c4_i13++) {
    c4_d_cellsize[c4_i13] = c4_b_cellsize[c4_i13];
  }

  for (c4_i14 = 0; c4_i14 < 2; c4_i14++) {
    c4_d_gridorig[c4_i14] = c4_b_gridorig[c4_i14];
  }

  c4_pos2cell(chartInstance, c4_b_goals[0], c4_b_goals[1], c4_d_cellsize,
              c4_d_gridorig, &c4_b_gx, &c4_b_gy);
  c4_gx = c4_b_gx;
  c4_gy = c4_b_gy;
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 20);
  c4_b_OccGrid[(_SFD_EML_ARRAY_BOUNDS_CHECK("OccGrid", (int32_T)
    _SFD_INTEGER_CHECK("gx", c4_gx), 1, 30, 1, 0) + 30 *
                (_SFD_EML_ARRAY_BOUNDS_CHECK("OccGrid", (int32_T)
    _SFD_INTEGER_CHECK("gy", c4_gy), 1, 30, 2, 0) - 1)) - 1] = 3.0;
  CV_EML_FOR(0, 1, 0, 0);
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 24);
  for (c4_i15 = 0; c4_i15 < 900; c4_i15++) {
    c4_c_OccGrid[c4_i15] = c4_b_OccGrid[c4_i15];
  }

  c4_planPath(chartInstance, c4_ix, c4_iy, c4_c_OccGrid, c4_b_cellpath_data,
              c4_b_cellpath_sizes, &c4_b_cellpathlen);
  c4_cellpath_sizes[0] = c4_b_cellpath_sizes[0];
  c4_cellpath_sizes[1] = 2;
  c4_cellpath = c4_cellpath_sizes[0];
  c4_b_cellpath = c4_cellpath_sizes[1];
  c4_loop_ub = c4_b_cellpath_sizes[0] * c4_b_cellpath_sizes[1] - 1;
  for (c4_i16 = 0; c4_i16 <= c4_loop_ub; c4_i16++) {
    c4_cellpath_data[c4_i16] = c4_b_cellpath_data[c4_i16];
  }

  c4_cellpathlen = c4_b_cellpathlen;
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 27);
  c4_b_path[0] = c4_b_x;
  c4_b_path[900] = c4_b_y;
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 28);
  c4_b_pathlen = 1.0;
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 29);
  if (CV_EML_IF(0, 1, 1, CV_RELATIONAL_EVAL(4U, 0U, 1, c4_cellpathlen, 0.0, -1,
        4U, c4_cellpathlen > 0.0))) {
    _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 31);
    c4_c_cellpathlen = c4_cellpathlen;
    c4_i17 = (int32_T)c4_c_cellpathlen - 1;
    c4_step = 1.0;
    c4_b_step = 0;
    while (c4_b_step <= c4_i17) {
      c4_step = 1.0 + (real_T)c4_b_step;
      CV_EML_FOR(0, 1, 1, 1);
      _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 32);
      for (c4_i18 = 0; c4_i18 < 2; c4_i18++) {
        c4_e_cellsize[c4_i18] = c4_b_cellsize[c4_i18];
      }

      for (c4_i19 = 0; c4_i19 < 2; c4_i19++) {
        c4_e_gridorig[c4_i19] = c4_b_gridorig[c4_i19];
      }

      c4_cell2pos(chartInstance, c4_cellpath_data[_SFD_EML_ARRAY_BOUNDS_CHECK(
        "cellpath", (int32_T)c4_step, 1, c4_cellpath_sizes[0], 1, 0) - 1],
                  c4_cellpath_data[(_SFD_EML_ARRAY_BOUNDS_CHECK("cellpath",
        (int32_T)c4_step, 1, c4_cellpath_sizes[0], 1, 0) + c4_cellpath_sizes[0])
                  - 1], c4_e_cellsize, c4_e_gridorig, &c4_b_tx, &c4_b_ty);
      c4_tx = c4_b_tx;
      c4_ty = c4_b_ty;
      _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 33);
      c4_c_pathlen = _SFD_EML_ARRAY_BOUNDS_CHECK("path", (int32_T)(c4_b_pathlen
        + 1.0), 1, 900, 1, 0) - 1;
      c4_b_path[c4_c_pathlen] = c4_tx;
      c4_b_path[900 + c4_c_pathlen] = c4_ty;
      _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 34);
      c4_b_pathlen++;
      c4_b_step++;
      _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
    }

    CV_EML_FOR(0, 1, 1, 0);
    _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 38);
    c4_goalcell[0] = c4_b_path[_SFD_EML_ARRAY_BOUNDS_CHECK("path", (int32_T)
      c4_b_pathlen, 1, 900, 1, 0) - 1];
    c4_goalcell[1] = c4_b_path[_SFD_EML_ARRAY_BOUNDS_CHECK("path", (int32_T)
      c4_b_pathlen, 1, 900, 1, 0) + 899];
    _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 39);
    c4_mindist = rtInf;
    _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 40);
    c4_goalindex = -1.0;
    _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 41);
    c4_goalnum = 1.0;
    CV_EML_FOR(0, 1, 2, 1);
    _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 42);
    c4_d0 = (c4_goalcell[0] - c4_b_goals[0]) * (c4_goalcell[0] - c4_b_goals[0])
      + (c4_goalcell[1] - c4_b_goals[1]) * (c4_goalcell[1] - c4_b_goals[1]);
    c4_b_sqrt(chartInstance, &c4_d0);
    c4_tmpdist = c4_d0;
    _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 43);
    if (CV_EML_IF(0, 1, 2, CV_RELATIONAL_EVAL(4U, 0U, 2, c4_tmpdist, rtInf, -1,
          2U, c4_tmpdist < rtInf))) {
      _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 44);
      c4_mindist = c4_tmpdist;
      _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 45);
      c4_goalindex = 1.0;
    }

    CV_EML_FOR(0, 1, 2, 0);
    _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 48);
    (real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("goals", (int32_T)c4_goalindex, 1, 1, 1,
      0);
    (real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("goals", (int32_T)c4_goalindex, 1, 1, 1,
      0);
    c4_d_pathlen = _SFD_EML_ARRAY_BOUNDS_CHECK("path", (int32_T)(c4_b_pathlen +
      1.0), 1, 900, 1, 0) - 1;
    c4_b_path[c4_d_pathlen] = c4_b_goals[0];
    c4_b_path[900 + c4_d_pathlen] = c4_b_goals[1];
    _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 49);
    c4_b_pathlen++;
  }

  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 52);
  c4_b_pathlen -= 2.0;
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 53);
  c4_b0 = (1.0 > c4_b_pathlen);
  c4_b1 = c4_b0;
  c4_b2 = c4_b1;
  if (c4_b2) {
    c4_i20 = 0;
  } else {
    c4_i20 = _SFD_EML_ARRAY_BOUNDS_CHECK("path", (int32_T)c4_b_pathlen, 1, 900,
      0, 0);
  }

  sf_mex_printf("%s =\\n", "ans");
  c4_path_sizes[0] = c4_i20;
  c4_path_sizes[1] = 2;
  for (c4_i21 = 0; c4_i21 < 2; c4_i21++) {
    c4_b_loop_ub = c4_i20 - 1;
    for (c4_i22 = 0; c4_i22 <= c4_b_loop_ub; c4_i22++) {
      c4_path_data[c4_i22 + c4_path_sizes[0] * c4_i21] = c4_b_path[c4_i22 + 900 *
        c4_i21];
    }
  }

  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "disp", 0U, 1U, 14,
                    c4_emlrt_marshallOut(chartInstance, c4_path_data,
    c4_path_sizes));
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, -53);
  _SFD_SYMBOL_SCOPE_POP();
  for (c4_i23 = 0; c4_i23 < 1800; c4_i23++) {
    (*chartInstance->c4_path)[c4_i23] = c4_b_path[c4_i23];
  }

  *chartInstance->c4_pathlen = c4_b_pathlen;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 3U, chartInstance->c4_sfEvent);
}

static void initSimStructsc4_trackVehicleNewVer7_SonarPath
  (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void init_script_number_translation(uint32_T c4_machineNumber, uint32_T
  c4_chartNumber, uint32_T c4_instanceNumber)
{
  (void)c4_machineNumber;
  (void)c4_chartNumber;
  (void)c4_instanceNumber;
}

static const mxArray *c4_sf_marshallOut(void *chartInstanceVoid, void *c4_inData)
{
  const mxArray *c4_mxArrayOutData = NULL;
  real_T c4_u;
  const mxArray *c4_b_y = NULL;
  SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance;
  chartInstance = (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *)
    chartInstanceVoid;
  c4_mxArrayOutData = NULL;
  c4_u = *(real_T *)c4_inData;
  c4_b_y = NULL;
  sf_mex_assign(&c4_b_y, sf_mex_create("y", &c4_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c4_mxArrayOutData, c4_b_y, false);
  return c4_mxArrayOutData;
}

static real_T c4_emlrt_marshallIn
  (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance, const
   mxArray *c4_b_pathlen, const char_T *c4_identifier)
{
  real_T c4_b_y;
  emlrtMsgIdentifier c4_thisId;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_b_y = c4_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_b_pathlen),
    &c4_thisId);
  sf_mex_destroy(&c4_b_pathlen);
  return c4_b_y;
}

static real_T c4_b_emlrt_marshallIn
  (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance, const
   mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId)
{
  real_T c4_b_y;
  real_T c4_d1;
  (void)chartInstance;
  sf_mex_import(c4_parentId, sf_mex_dup(c4_u), &c4_d1, 1, 0, 0U, 0, 0U, 0);
  c4_b_y = c4_d1;
  sf_mex_destroy(&c4_u);
  return c4_b_y;
}

static void c4_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData)
{
  const mxArray *c4_b_pathlen;
  const char_T *c4_identifier;
  emlrtMsgIdentifier c4_thisId;
  real_T c4_b_y;
  SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance;
  chartInstance = (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *)
    chartInstanceVoid;
  c4_b_pathlen = sf_mex_dup(c4_mxArrayInData);
  c4_identifier = c4_varName;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_b_y = c4_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_b_pathlen),
    &c4_thisId);
  sf_mex_destroy(&c4_b_pathlen);
  *(real_T *)c4_outData = c4_b_y;
  sf_mex_destroy(&c4_mxArrayInData);
}

static const mxArray *c4_b_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData)
{
  const mxArray *c4_mxArrayOutData = NULL;
  int32_T c4_i24;
  int32_T c4_i25;
  int32_T c4_i26;
  real_T c4_u[1800];
  const mxArray *c4_b_y = NULL;
  SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance;
  chartInstance = (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *)
    chartInstanceVoid;
  c4_mxArrayOutData = NULL;
  c4_i24 = 0;
  for (c4_i25 = 0; c4_i25 < 2; c4_i25++) {
    for (c4_i26 = 0; c4_i26 < 900; c4_i26++) {
      c4_u[c4_i26 + c4_i24] = (*(real_T (*)[1800])c4_inData)[c4_i26 + c4_i24];
    }

    c4_i24 += 900;
  }

  c4_b_y = NULL;
  sf_mex_assign(&c4_b_y, sf_mex_create("y", c4_u, 0, 0U, 1U, 0U, 2, 900, 2),
                false);
  sf_mex_assign(&c4_mxArrayOutData, c4_b_y, false);
  return c4_mxArrayOutData;
}

static void c4_c_emlrt_marshallIn
  (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance, const
   mxArray *c4_b_path, const char_T *c4_identifier, real_T c4_b_y[1800])
{
  emlrtMsgIdentifier c4_thisId;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_b_path), &c4_thisId, c4_b_y);
  sf_mex_destroy(&c4_b_path);
}

static void c4_d_emlrt_marshallIn
  (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance, const
   mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId, real_T c4_b_y[1800])
{
  real_T c4_dv1[1800];
  int32_T c4_i27;
  (void)chartInstance;
  sf_mex_import(c4_parentId, sf_mex_dup(c4_u), c4_dv1, 1, 0, 0U, 1, 0U, 2, 900,
                2);
  for (c4_i27 = 0; c4_i27 < 1800; c4_i27++) {
    c4_b_y[c4_i27] = c4_dv1[c4_i27];
  }

  sf_mex_destroy(&c4_u);
}

static void c4_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData)
{
  const mxArray *c4_b_path;
  const char_T *c4_identifier;
  emlrtMsgIdentifier c4_thisId;
  real_T c4_b_y[1800];
  int32_T c4_i28;
  int32_T c4_i29;
  int32_T c4_i30;
  SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance;
  chartInstance = (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *)
    chartInstanceVoid;
  c4_b_path = sf_mex_dup(c4_mxArrayInData);
  c4_identifier = c4_varName;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_b_path), &c4_thisId, c4_b_y);
  sf_mex_destroy(&c4_b_path);
  c4_i28 = 0;
  for (c4_i29 = 0; c4_i29 < 2; c4_i29++) {
    for (c4_i30 = 0; c4_i30 < 900; c4_i30++) {
      (*(real_T (*)[1800])c4_outData)[c4_i30 + c4_i28] = c4_b_y[c4_i30 + c4_i28];
    }

    c4_i28 += 900;
  }

  sf_mex_destroy(&c4_mxArrayInData);
}

static const mxArray *c4_c_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData)
{
  const mxArray *c4_mxArrayOutData = NULL;
  int32_T c4_i31;
  real_T c4_u[2];
  const mxArray *c4_b_y = NULL;
  SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance;
  chartInstance = (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *)
    chartInstanceVoid;
  c4_mxArrayOutData = NULL;
  for (c4_i31 = 0; c4_i31 < 2; c4_i31++) {
    c4_u[c4_i31] = (*(real_T (*)[2])c4_inData)[c4_i31];
  }

  c4_b_y = NULL;
  sf_mex_assign(&c4_b_y, sf_mex_create("y", c4_u, 0, 0U, 1U, 0U, 2, 1, 2), false);
  sf_mex_assign(&c4_mxArrayOutData, c4_b_y, false);
  return c4_mxArrayOutData;
}

static const mxArray *c4_d_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData)
{
  const mxArray *c4_mxArrayOutData = NULL;
  int32_T c4_i32;
  int32_T c4_i33;
  int32_T c4_i34;
  real_T c4_u[900];
  const mxArray *c4_b_y = NULL;
  SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance;
  chartInstance = (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *)
    chartInstanceVoid;
  c4_mxArrayOutData = NULL;
  c4_i32 = 0;
  for (c4_i33 = 0; c4_i33 < 30; c4_i33++) {
    for (c4_i34 = 0; c4_i34 < 30; c4_i34++) {
      c4_u[c4_i34 + c4_i32] = (*(real_T (*)[900])c4_inData)[c4_i34 + c4_i32];
    }

    c4_i32 += 30;
  }

  c4_b_y = NULL;
  sf_mex_assign(&c4_b_y, sf_mex_create("y", c4_u, 0, 0U, 1U, 0U, 2, 30, 30),
                false);
  sf_mex_assign(&c4_mxArrayOutData, c4_b_y, false);
  return c4_mxArrayOutData;
}

static void c4_e_emlrt_marshallIn
  (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance, const
   mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId, real_T c4_b_y[2])
{
  real_T c4_dv2[2];
  int32_T c4_i35;
  (void)chartInstance;
  sf_mex_import(c4_parentId, sf_mex_dup(c4_u), c4_dv2, 1, 0, 0U, 1, 0U, 2, 1, 2);
  for (c4_i35 = 0; c4_i35 < 2; c4_i35++) {
    c4_b_y[c4_i35] = c4_dv2[c4_i35];
  }

  sf_mex_destroy(&c4_u);
}

static void c4_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData)
{
  const mxArray *c4_goalcell;
  const char_T *c4_identifier;
  emlrtMsgIdentifier c4_thisId;
  real_T c4_b_y[2];
  int32_T c4_i36;
  SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance;
  chartInstance = (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *)
    chartInstanceVoid;
  c4_goalcell = sf_mex_dup(c4_mxArrayInData);
  c4_identifier = c4_varName;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_goalcell), &c4_thisId,
                        c4_b_y);
  sf_mex_destroy(&c4_goalcell);
  for (c4_i36 = 0; c4_i36 < 2; c4_i36++) {
    (*(real_T (*)[2])c4_outData)[c4_i36] = c4_b_y[c4_i36];
  }

  sf_mex_destroy(&c4_mxArrayInData);
}

static const mxArray *c4_e_sf_marshallOut(void *chartInstanceVoid, real_T
  c4_inData_data[], int32_T c4_inData_sizes[2])
{
  const mxArray *c4_mxArrayOutData = NULL;
  int32_T c4_u_sizes[2];
  int32_T c4_u;
  int32_T c4_b_u;
  int32_T c4_loop_ub;
  int32_T c4_i37;
  real_T c4_u_data[1800];
  const mxArray *c4_b_y = NULL;
  SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance;
  chartInstance = (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *)
    chartInstanceVoid;
  c4_mxArrayOutData = NULL;
  c4_u_sizes[0] = c4_inData_sizes[0];
  c4_u_sizes[1] = 2;
  c4_u = c4_u_sizes[0];
  c4_b_u = c4_u_sizes[1];
  c4_loop_ub = c4_inData_sizes[0] * c4_inData_sizes[1] - 1;
  for (c4_i37 = 0; c4_i37 <= c4_loop_ub; c4_i37++) {
    c4_u_data[c4_i37] = c4_inData_data[c4_i37];
  }

  c4_b_y = NULL;
  sf_mex_assign(&c4_b_y, sf_mex_create("y", c4_u_data, 0, 0U, 1U, 0U, 2,
    c4_u_sizes[0], c4_u_sizes[1]), false);
  sf_mex_assign(&c4_mxArrayOutData, c4_b_y, false);
  return c4_mxArrayOutData;
}

static void c4_f_emlrt_marshallIn
  (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance, const
   mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId, real_T c4_y_data[],
   int32_T c4_y_sizes[2])
{
  int32_T c4_i38;
  uint32_T c4_uv0[2];
  int32_T c4_i39;
  static boolean_T c4_bv0[2] = { true, false };

  boolean_T c4_bv1[2];
  int32_T c4_tmp_sizes[2];
  real_T c4_tmp_data[1800];
  int32_T c4_b_y;
  int32_T c4_c_y;
  int32_T c4_loop_ub;
  int32_T c4_i40;
  (void)chartInstance;
  for (c4_i38 = 0; c4_i38 < 2; c4_i38++) {
    c4_uv0[c4_i38] = 900U + (uint32_T)(-898 * c4_i38);
  }

  for (c4_i39 = 0; c4_i39 < 2; c4_i39++) {
    c4_bv1[c4_i39] = c4_bv0[c4_i39];
  }

  sf_mex_import_vs(c4_parentId, sf_mex_dup(c4_u), c4_tmp_data, 1, 0, 0U, 1, 0U,
                   2, c4_bv1, c4_uv0, c4_tmp_sizes);
  c4_y_sizes[0] = c4_tmp_sizes[0];
  c4_y_sizes[1] = 2;
  c4_b_y = c4_y_sizes[0];
  c4_c_y = c4_y_sizes[1];
  c4_loop_ub = c4_tmp_sizes[0] * c4_tmp_sizes[1] - 1;
  for (c4_i40 = 0; c4_i40 <= c4_loop_ub; c4_i40++) {
    c4_y_data[c4_i40] = c4_tmp_data[c4_i40];
  }

  sf_mex_destroy(&c4_u);
}

static void c4_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, real_T c4_outData_data[], int32_T
  c4_outData_sizes[2])
{
  const mxArray *c4_cellpath;
  const char_T *c4_identifier;
  emlrtMsgIdentifier c4_thisId;
  int32_T c4_y_sizes[2];
  real_T c4_y_data[1800];
  int32_T c4_i41;
  int32_T c4_loop_ub;
  int32_T c4_i42;
  SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance;
  chartInstance = (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *)
    chartInstanceVoid;
  c4_cellpath = sf_mex_dup(c4_mxArrayInData);
  c4_identifier = c4_varName;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_cellpath), &c4_thisId,
                        c4_y_data, c4_y_sizes);
  sf_mex_destroy(&c4_cellpath);
  c4_outData_sizes[0] = c4_y_sizes[0];
  c4_outData_sizes[1] = 2;
  for (c4_i41 = 0; c4_i41 < 2; c4_i41++) {
    c4_loop_ub = c4_y_sizes[0] - 1;
    for (c4_i42 = 0; c4_i42 <= c4_loop_ub; c4_i42++) {
      c4_outData_data[c4_i42 + c4_outData_sizes[0] * c4_i41] = c4_y_data[c4_i42
        + c4_y_sizes[0] * c4_i41];
    }
  }

  sf_mex_destroy(&c4_mxArrayInData);
}

static void c4_g_emlrt_marshallIn
  (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance, const
   mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId, real_T c4_b_y[900])
{
  real_T c4_dv3[900];
  int32_T c4_i43;
  (void)chartInstance;
  sf_mex_import(c4_parentId, sf_mex_dup(c4_u), c4_dv3, 1, 0, 0U, 1, 0U, 2, 30,
                30);
  for (c4_i43 = 0; c4_i43 < 900; c4_i43++) {
    c4_b_y[c4_i43] = c4_dv3[c4_i43];
  }

  sf_mex_destroy(&c4_u);
}

static void c4_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData)
{
  const mxArray *c4_b_OccGrid;
  const char_T *c4_identifier;
  emlrtMsgIdentifier c4_thisId;
  real_T c4_b_y[900];
  int32_T c4_i44;
  int32_T c4_i45;
  int32_T c4_i46;
  SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance;
  chartInstance = (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *)
    chartInstanceVoid;
  c4_b_OccGrid = sf_mex_dup(c4_mxArrayInData);
  c4_identifier = c4_varName;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_g_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_b_OccGrid), &c4_thisId,
                        c4_b_y);
  sf_mex_destroy(&c4_b_OccGrid);
  c4_i44 = 0;
  for (c4_i45 = 0; c4_i45 < 30; c4_i45++) {
    for (c4_i46 = 0; c4_i46 < 30; c4_i46++) {
      (*(real_T (*)[900])c4_outData)[c4_i46 + c4_i44] = c4_b_y[c4_i46 + c4_i44];
    }

    c4_i44 += 30;
  }

  sf_mex_destroy(&c4_mxArrayInData);
}

const mxArray
  *sf_c4_trackVehicleNewVer7_SonarPath_get_eml_resolved_functions_info(void)
{
  const mxArray *c4_nameCaptureInfo = NULL;
  c4_nameCaptureInfo = NULL;
  sf_mex_assign(&c4_nameCaptureInfo, sf_mex_create("nameCaptureInfo", NULL, 0,
    0U, 1U, 0U, 2, 0, 1), false);
  return c4_nameCaptureInfo;
}

static void c4_pos2cell(SFc4_trackVehicleNewVer7_SonarPathInstanceStruct
  *chartInstance, real_T c4_b_x, real_T c4_b_y, real_T c4_b_cellsize[2], real_T
  c4_b_gridorig[2], real_T *c4_ix, real_T *c4_iy)
{
  uint32_T c4_debug_family_var_map[8];
  real_T c4_nargin = 4.0;
  real_T c4_nargout = 2.0;
  real_T c4_A;
  real_T c4_B;
  real_T c4_c_x;
  real_T c4_c_y;
  real_T c4_d_x;
  real_T c4_d_y;
  real_T c4_e_x;
  real_T c4_e_y;
  real_T c4_f_y;
  real_T c4_f_x;
  real_T c4_g_x;
  real_T c4_varargin_1;
  real_T c4_varargin_2;
  real_T c4_h_x;
  real_T c4_i_x;
  real_T c4_xk;
  real_T c4_j_x;
  real_T c4_b_A;
  real_T c4_b_B;
  real_T c4_k_x;
  real_T c4_g_y;
  real_T c4_l_x;
  real_T c4_h_y;
  real_T c4_m_x;
  real_T c4_i_y;
  real_T c4_j_y;
  real_T c4_n_x;
  real_T c4_o_x;
  real_T c4_b_varargin_1;
  real_T c4_b_varargin_2;
  real_T c4_p_x;
  real_T c4_q_x;
  real_T c4_b_xk;
  real_T c4_r_x;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 8U, 8U, c4_b_debug_family_names,
    c4_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_nargin, 0U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_nargout, 1U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_b_x, 2U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_b_y, 3U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c4_b_cellsize, 4U, c4_c_sf_marshallOut,
    c4_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c4_b_gridorig, 5U, c4_c_sf_marshallOut,
    c4_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c4_ix, 6U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c4_iy, 7U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  CV_EML_FCN(0, 3);
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 199U);
  c4_A = c4_b_x - c4_b_gridorig[0];
  c4_B = c4_b_cellsize[0];
  c4_c_x = c4_A;
  c4_c_y = c4_B;
  c4_d_x = c4_c_x;
  c4_d_y = c4_c_y;
  c4_e_x = c4_d_x;
  c4_e_y = c4_d_y;
  c4_f_y = c4_e_x / c4_e_y;
  c4_f_x = c4_f_y;
  c4_g_x = c4_f_x;
  c4_g_x = muDoubleScalarFloor(c4_g_x);
  c4_varargin_1 = c4_g_x + 1.0;
  c4_varargin_2 = c4_varargin_1;
  c4_h_x = c4_varargin_2;
  c4_i_x = c4_h_x;
  c4_eml_scalar_eg(chartInstance);
  c4_dimagree(chartInstance);
  c4_xk = c4_i_x;
  c4_j_x = c4_xk;
  c4_eml_scalar_eg(chartInstance);
  *c4_ix = muDoubleScalarMax(c4_j_x, 1.0);
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 200U);
  c4_b_A = c4_b_y - c4_b_gridorig[1];
  c4_b_B = c4_b_cellsize[1];
  c4_k_x = c4_b_A;
  c4_g_y = c4_b_B;
  c4_l_x = c4_k_x;
  c4_h_y = c4_g_y;
  c4_m_x = c4_l_x;
  c4_i_y = c4_h_y;
  c4_j_y = c4_m_x / c4_i_y;
  c4_n_x = c4_j_y;
  c4_o_x = c4_n_x;
  c4_o_x = muDoubleScalarFloor(c4_o_x);
  c4_b_varargin_1 = c4_o_x + 1.0;
  c4_b_varargin_2 = c4_b_varargin_1;
  c4_p_x = c4_b_varargin_2;
  c4_q_x = c4_p_x;
  c4_eml_scalar_eg(chartInstance);
  c4_dimagree(chartInstance);
  c4_b_xk = c4_q_x;
  c4_r_x = c4_b_xk;
  c4_eml_scalar_eg(chartInstance);
  *c4_iy = muDoubleScalarMax(c4_r_x, 1.0);
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, -200);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c4_eml_scalar_eg(SFc4_trackVehicleNewVer7_SonarPathInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c4_dimagree(SFc4_trackVehicleNewVer7_SonarPathInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c4_planPath(SFc4_trackVehicleNewVer7_SonarPathInstanceStruct
  *chartInstance, real_T c4_ix, real_T c4_iy, real_T c4_b_OccGrid[900], real_T
  c4_path_data[], int32_T c4_path_sizes[2], real_T *c4_b_pathlen)
{
  uint32_T c4_debug_family_var_map[29];
  real_T c4_came_from[1800];
  real_T c4_open[900];
  real_T c4_g_matrix[900];
  real_T c4_f_matrix[900];
  real_T c4_closed[900];
  real_T c4_loop_counter;
  real_T c4_goal_x;
  real_T c4_goal_y;
  real_T c4_i;
  real_T c4_j;
  real_T c4_xStart;
  real_T c4_yStart;
  real_T c4_xNode;
  real_T c4_yNode;
  real_T c4_counter;
  real_T c4_camefrom_index[900];
  real_T c4_cf_counter;
  real_T c4_index;
  real_T c4_cfi;
  real_T c4_row[2];
  real_T c4_k;
  real_T c4_temp_x;
  real_T c4_temp_y;
  real_T c4_new_g;
  real_T c4_b_path[1800];
  int32_T c4_i47;
  int32_T c4_i48;
  int32_T c4_i49;
  int32_T c4_i50;
  int32_T c4_i51;
  int32_T c4_i52;
  int32_T c4_i53;
  int32_T c4_i54;
  int32_T c4_i55;
  int32_T c4_b_i;
  int32_T c4_b_j;
  int32_T c4_i56;
  int32_T c4_i57;
  static real_T c4_dv4[12] = { 115.0, 116.0, 97.0, 114.0, 116.0, 105.0, 110.0,
    103.0, 32.0, 120.0, 58.0, 32.0 };

  real_T c4_dv5[12];
  int32_T c4_i58;
  static real_T c4_dv6[12] = { 115.0, 116.0, 97.0, 114.0, 116.0, 105.0, 110.0,
    103.0, 32.0, 121.0, 58.0, 32.0 };

  real_T c4_dv7[12];
  int32_T c4_trueCount;
  int32_T c4_c_i;
  int32_T c4_b_trueCount[1];
  static char_T c4_cv0[7] = { 'k', 'i', 'l', 'l', ' ', 'm', 'e' };

  int32_T c4_i59;
  real_T c4_varargin_1[900];
  real_T c4_mtmp;
  int32_T c4_itmp;
  int32_T c4_b_ix;
  int32_T c4_c_ix;
  real_T c4_a;
  real_T c4_b;
  boolean_T c4_p;
  int32_T c4_b_itmp;
  int32_T c4_iindx;
  int32_T c4_b_iindx;
  real_T c4_indx;
  real_T c4_b_index;
  real_T c4_ndx;
  real_T c4_b_ndx;
  int32_T c4_idx;
  const mxArray *c4_b_y = NULL;
  static char_T c4_u[36] = { 'C', 'o', 'd', 'e', 'r', ':', 'M', 'A', 'T', 'L',
    'A', 'B', ':', 'i', 'n', 'd', '2', 's', 'u', 'b', '_', 'I', 'n', 'd', 'e',
    'x', 'O', 'u', 't', 'O', 'f', 'R', 'a', 'n', 'g', 'e' };

  int32_T c4_v1;
  int32_T c4_b_a;
  int32_T c4_vk;
  int32_T c4_varargout_4;
  int32_T c4_varargout_3;
  int32_T c4_b_varargout_3;
  int32_T c4_b_varargout_4;
  real_T c4_b_xNode;
  real_T c4_b_yNode;
  static char_T c4_cv1[22] = { 'f', 'o', 'u', 'n', 'd', ' ', 'c', 'u', 'r', 'r',
    'e', 'n', 't', ' ', 'b', 'r', 'e', 'a', 'k', 'i', 'n', 'g' };

  int32_T c4_i60;
  int32_T c4_i61;
  int32_T c4_b_cfi;
  int32_T c4_i62;
  int32_T c4_b_counter;
  int32_T c4_c_cfi;
  int32_T c4_i63;
  int32_T c4_b_k;
  int32_T c4_c_j;
  int32_T c4_b_cf_counter;
  boolean_T c4_b3;
  boolean_T c4_b4;
  boolean_T c4_b5;
  int32_T c4_i64;
  int32_T c4_i65;
  int32_T c4_loop_ub;
  int32_T c4_i66;
  int32_T c4_tmp_sizes[2];
  int32_T c4_i67;
  int32_T c4_i68;
  int32_T c4_b_loop_ub;
  int32_T c4_i69;
  real_T c4_tmp_data[1800];
  int32_T c4_c_path;
  int32_T c4_d_path;
  int32_T c4_c_loop_ub;
  int32_T c4_i70;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  boolean_T guard3 = false;
  boolean_T guard4 = false;
  boolean_T guard5 = false;
  boolean_T guard6 = false;
  int32_T exitg1;
  boolean_T exitg2;
  boolean_T guard11 = false;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 29U, 30U, c4_d_debug_family_names,
    c4_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c4_came_from, 0U, c4_b_sf_marshallOut,
    c4_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c4_open, 1U, c4_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c4_g_matrix, 2U, c4_d_sf_marshallOut,
    c4_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c4_f_matrix, 3U, c4_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c4_closed, 4U, c4_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_loop_counter, 5U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_goal_x, 6U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_goal_y, 7U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_i, 8U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_j, 9U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_xStart, 10U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_yStart, 11U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_xNode, 12U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_yNode, 13U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c4_counter, 14U, c4_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c4_camefrom_index, 15U,
    c4_d_sf_marshallOut, c4_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_cf_counter, 16U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_index, 17U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_cfi, 18U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c4_row, 19U, c4_c_sf_marshallOut,
    c4_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_k, 20U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_temp_x, 21U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_temp_y, 22U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_new_g, 23U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c4_b_path, MAX_uint32_T,
    c4_b_sf_marshallOut, c4_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_ix, 25U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_iy, 26U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c4_b_OccGrid, 27U, c4_d_sf_marshallOut,
    c4_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_DYN_IMPORTABLE(c4_path_data, (const int32_T *)
    c4_path_sizes, NULL, 0, -1, (void *)c4_e_sf_marshallOut, (void *)
    c4_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c4_b_pathlen, 28U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  CV_EML_FCN(0, 1);
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 63);
  for (c4_i47 = 0; c4_i47 < 1800; c4_i47++) {
    c4_b_path[c4_i47] = 0.0;
  }

  _SFD_SYMBOL_SWITCH(24U, 24U);
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 64);
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 65);
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 66);
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 67);
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 69);
  for (c4_i48 = 0; c4_i48 < 1800; c4_i48++) {
    c4_came_from[c4_i48] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 70);
  for (c4_i49 = 0; c4_i49 < 900; c4_i49++) {
    c4_open[c4_i49] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 71);
  for (c4_i50 = 0; c4_i50 < 900; c4_i50++) {
    c4_open[c4_i50] = rtInf;
  }

  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 72);
  c4_open[(_SFD_EML_ARRAY_BOUNDS_CHECK("open", (int32_T)_SFD_INTEGER_CHECK("ix",
             c4_ix), 1, 30, 1, 0) + 30 * (_SFD_EML_ARRAY_BOUNDS_CHECK("open",
             (int32_T)_SFD_INTEGER_CHECK("iy", c4_iy), 1, 30, 2, 0) - 1)) - 1] =
    1.0;
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 74);
  for (c4_i51 = 0; c4_i51 < 900; c4_i51++) {
    c4_g_matrix[c4_i51] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 76);
  for (c4_i52 = 0; c4_i52 < 900; c4_i52++) {
    c4_f_matrix[c4_i52] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 77);
  for (c4_i53 = 0; c4_i53 < 900; c4_i53++) {
    c4_f_matrix[c4_i53] = rtInf;
  }

  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 79);
  for (c4_i54 = 0; c4_i54 < 900; c4_i54++) {
    c4_closed[c4_i54] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 80);
  for (c4_i55 = 0; c4_i55 < 900; c4_i55++) {
    c4_closed[c4_i55] = rtInf;
  }

  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 82);
  c4_loop_counter = 1.0;
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 83);
  c4_goal_x = 0.0;
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 84);
  c4_goal_y = 0.0;
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 86);
  c4_b_i = 0;
  while (c4_b_i < 30) {
    c4_i = 1.0 + (real_T)c4_b_i;
    CV_EML_FOR(0, 1, 3, 1);
    _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 87);
    c4_b_j = 0;
    while (c4_b_j < 30) {
      c4_j = 1.0 + (real_T)c4_b_j;
      CV_EML_FOR(0, 1, 4, 1);
      _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 88);
      guard6 = false;
      if (CV_EML_COND(0, 1, 0, CV_RELATIONAL_EVAL(4U, 0U, 3, c4_b_OccGrid
            [((int32_T)c4_i + 30 * ((int32_T)c4_j - 1)) - 1], 1.0, -1, 0U,
            c4_b_OccGrid[((int32_T)c4_i + 30 * ((int32_T)c4_j - 1)) - 1] == 1.0)))
      {
        guard6 = true;
      } else if (CV_EML_COND(0, 1, 1, CV_RELATIONAL_EVAL(4U, 0U, 4,
                   c4_b_OccGrid[((int32_T)c4_i + 30 * ((int32_T)c4_j - 1)) - 1],
        2.0, -1, 0U, c4_b_OccGrid[((int32_T)c4_i + 30 * ((int32_T)c4_j - 1)) - 1]
        == 2.0))) {
        guard6 = true;
      } else {
        CV_EML_MCDC(0, 1, 0, false);
        CV_EML_IF(0, 1, 3, false);
      }

      if (guard6 == true) {
        CV_EML_MCDC(0, 1, 0, true);
        CV_EML_IF(0, 1, 3, true);
        _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 89);
        c4_closed[((int32_T)c4_i + 30 * ((int32_T)c4_j - 1)) - 1] = 1.0;
      }

      _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 92);
      if (CV_EML_IF(0, 1, 4, CV_RELATIONAL_EVAL(4U, 0U, 5, c4_b_OccGrid
            [((int32_T)c4_i + 30 * ((int32_T)c4_j - 1)) - 1], 3.0, -1, 0U,
            c4_b_OccGrid[((int32_T)c4_i + 30 * ((int32_T)c4_j - 1)) - 1] == 3.0)))
      {
        _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 93);
        c4_goal_x = c4_i;
        _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 94);
        c4_goal_y = c4_j;
      }

      c4_b_j++;
      _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
    }

    CV_EML_FOR(0, 1, 4, 0);
    c4_b_i++;
    _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
  }

  CV_EML_FOR(0, 1, 3, 0);
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 99);
  c4_f_matrix[((int32_T)c4_ix + 30 * ((int32_T)c4_iy - 1)) - 1] = c4_distance
    (chartInstance, c4_ix, c4_iy, c4_goal_x, c4_goal_y);
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 101);
  c4_xStart = c4_ix;
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 102);
  c4_yStart = c4_iy;
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 103);
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 104);
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 105);
  c4_counter = 1.0;
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 106);
  for (c4_i56 = 0; c4_i56 < 900; c4_i56++) {
    c4_camefrom_index[c4_i56] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 108);
  c4_cf_counter = 1.0;
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 109);
  for (c4_i57 = 0; c4_i57 < 12; c4_i57++) {
    c4_dv5[c4_i57] = c4_dv4[c4_i57] + c4_ix;
  }

  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "disp", 0U, 1U, 14,
                    c4_b_emlrt_marshallOut(chartInstance, c4_dv5));
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 110);
  for (c4_i58 = 0; c4_i58 < 12; c4_i58++) {
    c4_dv7[c4_i58] = c4_dv6[c4_i58] + c4_iy;
  }

  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "disp", 0U, 1U, 14,
                    c4_b_emlrt_marshallOut(chartInstance, c4_dv7));
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 111);
  do {
    exitg1 = 0;
    c4_trueCount = 0;
    c4_c_i = 0;
    while (c4_c_i <= 899) {
      if (c4_open[c4_c_i] != rtInf) {
        c4_trueCount++;
      }

      c4_c_i++;
      _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
    }

    c4_b_trueCount[0] = c4_trueCount;
    if (CV_EML_WHILE(0, 1, 0, !(c4_b_trueCount[0] == 0))) {
      _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 112);
      c4_loop_counter++;
      _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 113);
      sf_mex_printf("%s =\\n", "loop_counter");
      sf_mex_call_debug(sfGlobalDebugInstanceStruct, "disp", 0U, 1U, 14,
                        c4_c_emlrt_marshallOut(chartInstance, c4_loop_counter));
      _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 114);
      if (CV_EML_IF(0, 1, 5, CV_RELATIONAL_EVAL(4U, 0U, 6, c4_loop_counter,
            5000.0, -1, 4U, c4_loop_counter > 5000.0))) {
        _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 115);
        sf_mex_call_debug(sfGlobalDebugInstanceStruct, "disp", 0U, 1U, 14,
                          c4_d_emlrt_marshallOut(chartInstance, c4_cv0));
        _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 116);
        exitg1 = 1;
      } else {
        _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 118);
        for (c4_i59 = 0; c4_i59 < 900; c4_i59++) {
          c4_varargin_1[c4_i59] = c4_f_matrix[c4_i59];
        }

        c4_mtmp = c4_varargin_1[0];
        c4_itmp = 1;
        c4_eml_switch_helper(chartInstance);
        c4_eml_switch_helper(chartInstance);
        for (c4_b_ix = 2; c4_b_ix < 901; c4_b_ix++) {
          c4_c_ix = c4_b_ix - 1;
          c4_a = c4_varargin_1[c4_c_ix];
          c4_b = c4_mtmp;
          c4_p = (c4_a < c4_b);
          if (c4_p) {
            c4_mtmp = c4_varargin_1[c4_c_ix];
            c4_itmp = c4_c_ix + 1;
          }
        }

        c4_b_itmp = c4_itmp;
        c4_iindx = c4_b_itmp;
        c4_b_iindx = c4_iindx;
        c4_indx = (real_T)c4_b_iindx;
        c4_b_index = c4_indx;
        c4_index = c4_b_index;
        _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 119);
        c4_ndx = c4_index;
        c4_b_ndx = c4_ndx;
        c4_idx = (int32_T)c4_b_ndx;
        if (c4_allinrange(chartInstance, c4_idx, 1.0, 900)) {
        } else {
          c4_b_y = NULL;
          sf_mex_assign(&c4_b_y, sf_mex_create("y", c4_u, 10, 0U, 1U, 0U, 2, 1,
            36), false);
          sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                            sf_mex_call_debug(sfGlobalDebugInstanceStruct,
            "message", 1U, 1U, 14, c4_b_y));
        }

        c4_v1 = c4_idx - 1;
        c4_b_a = c4_v1;
        c4_vk = c4_div_s32(chartInstance, c4_b_a, 30);
        c4_varargout_4 = c4_vk;
        c4_v1 = (c4_v1 - c4_vk * 30) + 1;
        c4_varargout_3 = c4_v1;
        c4_b_varargout_3 = c4_varargout_3;
        c4_b_varargout_4 = c4_varargout_4 + 1;
        c4_b_xNode = (real_T)c4_b_varargout_3;
        c4_b_yNode = (real_T)c4_b_varargout_4;
        c4_xNode = c4_b_xNode;
        c4_yNode = c4_b_yNode;
        _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 121);
        guard11 = false;
        if (CV_EML_COND(0, 1, 2, CV_RELATIONAL_EVAL(4U, 0U, 7, c4_xNode,
              c4_goal_x, -1, 0U, c4_xNode == c4_goal_x))) {
          if (CV_EML_COND(0, 1, 3, CV_RELATIONAL_EVAL(4U, 0U, 8, c4_yNode,
                c4_goal_y, -1, 0U, c4_yNode == c4_goal_y))) {
            CV_EML_MCDC(0, 1, 1, true);
            CV_EML_IF(0, 1, 6, true);
            _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 122);
            sf_mex_call_debug(sfGlobalDebugInstanceStruct, "disp", 0U, 1U, 14,
                              c4_e_emlrt_marshallOut(chartInstance, c4_cv1));
            _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 123);
            c4_i60 = 0;
            for (c4_i61 = 0; c4_i61 < 2; c4_i61++) {
              c4_b_path[c4_i60] = 0.0;
              c4_i60 += 900;
            }

            _SFD_SYMBOL_SWITCH(24U, 24U);
            _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 124);
            _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 125);
            c4_b_path[1] = c4_xNode;
            c4_b_path[901] = c4_yNode;
            _SFD_SYMBOL_SWITCH(24U, 24U);
            _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 126);
            c4_counter = 3.0;
            _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, MAX_int8_T);
            c4_cfi = c4_camefrom_index[(_SFD_EML_ARRAY_BOUNDS_CHECK(
              "camefrom_index", (int32_T)c4_xNode, 1, 30, 1, 0) + 30 * ((int32_T)
              c4_yNode - 1)) - 1];
            _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 128U);
            c4_b_cfi = _SFD_EML_ARRAY_BOUNDS_CHECK("came_from", (int32_T)c4_cfi,
              1, 900, 1, 0) - 1;
            for (c4_i62 = 0; c4_i62 < 2; c4_i62++) {
              c4_row[c4_i62] = c4_came_from[c4_b_cfi + 900 * c4_i62];
            }

            _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 131U);
            exitg2 = false;
            while ((exitg2 == false) && (c4_row[0] != c4_xStart)) {
              if (c4_row[1] != c4_yStart) {
                CV_EML_WHILE(0, 1, 1, true);
                _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 132U);
                c4_b_counter = _SFD_EML_ARRAY_BOUNDS_CHECK("path", (int32_T)
                  c4_counter, 1, 900, 1, 0) - 1;
                c4_b_path[c4_b_counter] = c4_row[0];
                c4_b_path[900 + c4_b_counter] = c4_row[1];
                _SFD_SYMBOL_SWITCH(24U, 24U);
                _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 133U);
                c4_counter++;
                _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 134U);
                c4_cfi = c4_camefrom_index[(_SFD_EML_ARRAY_BOUNDS_CHECK(
                  "camefrom_index", (int32_T)c4_row[0], 1, 30, 1, 0) + 30 *
                  (_SFD_EML_ARRAY_BOUNDS_CHECK("camefrom_index", (int32_T)
                  c4_row[1], 1, 30, 2, 0) - 1)) - 1];
                _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 135U);
                c4_c_cfi = _SFD_EML_ARRAY_BOUNDS_CHECK("came_from", (int32_T)
                  c4_cfi, 1, 900, 1, 0) - 1;
                for (c4_i63 = 0; c4_i63 < 2; c4_i63++) {
                  c4_row[c4_i63] = c4_came_from[c4_c_cfi + 900 * c4_i63];
                }

                _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 131U);
                _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
              } else {
                exitg2 = true;
              }
            }

            CV_EML_WHILE(0, 1, 1, false);
            _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 139U);
            exitg1 = 1;
          } else {
            guard11 = true;
          }
        } else {
          guard11 = true;
        }

        if (guard11 == true) {
          CV_EML_MCDC(0, 1, 1, false);
          CV_EML_IF(0, 1, 6, false);
          _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 143U);
          c4_open[(_SFD_EML_ARRAY_BOUNDS_CHECK("open", (int32_T)c4_xNode, 1, 30,
                    1, 0) + 30 * (_SFD_EML_ARRAY_BOUNDS_CHECK("open", (int32_T)
                     c4_yNode, 1, 30, 2, 0) - 1)) - 1] = rtInf;
          _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 144U);
          c4_closed[((int32_T)c4_xNode + 30 * ((int32_T)c4_yNode - 1)) - 1] =
            1.0;
          _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 145U);
          c4_f_matrix[((int32_T)c4_xNode + 30 * ((int32_T)c4_yNode - 1)) - 1] =
            rtInf;
          _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 146U);
          c4_b_k = 0;
          while (c4_b_k < 3) {
            c4_k = 1.0 + -(real_T)c4_b_k;
            CV_EML_FOR(0, 1, 5, 1);
            _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 147U);
            c4_c_j = 0;
            while (c4_c_j < 3) {
              c4_j = 1.0 + -(real_T)c4_c_j;
              CV_EML_FOR(0, 1, 6, 1);
              _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 148U);
              guard1 = false;
              guard2 = false;
              guard3 = false;
              guard4 = false;
              if (CV_EML_COND(0, 1, 4, CV_RELATIONAL_EVAL(4U, 0U, 9, c4_k, c4_j,
                    -1, 1U, c4_k != c4_j))) {
                guard4 = true;
              } else if (CV_EML_COND(0, 1, 5, CV_RELATIONAL_EVAL(4U, 0U, 10,
                           c4_k, 0.0, -1, 1U, c4_k != 0.0))) {
                guard4 = true;
              } else {
                CV_EML_MCDC(0, 1, 2, false);
                CV_EML_IF(0, 1, 7, false);
              }

              if (guard4 == true) {
                CV_EML_MCDC(0, 1, 2, true);
                CV_EML_IF(0, 1, 7, true);
                _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 149U);
                c4_temp_x = c4_xNode + c4_k;
                _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 150U);
                c4_temp_y = c4_yNode + c4_j;
                _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 151U);
                if (CV_EML_COND(0, 1, 6, CV_RELATIONAL_EVAL(4U, 0U, 11,
                      c4_temp_x, 0.0, -1, 4U, c4_temp_x > 0.0))) {
                  if (CV_EML_COND(0, 1, 7, CV_RELATIONAL_EVAL(4U, 0U, 12,
                        c4_temp_x, 30.0, -1, 3U, c4_temp_x <= 30.0))) {
                    if (CV_EML_COND(0, 1, 8, CV_RELATIONAL_EVAL(4U, 0U, 13,
                          c4_temp_y, 0.0, -1, 4U, c4_temp_y > 0.0))) {
                      if (CV_EML_COND(0, 1, 9, CV_RELATIONAL_EVAL(4U, 0U, 14,
                            c4_temp_y, 30.0, -1, 3U, c4_temp_y <= 30.0))) {
                        CV_EML_MCDC(0, 1, 3, true);
                        CV_EML_IF(0, 1, 8, true);
                        _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 152U);
                        if (CV_EML_IF(0, 1, 9, CV_RELATIONAL_EVAL(4U, 0U, 15,
                              c4_closed[((int32_T)c4_temp_x + 30 * ((int32_T)
                                c4_temp_y - 1)) - 1], rtInf, -1, 1U, c4_closed
                              [((int32_T)c4_temp_x + 30 * ((int32_T)c4_temp_y -
                                1)) - 1] != rtInf))) {
                          _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 153U);
                        } else {
                          _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 156U);
                          c4_new_g = c4_g_matrix[((int32_T)c4_xNode + 30 *
                            ((int32_T)c4_yNode - 1)) - 1] + c4_distance
                            (chartInstance, c4_xNode, c4_yNode, c4_temp_x,
                             c4_temp_y);
                          _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 158U);
                          guard5 = false;
                          if (CV_EML_COND(0, 1, 10, CV_RELATIONAL_EVAL(4U, 0U,
                                16, c4_open[((int32_T)c4_temp_x + 30 * ((int32_T)
                                  c4_temp_y - 1)) - 1], rtInf, -1, 0U, c4_open
                                [((int32_T)c4_temp_x + 30 * ((int32_T)c4_temp_y
                                  - 1)) - 1] == rtInf))) {
                            guard5 = true;
                          } else if (CV_EML_COND(0, 1, 11, CV_RELATIONAL_EVAL(4U,
                            0U, 17, c4_new_g, c4_g_matrix[((int32_T)c4_temp_x +
                                        30 * ((int32_T)c4_temp_y - 1)) - 1], -1,
                            2U, c4_new_g < c4_g_matrix[((int32_T)c4_temp_x + 30 *
                            ((int32_T)c4_temp_y - 1)) - 1]))) {
                            guard5 = true;
                          } else {
                            CV_EML_MCDC(0, 1, 4, false);
                            CV_EML_IF(0, 1, 10, false);
                          }

                          if (guard5 == true) {
                            CV_EML_MCDC(0, 1, 4, true);
                            CV_EML_IF(0, 1, 10, true);
                            _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 159U);
                            c4_b_cf_counter = _SFD_EML_ARRAY_BOUNDS_CHECK(
                              "came_from", (int32_T)c4_cf_counter, 1, 900, 1, 0)
                              - 1;
                            c4_came_from[c4_b_cf_counter] = c4_xNode;
                            c4_came_from[900 + c4_b_cf_counter] = c4_yNode;
                            _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 160U);
                            c4_camefrom_index[((int32_T)c4_temp_x + 30 *
                                               ((int32_T)c4_temp_y - 1)) - 1] =
                              c4_cf_counter;
                            _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 161U);
                            c4_cf_counter++;
                            _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 165U);
                            c4_g_matrix[((int32_T)c4_temp_x + 30 * ((int32_T)
                              c4_temp_y - 1)) - 1] = c4_new_g;
                            _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 166U);
                            c4_f_matrix[((int32_T)c4_temp_x + 30 * ((int32_T)
                              c4_temp_y - 1)) - 1] = c4_new_g + c4_distance
                              (chartInstance, c4_temp_x, c4_temp_y, c4_goal_x,
                               c4_goal_y);
                            _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 167U);
                            if (CV_EML_IF(0, 1, 11, CV_RELATIONAL_EVAL(4U, 0U,
                                  18, c4_open[((int32_T)c4_temp_x + 30 *
                                               ((int32_T)c4_temp_y - 1)) - 1],
                                  rtInf, -1, 0U, c4_open[((int32_T)c4_temp_x +
                                   30 * ((int32_T)c4_temp_y - 1)) - 1] == rtInf)))
                            {
                              _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 168U);
                              c4_open[((int32_T)c4_temp_x + 30 * ((int32_T)
                                        c4_temp_y - 1)) - 1] = 1.0;
                            }
                          }
                        }
                      } else {
                        guard3 = true;
                      }
                    } else {
                      guard3 = true;
                    }
                  } else {
                    guard2 = true;
                  }
                } else {
                  guard2 = true;
                }
              }

              if (guard3 == true) {
                guard1 = true;
              }

              if (guard2 == true) {
                guard1 = true;
              }

              if (guard1 == true) {
                CV_EML_MCDC(0, 1, 3, false);
                CV_EML_IF(0, 1, 8, false);
              }

              c4_c_j++;
              _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
            }

            CV_EML_FOR(0, 1, 6, 0);
            c4_b_k++;
            _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
          }

          CV_EML_FOR(0, 1, 5, 0);
          _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 111);
          _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
        }
      }
    } else {
      exitg1 = 1;
    }
  } while (exitg1 == 0);

  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 177U);
  *c4_b_pathlen = c4_counter - 1.0;
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 178U);
  c4_b3 = (1.0 > *c4_b_pathlen);
  c4_b4 = c4_b3;
  c4_b5 = c4_b4;
  if (c4_b5) {
    c4_i64 = 0;
  } else {
    c4_i64 = _SFD_EML_ARRAY_BOUNDS_CHECK("path", (int32_T)*c4_b_pathlen, 1, 900,
      0, 0);
  }

  c4_path_sizes[0] = c4_i64;
  c4_path_sizes[1] = 2;
  for (c4_i65 = 0; c4_i65 < 2; c4_i65++) {
    c4_loop_ub = c4_i64 - 1;
    for (c4_i66 = 0; c4_i66 <= c4_loop_ub; c4_i66++) {
      c4_path_data[c4_i66 + c4_path_sizes[0] * c4_i65] = c4_b_path[c4_i66 + 900 *
        c4_i65];
    }
  }

  _SFD_SYMBOL_SWITCH(24U, 28U);
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 179U);
  c4_tmp_sizes[0] = c4_path_sizes[0];
  c4_tmp_sizes[1] = 2;
  c4_i67 = c4_tmp_sizes[0];
  c4_i68 = c4_tmp_sizes[1];
  c4_b_loop_ub = c4_path_sizes[0] * c4_path_sizes[1] - 1;
  for (c4_i69 = 0; c4_i69 <= c4_b_loop_ub; c4_i69++) {
    c4_tmp_data[c4_i69] = c4_path_data[c4_i69];
  }

  c4_b_flipud(chartInstance, c4_tmp_data, c4_tmp_sizes);
  c4_path_sizes[0] = c4_tmp_sizes[0];
  c4_path_sizes[1] = 2;
  c4_c_path = c4_path_sizes[0];
  c4_d_path = c4_path_sizes[1];
  c4_c_loop_ub = c4_tmp_sizes[0] * c4_tmp_sizes[1] - 1;
  for (c4_i70 = 0; c4_i70 <= c4_c_loop_ub; c4_i70++) {
    c4_path_data[c4_i70] = c4_tmp_data[c4_i70];
  }

  _SFD_SYMBOL_SWITCH(24U, 28U);
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, -179);
  _SFD_SYMBOL_SCOPE_POP();
}

static real_T c4_distance(SFc4_trackVehicleNewVer7_SonarPathInstanceStruct
  *chartInstance, real_T c4_x1, real_T c4_y1, real_T c4_x2, real_T c4_y2)
{
  real_T c4_dist;
  uint32_T c4_debug_family_var_map[7];
  real_T c4_nargin = 4.0;
  real_T c4_nargout = 1.0;
  real_T c4_b_x;
  real_T c4_c_x;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 7U, 7U, c4_c_debug_family_names,
    c4_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_nargin, 0U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_nargout, 1U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_x1, 2U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_y1, 3U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_x2, 4U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_y2, 5U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_dist, 6U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  CV_EML_FCN(0, 2);
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 194U);
  c4_b_x = c4_mpower(chartInstance, c4_x1 - c4_x2) + c4_mpower(chartInstance,
    c4_y1 - c4_y2);
  c4_dist = c4_b_x;
  if (c4_dist < 0.0) {
    c4_eml_error(chartInstance);
  }

  c4_c_x = c4_dist;
  c4_dist = c4_c_x;
  c4_dist = muDoubleScalarSqrt(c4_dist);
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, -194);
  _SFD_SYMBOL_SCOPE_POP();
  return c4_dist;
}

static real_T c4_mpower(SFc4_trackVehicleNewVer7_SonarPathInstanceStruct
  *chartInstance, real_T c4_a)
{
  real_T c4_b_a;
  real_T c4_c_a;
  real_T c4_ak;
  real_T c4_d_a;
  c4_b_a = c4_a;
  c4_c_a = c4_b_a;
  c4_eml_scalar_eg(chartInstance);
  c4_dimagree(chartInstance);
  c4_ak = c4_c_a;
  c4_d_a = c4_ak;
  c4_eml_scalar_eg(chartInstance);
  return c4_d_a * c4_d_a;
}

static real_T c4_sqrt(SFc4_trackVehicleNewVer7_SonarPathInstanceStruct
                      *chartInstance, real_T c4_b_x)
{
  real_T c4_c_x;
  c4_c_x = c4_b_x;
  c4_b_sqrt(chartInstance, &c4_c_x);
  return c4_c_x;
}

static void c4_eml_error(SFc4_trackVehicleNewVer7_SonarPathInstanceStruct
  *chartInstance)
{
  const mxArray *c4_b_y = NULL;
  static char_T c4_u[30] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'E', 'l', 'F', 'u', 'n', 'D', 'o', 'm', 'a', 'i', 'n',
    'E', 'r', 'r', 'o', 'r' };

  const mxArray *c4_c_y = NULL;
  static char_T c4_b_u[4] = { 's', 'q', 'r', 't' };

  (void)chartInstance;
  c4_b_y = NULL;
  sf_mex_assign(&c4_b_y, sf_mex_create("y", c4_u, 10, 0U, 1U, 0U, 2, 1, 30),
                false);
  c4_c_y = NULL;
  sf_mex_assign(&c4_c_y, sf_mex_create("y", c4_b_u, 10, 0U, 1U, 0U, 2, 1, 4),
                false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    2U, 14, c4_b_y, 14, c4_c_y));
}

static void c4_eml_switch_helper
  (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static boolean_T c4_allinrange(SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *
  chartInstance, int32_T c4_b_x, real_T c4_lo, int32_T c4_hi)
{
  (void)chartInstance;
  (void)c4_b_x;
  (void)c4_lo;
  (void)c4_hi;
  return true;
}

static void c4_flipud(SFc4_trackVehicleNewVer7_SonarPathInstanceStruct
                      *chartInstance, real_T c4_x_data[], int32_T c4_x_sizes[2],
                      real_T c4_b_x_data[], int32_T c4_b_x_sizes[2])
{
  int32_T c4_b_x;
  int32_T c4_c_x;
  int32_T c4_loop_ub;
  int32_T c4_i71;
  c4_b_x_sizes[0] = c4_x_sizes[0];
  c4_b_x_sizes[1] = 2;
  c4_b_x = c4_b_x_sizes[0];
  c4_c_x = c4_b_x_sizes[1];
  c4_loop_ub = c4_x_sizes[0] * c4_x_sizes[1] - 1;
  for (c4_i71 = 0; c4_i71 <= c4_loop_ub; c4_i71++) {
    c4_b_x_data[c4_i71] = c4_x_data[c4_i71];
  }

  c4_b_flipud(chartInstance, c4_b_x_data, c4_b_x_sizes);
}

static void c4_cell2pos(SFc4_trackVehicleNewVer7_SonarPathInstanceStruct
  *chartInstance, real_T c4_ix, real_T c4_iy, real_T c4_b_cellsize[2], real_T
  c4_b_gridorig[2], real_T *c4_b_x, real_T *c4_b_y)
{
  uint32_T c4_debug_family_var_map[8];
  real_T c4_nargin = 4.0;
  real_T c4_nargout = 2.0;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 8U, 8U, c4_e_debug_family_names,
    c4_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_nargin, 0U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_nargout, 1U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_ix, 2U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c4_iy, 3U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c4_b_cellsize, 4U, c4_c_sf_marshallOut,
    c4_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c4_b_gridorig, 5U, c4_c_sf_marshallOut,
    c4_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c4_b_x, 6U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c4_b_y, 7U, c4_sf_marshallOut,
    c4_sf_marshallIn);
  CV_EML_FCN(0, 4);
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 205U);
  *c4_b_x = (c4_ix - 0.5) * c4_b_cellsize[0] + c4_b_gridorig[0];
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, 206U);
  *c4_b_y = (c4_iy - 0.5) * c4_b_cellsize[1] + c4_b_gridorig[1];
  _SFD_EML_CALL(0U, chartInstance->c4_sfEvent, -206);
  _SFD_SYMBOL_SCOPE_POP();
}

static const mxArray *c4_emlrt_marshallOut
  (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance, const real_T
   c4_u_data[], const int32_T c4_u_sizes[2])
{
  const mxArray *c4_b_y = NULL;
  (void)chartInstance;
  c4_b_y = NULL;
  sf_mex_assign(&c4_b_y, sf_mex_create("y", c4_u_data, 0, 0U, 1U, 0U, 2,
    c4_u_sizes[0], c4_u_sizes[1]), false);
  return c4_b_y;
}

static const mxArray *c4_b_emlrt_marshallOut
  (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance, const real_T
   c4_u[12])
{
  const mxArray *c4_b_y = NULL;
  (void)chartInstance;
  c4_b_y = NULL;
  sf_mex_assign(&c4_b_y, sf_mex_create("y", c4_u, 0, 0U, 1U, 0U, 2, 1, 12),
                false);
  return c4_b_y;
}

static const mxArray *c4_c_emlrt_marshallOut
  (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance, const real_T
   c4_u)
{
  const mxArray *c4_b_y = NULL;
  (void)chartInstance;
  c4_b_y = NULL;
  sf_mex_assign(&c4_b_y, sf_mex_create("y", &c4_u, 0, 0U, 0U, 0U, 0), false);
  return c4_b_y;
}

static const mxArray *c4_d_emlrt_marshallOut
  (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance, const char_T
   c4_u[7])
{
  const mxArray *c4_b_y = NULL;
  (void)chartInstance;
  c4_b_y = NULL;
  sf_mex_assign(&c4_b_y, sf_mex_create("y", c4_u, 10, 0U, 1U, 0U, 2, 1, 7),
                false);
  return c4_b_y;
}

static const mxArray *c4_e_emlrt_marshallOut
  (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance, const char_T
   c4_u[22])
{
  const mxArray *c4_b_y = NULL;
  (void)chartInstance;
  c4_b_y = NULL;
  sf_mex_assign(&c4_b_y, sf_mex_create("y", c4_u, 10, 0U, 1U, 0U, 2, 1, 22),
                false);
  return c4_b_y;
}

static const mxArray *c4_f_sf_marshallOut(void *chartInstanceVoid, void
  *c4_inData)
{
  const mxArray *c4_mxArrayOutData = NULL;
  int32_T c4_u;
  const mxArray *c4_b_y = NULL;
  SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance;
  chartInstance = (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *)
    chartInstanceVoid;
  c4_mxArrayOutData = NULL;
  c4_u = *(int32_T *)c4_inData;
  c4_b_y = NULL;
  sf_mex_assign(&c4_b_y, sf_mex_create("y", &c4_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c4_mxArrayOutData, c4_b_y, false);
  return c4_mxArrayOutData;
}

static int32_T c4_h_emlrt_marshallIn
  (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance, const
   mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId)
{
  int32_T c4_b_y;
  int32_T c4_i72;
  (void)chartInstance;
  sf_mex_import(c4_parentId, sf_mex_dup(c4_u), &c4_i72, 1, 6, 0U, 0, 0U, 0);
  c4_b_y = c4_i72;
  sf_mex_destroy(&c4_u);
  return c4_b_y;
}

static void c4_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c4_mxArrayInData, const char_T *c4_varName, void *c4_outData)
{
  const mxArray *c4_b_sfEvent;
  const char_T *c4_identifier;
  emlrtMsgIdentifier c4_thisId;
  int32_T c4_b_y;
  SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance;
  chartInstance = (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *)
    chartInstanceVoid;
  c4_b_sfEvent = sf_mex_dup(c4_mxArrayInData);
  c4_identifier = c4_varName;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_b_y = c4_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_b_sfEvent),
    &c4_thisId);
  sf_mex_destroy(&c4_b_sfEvent);
  *(int32_T *)c4_outData = c4_b_y;
  sf_mex_destroy(&c4_mxArrayInData);
}

static uint8_T c4_i_emlrt_marshallIn
  (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance, const
   mxArray *c4_b_is_active_c4_trackVehicleNewVer7_SonarPath, const char_T
   *c4_identifier)
{
  uint8_T c4_b_y;
  emlrtMsgIdentifier c4_thisId;
  c4_thisId.fIdentifier = c4_identifier;
  c4_thisId.fParent = NULL;
  c4_b_y = c4_j_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c4_b_is_active_c4_trackVehicleNewVer7_SonarPath), &c4_thisId);
  sf_mex_destroy(&c4_b_is_active_c4_trackVehicleNewVer7_SonarPath);
  return c4_b_y;
}

static uint8_T c4_j_emlrt_marshallIn
  (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance, const
   mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId)
{
  uint8_T c4_b_y;
  uint8_T c4_u0;
  (void)chartInstance;
  sf_mex_import(c4_parentId, sf_mex_dup(c4_u), &c4_u0, 1, 3, 0U, 0, 0U, 0);
  c4_b_y = c4_u0;
  sf_mex_destroy(&c4_u);
  return c4_b_y;
}

static void c4_b_sqrt(SFc4_trackVehicleNewVer7_SonarPathInstanceStruct
                      *chartInstance, real_T *c4_b_x)
{
  if (*c4_b_x < 0.0) {
    c4_eml_error(chartInstance);
  }

  *c4_b_x = muDoubleScalarSqrt(*c4_b_x);
}

static void c4_b_flipud(SFc4_trackVehicleNewVer7_SonarPathInstanceStruct
  *chartInstance, real_T c4_x_data[], int32_T c4_x_sizes[2])
{
  int32_T c4_m;
  int32_T c4_md2;
  int32_T c4_j;
  int32_T c4_b_j;
  int32_T c4_b_md2;
  int32_T c4_b;
  int32_T c4_b_b;
  int32_T c4_i;
  int32_T c4_b_i;
  int32_T c4_b_x[2];
  real_T c4_xtmp;
  int32_T c4_c_x[2];
  int32_T c4_d_x[2];
  int32_T c4_tmp_sizes[2];
  int32_T c4_e_x[2];
  int32_T c4_b_tmp_sizes[2];
  c4_m = c4_x_sizes[0] + 1;
  c4_md2 = (c4_m - 1) >> 1;
  c4_eml_switch_helper(chartInstance);
  for (c4_j = 1; c4_j < 3; c4_j++) {
    c4_b_j = c4_j - 1;
    c4_b_md2 = c4_md2;
    c4_b = c4_b_md2;
    c4_b_b = c4_b;
    if (1 > c4_b_b) {
    } else {
      c4_eml_switch_helper(chartInstance);
      c4_eml_switch_helper(chartInstance);
    }

    for (c4_i = 1; c4_i <= c4_b_md2; c4_i++) {
      c4_b_i = c4_i;
      c4_b_x[0] = c4_x_sizes[0];
      c4_b_x[1] = 2;
      c4_xtmp = c4_x_data[(_SFD_EML_ARRAY_BOUNDS_CHECK("", c4_b_i, 1,
        c4_x_sizes[0], 1, 0) + c4_b_x[0] * c4_b_j) - 1];
      c4_c_x[0] = c4_x_sizes[0];
      c4_c_x[1] = 2;
      c4_d_x[0] = c4_x_sizes[0];
      c4_d_x[1] = 2;
      c4_tmp_sizes[0] = c4_c_x[0];
      c4_tmp_sizes[1] = c4_c_x[1];
      c4_x_data[(_SFD_EML_ARRAY_BOUNDS_CHECK("", c4_b_i, 1, c4_x_sizes[0], 1, 0)
                 + c4_tmp_sizes[0] * c4_b_j) - 1] = c4_x_data
        [(_SFD_EML_ARRAY_BOUNDS_CHECK("", c4_m - c4_b_i, 1, c4_x_sizes[0], 1, 0)
          + c4_d_x[0] * c4_b_j) - 1];
      c4_e_x[0] = c4_x_sizes[0];
      c4_e_x[1] = 2;
      c4_b_tmp_sizes[0] = c4_e_x[0];
      c4_b_tmp_sizes[1] = c4_e_x[1];
      c4_x_data[(_SFD_EML_ARRAY_BOUNDS_CHECK("", c4_m - c4_b_i, 1, c4_x_sizes[0],
                  1, 0) + c4_b_tmp_sizes[0] * c4_b_j) - 1] = c4_xtmp;
    }
  }
}

static int32_T c4_div_s32(SFc4_trackVehicleNewVer7_SonarPathInstanceStruct
  *chartInstance, int32_T c4_numerator, int32_T c4_denominator)
{
  int32_T c4_quotient;
  uint32_T c4_absNumerator;
  uint32_T c4_absDenominator;
  boolean_T c4_quotientNeedsNegation;
  uint32_T c4_tempAbsQuotient;
  if (c4_denominator == 0) {
    if (c4_numerator >= 0) {
      c4_quotient = MAX_int32_T;
    } else {
      c4_quotient = MIN_int32_T;
    }

    _SFD_OVERFLOW_DETECTION(SFDB_DIVIDE_BY_ZERO, 1U, 0U,
      chartInstance->c4_sfEvent, false);
  } else {
    if (c4_numerator >= 0) {
      c4_absNumerator = (uint32_T)c4_numerator;
    } else {
      c4_absNumerator = (uint32_T)-c4_numerator;
    }

    if (c4_denominator >= 0) {
      c4_absDenominator = (uint32_T)c4_denominator;
    } else {
      c4_absDenominator = (uint32_T)-c4_denominator;
    }

    c4_quotientNeedsNegation = (c4_numerator < 0 != c4_denominator < 0);
    c4_tempAbsQuotient = c4_absNumerator / c4_absDenominator;
    if (c4_quotientNeedsNegation) {
      c4_quotient = -(int32_T)c4_tempAbsQuotient;
    } else {
      c4_quotient = (int32_T)c4_tempAbsQuotient;
    }
  }

  return c4_quotient;
}

static void init_dsm_address_info
  (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void init_simulink_io_address
  (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance)
{
  chartInstance->c4_x = (real_T *)ssGetInputPortSignal_wrapper(chartInstance->S,
    0);
  chartInstance->c4_y = (real_T *)ssGetInputPortSignal_wrapper(chartInstance->S,
    1);
  chartInstance->c4_heading = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 2);
  chartInstance->c4_goals = (real_T (*)[2])ssGetInputPortSignal_wrapper
    (chartInstance->S, 3);
  chartInstance->c4_OccGrid = (real_T (*)[900])ssGetInputPortSignal_wrapper
    (chartInstance->S, 4);
  chartInstance->c4_path = (real_T (*)[1800])ssGetOutputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c4_cellsize = (real_T (*)[2])ssGetInputPortSignal_wrapper
    (chartInstance->S, 5);
  chartInstance->c4_gridorig = (real_T (*)[2])ssGetInputPortSignal_wrapper
    (chartInstance->S, 6);
  chartInstance->c4_pathlen = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 2);
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

void sf_c4_trackVehicleNewVer7_SonarPath_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(4196623932U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(4034653034U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(467567132U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1238168487U);
}

mxArray* sf_c4_trackVehicleNewVer7_SonarPath_get_post_codegen_info(void);
mxArray *sf_c4_trackVehicleNewVer7_SonarPath_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals", "postCodegenInfo" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1, 1, sizeof
    (autoinheritanceFields)/sizeof(autoinheritanceFields[0]),
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("Wsl68KwkrPrzEDgB46uciC");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,7,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
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
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
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
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
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
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,3,"type",mxType);
    }

    mxSetField(mxData,3,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(30);
      pr[1] = (double)(30);
      mxSetField(mxData,4,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
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
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,5,"type",mxType);
    }

    mxSetField(mxData,5,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(2);
      mxSetField(mxData,6,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,6,"type",mxType);
    }

    mxSetField(mxData,6,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,2,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(900);
      pr[1] = (double)(2);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
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
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  {
    mxArray* mxPostCodegenInfo =
      sf_c4_trackVehicleNewVer7_SonarPath_get_post_codegen_info();
    mxSetField(mxAutoinheritanceInfo,0,"postCodegenInfo",mxPostCodegenInfo);
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c4_trackVehicleNewVer7_SonarPath_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c4_trackVehicleNewVer7_SonarPath_jit_fallback_info(void)
{
  const char *infoFields[] = { "fallbackType", "fallbackReason",
    "hiddenFallbackType", "hiddenFallbackReason", "incompatibleSymbol" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 5, infoFields);
  mxArray *fallbackType = mxCreateString("pre");
  mxArray *fallbackReason = mxCreateString("hasBreakpoints");
  mxArray *hiddenFallbackType = mxCreateString("none");
  mxArray *hiddenFallbackReason = mxCreateString("");
  mxArray *incompatibleSymbol = mxCreateString("");
  mxSetField(mxInfo, 0, infoFields[0], fallbackType);
  mxSetField(mxInfo, 0, infoFields[1], fallbackReason);
  mxSetField(mxInfo, 0, infoFields[2], hiddenFallbackType);
  mxSetField(mxInfo, 0, infoFields[3], hiddenFallbackReason);
  mxSetField(mxInfo, 0, infoFields[4], incompatibleSymbol);
  return mxInfo;
}

mxArray *sf_c4_trackVehicleNewVer7_SonarPath_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

mxArray* sf_c4_trackVehicleNewVer7_SonarPath_get_post_codegen_info(void)
{
  const char* fieldNames[] = { "exportedFunctionsUsedByThisChart",
    "exportedFunctionsChecksum" };

  mwSize dims[2] = { 1, 1 };

  mxArray* mxPostCodegenInfo = mxCreateStructArray(2, dims, sizeof(fieldNames)/
    sizeof(fieldNames[0]), fieldNames);

  {
    mxArray* mxExportedFunctionsChecksum = mxCreateString("");
    mwSize exp_dims[2] = { 0, 1 };

    mxArray* mxExportedFunctionsUsedByThisChart = mxCreateCellArray(2, exp_dims);
    mxSetField(mxPostCodegenInfo, 0, "exportedFunctionsUsedByThisChart",
               mxExportedFunctionsUsedByThisChart);
    mxSetField(mxPostCodegenInfo, 0, "exportedFunctionsChecksum",
               mxExportedFunctionsChecksum);
  }

  return mxPostCodegenInfo;
}

static const mxArray *sf_get_sim_state_info_c4_trackVehicleNewVer7_SonarPath
  (void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x3'type','srcId','name','auxInfo'{{M[1],M[9],T\"path\",},{M[1],M[12],T\"pathlen\",},{M[8],M[0],T\"is_active_c4_trackVehicleNewVer7_SonarPath\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 3, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c4_trackVehicleNewVer7_SonarPath_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    chartInstance = (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *)
      chartInfo->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _trackVehicleNewVer7_SonarPathMachineNumber_,
           4,
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

        /* Each instance must initialize its own list of scripts */
        init_script_number_translation
          (_trackVehicleNewVer7_SonarPathMachineNumber_,
           chartInstance->chartNumber,chartInstance->instanceNumber);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          sf_debug_set_chart_disable_implicit_casting
            (sfGlobalDebugInstanceStruct,
             _trackVehicleNewVer7_SonarPathMachineNumber_,
             chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(sfGlobalDebugInstanceStruct,
            _trackVehicleNewVer7_SonarPathMachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,1,1,0,"x");
          _SFD_SET_DATA_PROPS(1,1,1,0,"y");
          _SFD_SET_DATA_PROPS(2,1,1,0,"heading");
          _SFD_SET_DATA_PROPS(3,1,1,0,"goals");
          _SFD_SET_DATA_PROPS(4,1,1,0,"OccGrid");
          _SFD_SET_DATA_PROPS(5,1,1,0,"cellsize");
          _SFD_SET_DATA_PROPS(6,1,1,0,"gridorig");
          _SFD_SET_DATA_PROPS(7,2,0,1,"path");
          _SFD_SET_DATA_PROPS(8,2,0,1,"pathlen");
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
        _SFD_CV_INIT_EML(0,1,5,0,12,0,0,0,7,2,12,5);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",273,-1,1934);
        _SFD_CV_INIT_EML_FCN(0,1,"planPath",2159,-1,5860);
        _SFD_CV_INIT_EML_FCN(0,2,"distance",5862,-1,5937);
        _SFD_CV_INIT_EML_FCN(0,3,"pos2cell",5990,-1,6154);
        _SFD_CV_INIT_EML_FCN(0,4,"cell2pos",6206,-1,6348);
        _SFD_CV_INIT_EML_IF(0,1,0,556,579,-1,1890);
        _SFD_CV_INIT_EML_IF(0,1,1,963,983,-1,1885);
        _SFD_CV_INIT_EML_IF(0,1,2,1650,1672,-1,1760);
        _SFD_CV_INIT_EML_IF(0,1,3,2717,2760,-1,2872);
        _SFD_CV_INIT_EML_IF(0,1,4,2881,2903,-1,2963);
        _SFD_CV_INIT_EML_IF(0,1,5,3359,3383,3533,4180);
        _SFD_CV_INIT_EML_IF(0,1,6,3533,3572,-1,-2);
        _SFD_CV_INIT_EML_IF(0,1,7,4337,4358,-1,5531);
        _SFD_CV_INIT_EML_IF(0,1,8,4447,4518,-1,5515);
        _SFD_CV_INIT_EML_IF(0,1,9,4539,4573,4784,5495);
        _SFD_CV_INIT_EML_IF(0,1,10,4784,4852,-1,5495);
        _SFD_CV_INIT_EML_IF(0,1,11,5357,5389,-1,5471);
        _SFD_CV_INIT_EML_FOR(0,1,0,584,613,737);
        _SFD_CV_INIT_EML_FOR(0,1,1,1047,1070,1246);
        _SFD_CV_INIT_EML_FOR(0,1,2,1453,1482,1773);
        _SFD_CV_INIT_EML_FOR(0,1,3,2677,2691,2975);
        _SFD_CV_INIT_EML_FOR(0,1,4,2695,2709,2971);
        _SFD_CV_INIT_EML_FOR(0,1,5,4289,4303,5551);
        _SFD_CV_INIT_EML_FOR(0,1,6,4311,4325,5543);
        _SFD_CV_INIT_EML_WHILE(0,1,0,3265,3300,5555);
        _SFD_CV_INIT_EML_WHILE(0,1,1,3898,3941,4123);

        {
          static int condStart[] = { 2721, 2742 };

          static int condEnd[] = { 2738, 2759 };

          static int pfixExpr[] = { 0, 1, -2 };

          _SFD_CV_INIT_EML_MCDC(0,1,0,2721,2759,2,0,&(condStart[0]),&(condEnd[0]),
                                3,&(pfixExpr[0]));
        }

        {
          static int condStart[] = { 3537, 3556 };

          static int condEnd[] = { 3552, 3571 };

          static int pfixExpr[] = { 0, 1, -3 };

          _SFD_CV_INIT_EML_MCDC(0,1,1,3537,3571,2,2,&(condStart[0]),&(condEnd[0]),
                                3,&(pfixExpr[0]));
        }

        {
          static int condStart[] = { 4341, 4351 };

          static int condEnd[] = { 4347, 4357 };

          static int pfixExpr[] = { 0, 1, -2 };

          _SFD_CV_INIT_EML_MCDC(0,1,2,4341,4357,2,4,&(condStart[0]),&(condEnd[0]),
                                3,&(pfixExpr[0]));
        }

        {
          static int condStart[] = { 4452, 4466, 4487, 4501 };

          static int condEnd[] = { 4462, 4481, 4497, 4516 };

          static int pfixExpr[] = { 0, 1, -3, 2, 3, -3, -3 };

          _SFD_CV_INIT_EML_MCDC(0,1,3,4451,4517,4,6,&(condStart[0]),&(condEnd[0]),
                                7,&(pfixExpr[0]));
        }

        {
          static int condStart[] = { 4788, 4819 };

          static int condEnd[] = { 4815, 4851 };

          static int pfixExpr[] = { 0, 1, -2 };

          _SFD_CV_INIT_EML_MCDC(0,1,4,4788,4851,2,10,&(condStart[0]),&(condEnd[0]),
                                3,&(pfixExpr[0]));
        }

        _SFD_CV_INIT_EML_RELATIONAL(0,1,0,560,578,-1,4);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,1,967,982,-1,4);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,2,1654,1671,-1,2);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,3,2721,2738,-1,0);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,4,2742,2759,-1,0);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,5,2885,2902,-1,0);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,6,3363,3382,-1,4);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,7,3537,3552,-1,0);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,8,3556,3571,-1,0);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,9,4341,4347,-1,1);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,10,4351,4357,-1,1);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,11,4452,4462,-1,4);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,12,4466,4481,-1,3);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,13,4487,4497,-1,4);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,14,4501,4516,-1,3);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,15,4543,4572,-1,1);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,16,4788,4815,-1,0);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,17,4819,4851,-1,2);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,18,5361,5388,-1,0);
        _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c4_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c4_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c4_sf_marshallOut,(MexInFcnForType)NULL);

        {
          unsigned int dimVector[2];
          dimVector[0]= 1;
          dimVector[1]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c4_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 30;
          dimVector[1]= 30;
          _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c4_d_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 1;
          dimVector[1]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c4_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 1;
          dimVector[1]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c4_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 900;
          dimVector[1]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(7,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c4_b_sf_marshallOut,(MexInFcnForType)
            c4_b_sf_marshallIn);
        }

        _SFD_SET_DATA_COMPILED_PROPS(8,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c4_sf_marshallOut,(MexInFcnForType)c4_sf_marshallIn);
      }
    } else {
      sf_debug_reset_current_state_configuration(sfGlobalDebugInstanceStruct,
        _trackVehicleNewVer7_SonarPathMachineNumber_,chartInstance->chartNumber,
        chartInstance->instanceNumber);
    }
  }
}

static void chart_debug_initialize_data_addresses(SimStruct *S)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    chartInstance = (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *)
      chartInfo->chartInstance;
    if (ssIsFirstInitCond(S)) {
      /* do this only if simulation is starting and after we know the addresses of all data */
      {
        _SFD_SET_DATA_VALUE_PTR(0U, chartInstance->c4_x);
        _SFD_SET_DATA_VALUE_PTR(1U, chartInstance->c4_y);
        _SFD_SET_DATA_VALUE_PTR(2U, chartInstance->c4_heading);
        _SFD_SET_DATA_VALUE_PTR(3U, *chartInstance->c4_goals);
        _SFD_SET_DATA_VALUE_PTR(4U, *chartInstance->c4_OccGrid);
        _SFD_SET_DATA_VALUE_PTR(7U, *chartInstance->c4_path);
        _SFD_SET_DATA_VALUE_PTR(5U, *chartInstance->c4_cellsize);
        _SFD_SET_DATA_VALUE_PTR(6U, *chartInstance->c4_gridorig);
        _SFD_SET_DATA_VALUE_PTR(8U, chartInstance->c4_pathlen);
      }
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "sWbj9YbJNINRj6Yr8xAhqnD";
}

static void sf_opaque_initialize_c4_trackVehicleNewVer7_SonarPath(void
  *chartInstanceVar)
{
  chart_debug_initialization(((SFc4_trackVehicleNewVer7_SonarPathInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c4_trackVehicleNewVer7_SonarPath
    ((SFc4_trackVehicleNewVer7_SonarPathInstanceStruct*) chartInstanceVar);
  initialize_c4_trackVehicleNewVer7_SonarPath
    ((SFc4_trackVehicleNewVer7_SonarPathInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c4_trackVehicleNewVer7_SonarPath(void
  *chartInstanceVar)
{
  enable_c4_trackVehicleNewVer7_SonarPath
    ((SFc4_trackVehicleNewVer7_SonarPathInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c4_trackVehicleNewVer7_SonarPath(void
  *chartInstanceVar)
{
  disable_c4_trackVehicleNewVer7_SonarPath
    ((SFc4_trackVehicleNewVer7_SonarPathInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c4_trackVehicleNewVer7_SonarPath(void
  *chartInstanceVar)
{
  sf_gateway_c4_trackVehicleNewVer7_SonarPath
    ((SFc4_trackVehicleNewVer7_SonarPathInstanceStruct*) chartInstanceVar);
}

static const mxArray* sf_opaque_get_sim_state_c4_trackVehicleNewVer7_SonarPath
  (SimStruct* S)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  return get_sim_state_c4_trackVehicleNewVer7_SonarPath
    ((SFc4_trackVehicleNewVer7_SonarPathInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
}

static void sf_opaque_set_sim_state_c4_trackVehicleNewVer7_SonarPath(SimStruct*
  S, const mxArray *st)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  set_sim_state_c4_trackVehicleNewVer7_SonarPath
    ((SFc4_trackVehicleNewVer7_SonarPathInstanceStruct*)chartInfo->chartInstance,
     st);
}

static void sf_opaque_terminate_c4_trackVehicleNewVer7_SonarPath(void
  *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc4_trackVehicleNewVer7_SonarPathInstanceStruct*)
                    chartInstanceVar)->S;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_trackVehicleNewVer7_SonarPath_optimization_info();
    }

    finalize_c4_trackVehicleNewVer7_SonarPath
      ((SFc4_trackVehicleNewVer7_SonarPathInstanceStruct*) chartInstanceVar);
    utFree(chartInstanceVar);
    if (crtInfo != NULL) {
      utFree(crtInfo);
    }

    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc4_trackVehicleNewVer7_SonarPath
    ((SFc4_trackVehicleNewVer7_SonarPathInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c4_trackVehicleNewVer7_SonarPath(SimStruct *S)
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
    initialize_params_c4_trackVehicleNewVer7_SonarPath
      ((SFc4_trackVehicleNewVer7_SonarPathInstanceStruct*)
       (chartInfo->chartInstance));
  }
}

static void mdlSetWorkWidths_c4_trackVehicleNewVer7_SonarPath(SimStruct *S)
{
  ssMdlUpdateIsEmpty(S, 1);
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_trackVehicleNewVer7_SonarPath_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,4);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,1);
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,4,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(sf_get_instance_specialization(),infoStruct,4);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 4, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 5, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 6, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,4,7);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,4,2);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=2; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 7; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,4);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(1855775647U));
  ssSetChecksum1(S,(3854655954U));
  ssSetChecksum2(S,(2607960893U));
  ssSetChecksum3(S,(3601176490U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c4_trackVehicleNewVer7_SonarPath(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c4_trackVehicleNewVer7_SonarPath(SimStruct *S)
{
  SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *chartInstance;
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)utMalloc(sizeof
    (ChartRunTimeInfo));
  chartInstance = (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct *)utMalloc
    (sizeof(SFc4_trackVehicleNewVer7_SonarPathInstanceStruct));
  memset(chartInstance, 0, sizeof
         (SFc4_trackVehicleNewVer7_SonarPathInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c4_trackVehicleNewVer7_SonarPath;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c4_trackVehicleNewVer7_SonarPath;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c4_trackVehicleNewVer7_SonarPath;
  chartInstance->chartInfo.enableChart =
    sf_opaque_enable_c4_trackVehicleNewVer7_SonarPath;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c4_trackVehicleNewVer7_SonarPath;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c4_trackVehicleNewVer7_SonarPath;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c4_trackVehicleNewVer7_SonarPath;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c4_trackVehicleNewVer7_SonarPath;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c4_trackVehicleNewVer7_SonarPath;
  chartInstance->chartInfo.mdlStart = mdlStart_c4_trackVehicleNewVer7_SonarPath;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c4_trackVehicleNewVer7_SonarPath;
  chartInstance->chartInfo.extModeExec = NULL;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->chartInfo.callAtomicSubchartUserFcn = NULL;
  chartInstance->chartInfo.callAtomicSubchartAutoFcn = NULL;
  chartInstance->chartInfo.debugInstance = sfGlobalDebugInstanceStruct;
  chartInstance->S = S;
  crtInfo->isEnhancedMooreMachine = 0;
  crtInfo->checksum = SF_RUNTIME_INFO_CHECKSUM;
  crtInfo->fCheckOverflow = sf_runtime_overflow_check_is_on(S);
  crtInfo->instanceInfo = (&(chartInstance->chartInfo));
  crtInfo->isJITEnabled = false;
  crtInfo->compiledInfo = NULL;
  ssSetUserData(S,(void *)(crtInfo));  /* register the chart instance with simstruct */
  init_dsm_address_info(chartInstance);
  init_simulink_io_address(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  chart_debug_initialization(S,1);
}

void c4_trackVehicleNewVer7_SonarPath_method_dispatcher(SimStruct *S, int_T
  method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c4_trackVehicleNewVer7_SonarPath(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c4_trackVehicleNewVer7_SonarPath(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c4_trackVehicleNewVer7_SonarPath(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c4_trackVehicleNewVer7_SonarPath_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
