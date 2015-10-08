#ifndef __c4_trackVehicleNewVer7_SonarPath_h__
#define __c4_trackVehicleNewVer7_SonarPath_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc4_trackVehicleNewVer7_SonarPathInstanceStruct
#define typedef_SFc4_trackVehicleNewVer7_SonarPathInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c4_sfEvent;
  boolean_T c4_isStable;
  boolean_T c4_doneDoubleBufferReInit;
  uint8_T c4_is_active_c4_trackVehicleNewVer7_SonarPath;
  real_T *c4_x;
  real_T *c4_y;
  real_T *c4_heading;
  real_T (*c4_goals)[2];
  real_T (*c4_OccGrid)[900];
  real_T (*c4_path)[1800];
  real_T (*c4_cellsize)[2];
  real_T (*c4_gridorig)[2];
  real_T *c4_pathlen;
} SFc4_trackVehicleNewVer7_SonarPathInstanceStruct;

#endif                                 /*typedef_SFc4_trackVehicleNewVer7_SonarPathInstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray
  *sf_c4_trackVehicleNewVer7_SonarPath_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c4_trackVehicleNewVer7_SonarPath_get_check_sum(mxArray *plhs[]);
extern void c4_trackVehicleNewVer7_SonarPath_method_dispatcher(SimStruct *S,
  int_T method, void *data);

#endif
