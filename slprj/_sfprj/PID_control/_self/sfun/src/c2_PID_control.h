#ifndef __c2_PID_control_h__
#define __c2_PID_control_h__

/* Type Definitions */
#ifndef typedef_SFc2_PID_controlInstanceStruct
#define typedef_SFc2_PID_controlInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c2_sfEvent;
  boolean_T c2_doneDoubleBufferReInit;
  uint8_T c2_is_active_c2_PID_control;
  void *c2_fEmlrtCtx;
  real_T *c2_xddot;
  real_T *c2_xdot;
  real_T *c2_ydot;
  real_T *c2_yddot;
  real_T *c2_zdot;
  real_T *c2_txdot;
  real_T *c2_tzdot;
  real_T *c2_zddot;
  real_T *c2_txddot;
  real_T *c2_tzddot;
  real_T (*c2_force)[5];
} SFc2_PID_controlInstanceStruct;

#endif                                 /*typedef_SFc2_PID_controlInstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c2_PID_control_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c2_PID_control_get_check_sum(mxArray *plhs[]);
extern void c2_PID_control_method_dispatcher(SimStruct *S, int_T method, void
  *data);

#endif
