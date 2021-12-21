#ifndef FORCENLPsolver_H
#define FORCENLPsolver_H
/* Generated by FORCESPRO v5.1.0 on Friday, December 17, 2021 at 8:03:15 PM */

#ifndef SOLVER_STDIO_H
#define SOLVER_STDIO_H
#include <stdio.h>
#include <stddef.h>
#endif


/* DATA TYPE ------------------------------------------------------------*/
typedef double FORCENLPsolver_float;
typedef double FORCENLPsolver_callback_float;
typedef double FORCENLPsolverinterface_float;
typedef int FORCENLPsolver_int;

#ifndef SOLVER_STANDARD_TYPES
#define SOLVER_STANDARD_TYPES

typedef signed char solver_int8_signed;
typedef unsigned char solver_int8_unsigned;
typedef char solver_int8_default;
typedef signed short int solver_int16_signed;
typedef unsigned short int solver_int16_unsigned;
typedef short int solver_int16_default;
typedef signed int solver_int32_signed;
typedef unsigned int solver_int32_unsigned;
typedef int solver_int32_default;
typedef signed long long int solver_int64_signed;
typedef unsigned long long int solver_int64_unsigned;
typedef long long int solver_int64_default;

#endif

/* SOLVER SETTINGS ------------------------------------------------------*/

/* MISRA-C compliance */
#ifndef MISRA_C_FORCENLPsolver
#define MISRA_C_FORCENLPsolver (0)
#endif

/* restrict code */
#ifndef RESTRICT_CODE_FORCENLPsolver
#define RESTRICT_CODE_FORCENLPsolver (0)
#endif

/* print level */
#ifndef SET_PRINTLEVEL_FORCENLPsolver
#define SET_PRINTLEVEL_FORCENLPsolver    (0)
#endif

/* timing */
#ifndef SET_TIMING_FORCENLPsolver
#define SET_TIMING_FORCENLPsolver    (1)
#endif

/* Numeric Warnings */
/* #define PRINTNUMERICALWARNINGS */

/* maximum number of iterations  */
#define SET_MAXIT_FORCENLPsolver			(200)	 

/* desired maximum residual on equality constraints */
#define SET_ACC_RESEQ_FORCENLPsolver		(FORCENLPsolver_float)(1E-06)


/* RETURN CODES----------------------------------------------------------*/
/* solver has converged within desired accuracy */
#define OPTIMAL_FORCENLPsolver      (1)

/* maximum number of iterations has been reached */
#define MAXITREACHED_FORCENLPsolver (0)

/* solver has stopped due to a timeout */
#define TIMEOUT_FORCENLPsolver   (2)

/* NaN encountered in function evaluations */
#define BADFUNCEVAL_FORCENLPsolver  (-6)

/* no progress in method possible */
#define NOPROGRESS_FORCENLPsolver   (-7)

/* regularization error */
#define REGULARIZATION_ERROR_FORCENLPsolver   (-9)

/* invalid values in parameters */
#define PARAM_VALUE_ERROR_FORCENLPsolver   (-11)

/* too small timeout given */
#define INVALID_TIMEOUT_FORCENLPsolver   (-12)

/* error in linesearch */
#define LINESEARCH_ERROR_FORCENLPsolver   (-13)

/* thread error */
#define THREAD_FAILURE_FORCENLPsolver  (-98)

/* locking mechanism error */
#define LOCK_FAILURE_FORCENLPsolver  (-99)

/* licensing error - solver not valid on this machine */
#define LICENSE_ERROR_FORCENLPsolver  (-100)

/* qp solver error */
#define QP_SOLVER_FAILURE_FORCENLPsolver (-8)


/* INTEGRATORS RETURN CODE ------------*/
/* Integrator ran successfully */
#define INTEGRATOR_SUCCESS (11)
/* Number of steps set by user exceeds maximum number of steps allowed */
#define INTEGRATOR_MAXSTEPS_EXCEEDED (12)

/* PARAMETERS -----------------------------------------------------------*/
/* fill this with data before calling the solver! */
typedef struct
{
	    /* vector of size 13 */
    FORCENLPsolver_float xinit[13];

    /* vector of size 850 */
    FORCENLPsolver_float x0[850];

    /* vector of size 600 */
    FORCENLPsolver_float all_parameters[600];

    /* scalar */
    FORCENLPsolver_int reinitialize;


} FORCENLPsolver_params;


/* OUTPUTS --------------------------------------------------------------*/
/* the desired variables are put here by the solver */
typedef struct
{
	    /* vector of size 17 */
    FORCENLPsolver_float x01[17];

    /* vector of size 17 */
    FORCENLPsolver_float x02[17];

    /* vector of size 17 */
    FORCENLPsolver_float x03[17];

    /* vector of size 17 */
    FORCENLPsolver_float x04[17];

    /* vector of size 17 */
    FORCENLPsolver_float x05[17];

    /* vector of size 17 */
    FORCENLPsolver_float x06[17];

    /* vector of size 17 */
    FORCENLPsolver_float x07[17];

    /* vector of size 17 */
    FORCENLPsolver_float x08[17];

    /* vector of size 17 */
    FORCENLPsolver_float x09[17];

    /* vector of size 17 */
    FORCENLPsolver_float x10[17];

    /* vector of size 17 */
    FORCENLPsolver_float x11[17];

    /* vector of size 17 */
    FORCENLPsolver_float x12[17];

    /* vector of size 17 */
    FORCENLPsolver_float x13[17];

    /* vector of size 17 */
    FORCENLPsolver_float x14[17];

    /* vector of size 17 */
    FORCENLPsolver_float x15[17];

    /* vector of size 17 */
    FORCENLPsolver_float x16[17];

    /* vector of size 17 */
    FORCENLPsolver_float x17[17];

    /* vector of size 17 */
    FORCENLPsolver_float x18[17];

    /* vector of size 17 */
    FORCENLPsolver_float x19[17];

    /* vector of size 17 */
    FORCENLPsolver_float x20[17];

    /* vector of size 17 */
    FORCENLPsolver_float x21[17];

    /* vector of size 17 */
    FORCENLPsolver_float x22[17];

    /* vector of size 17 */
    FORCENLPsolver_float x23[17];

    /* vector of size 17 */
    FORCENLPsolver_float x24[17];

    /* vector of size 17 */
    FORCENLPsolver_float x25[17];

    /* vector of size 17 */
    FORCENLPsolver_float x26[17];

    /* vector of size 17 */
    FORCENLPsolver_float x27[17];

    /* vector of size 17 */
    FORCENLPsolver_float x28[17];

    /* vector of size 17 */
    FORCENLPsolver_float x29[17];

    /* vector of size 17 */
    FORCENLPsolver_float x30[17];

    /* vector of size 17 */
    FORCENLPsolver_float x31[17];

    /* vector of size 17 */
    FORCENLPsolver_float x32[17];

    /* vector of size 17 */
    FORCENLPsolver_float x33[17];

    /* vector of size 17 */
    FORCENLPsolver_float x34[17];

    /* vector of size 17 */
    FORCENLPsolver_float x35[17];

    /* vector of size 17 */
    FORCENLPsolver_float x36[17];

    /* vector of size 17 */
    FORCENLPsolver_float x37[17];

    /* vector of size 17 */
    FORCENLPsolver_float x38[17];

    /* vector of size 17 */
    FORCENLPsolver_float x39[17];

    /* vector of size 17 */
    FORCENLPsolver_float x40[17];

    /* vector of size 17 */
    FORCENLPsolver_float x41[17];

    /* vector of size 17 */
    FORCENLPsolver_float x42[17];

    /* vector of size 17 */
    FORCENLPsolver_float x43[17];

    /* vector of size 17 */
    FORCENLPsolver_float x44[17];

    /* vector of size 17 */
    FORCENLPsolver_float x45[17];

    /* vector of size 17 */
    FORCENLPsolver_float x46[17];

    /* vector of size 17 */
    FORCENLPsolver_float x47[17];

    /* vector of size 17 */
    FORCENLPsolver_float x48[17];

    /* vector of size 17 */
    FORCENLPsolver_float x49[17];

    /* vector of size 17 */
    FORCENLPsolver_float x50[17];


} FORCENLPsolver_output;


/* SOLVER INFO ----------------------------------------------------------*/
/* diagnostic data from last interior point step */
typedef struct
{
	/* iteration number */
	solver_int32_default it;

	/* inf-norm of equality constraint residuals */
	FORCENLPsolver_float res_eq;

	/* norm of stationarity condition */
	FORCENLPsolver_float rsnorm;

	/* primal objective */
	FORCENLPsolver_float pobj;

	/* total solve time */
	FORCENLPsolver_float solvetime;

	/* time spent in function evaluations */
	FORCENLPsolver_float fevalstime;

	/* time spent solving QPs */
	FORCENLPsolver_float QPtime;
} FORCENLPsolver_info;



/* SOLVER FUNCTION DEFINITION -------------------------------------------*/
/* Time of Solver Generation: (UTC) Friday, December 17, 2021 8:03:17 PM */
/* User License expires on: (UTC) Wednesday, June 1, 2022 10:00:00 PM (approx.) (at the time of code generation) */
/* Solver Static License expires on: (UTC) Wednesday, June 1, 2022 10:00:00 PM (approx.) */
/* Solver Generation Request Id: 2175d245-523e-4a40-ad25-852f8322f47d */
/* examine exitflag before using the result! */
#ifdef __cplusplus
extern "C" {
#endif		

typedef void(*FORCENLPsolver_extfunc)(FORCENLPsolver_float* x, FORCENLPsolver_float* y, FORCENLPsolver_float* lambda, FORCENLPsolver_float* params, FORCENLPsolver_float* pobj, FORCENLPsolver_float* g, FORCENLPsolver_float* c, FORCENLPsolver_float* Jeq, FORCENLPsolver_float* h, FORCENLPsolver_float* Jineq, FORCENLPsolver_float* H, solver_int32_default stage, solver_int32_default iterations, solver_int32_default threadID);

extern solver_int32_default FORCENLPsolver_solve(FORCENLPsolver_params *params, FORCENLPsolver_output *output, FORCENLPsolver_info *info, FILE *fs, FORCENLPsolver_extfunc evalextfunctions_FORCENLPsolver);







#ifdef __cplusplus
}
#endif

#endif
