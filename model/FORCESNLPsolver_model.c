/* This file was automatically generated by CasADi.
   The CasADi copyright holders make no ownership claim of its contents. */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) FORCESNLPsolver_model_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real FORCESNLPsolver_float
#endif

#ifndef casadi_int
#define casadi_int solver_int32_default
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_f1 CASADI_PREFIX(f1)
#define casadi_f2 CASADI_PREFIX(f2)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)
#define casadi_s4 CASADI_PREFIX(s4)
#define casadi_s5 CASADI_PREFIX(s5)
#define casadi_sq CASADI_PREFIX(sq)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define CASADI_SYMBOL_EXPORT __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define CASADI_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif

casadi_real casadi_sq(casadi_real x) { return x*x;}

static const casadi_int casadi_s0[12] = {8, 1, 0, 8, 0, 1, 2, 3, 4, 5, 6, 7};
static const casadi_int casadi_s1[4] = {0, 1, 0, 0};
static const casadi_int casadi_s2[5] = {1, 1, 0, 1, 0};
static const casadi_int casadi_s3[13] = {1, 8, 0, 0, 0, 1, 2, 2, 2, 2, 2, 0, 0};
static const casadi_int casadi_s4[10] = {6, 1, 0, 6, 0, 1, 2, 3, 4, 5};
static const casadi_int casadi_s5[38] = {6, 8, 0, 4, 10, 11, 12, 17, 19, 21, 27, 0, 1, 3, 4, 0, 1, 2, 3, 4, 5, 0, 1, 0, 1, 2, 3, 4, 0, 3, 1, 4, 0, 1, 2, 3, 4, 5};

/* FORCESNLPsolver_objective_0:(i0[8],i1[0])->(o0,o1[1x8,2nz]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a2, a3;
  a0=arg[0]? arg[0][2] : 0;
  a1=5.0000000000000000e-01;
  a0=(a0-a1);
  a2=casadi_sq(a0);
  a3=arg[0]? arg[0][3] : 0;
  a3=(a3-a1);
  a1=casadi_sq(a3);
  a2=(a2+a1);
  if (res[0]!=0) res[0][0]=a2;
  a0=(a0+a0);
  if (res[1]!=0) res[1][0]=a0;
  a3=(a3+a3);
  if (res[1]!=0) res[1][1]=a3;
  return 0;
}

CASADI_SYMBOL_EXPORT int FORCESNLPsolver_objective_0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int FORCESNLPsolver_objective_0_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int FORCESNLPsolver_objective_0_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void FORCESNLPsolver_objective_0_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int FORCESNLPsolver_objective_0_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void FORCESNLPsolver_objective_0_release(int mem) {
}

CASADI_SYMBOL_EXPORT void FORCESNLPsolver_objective_0_incref(void) {
}

CASADI_SYMBOL_EXPORT void FORCESNLPsolver_objective_0_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int FORCESNLPsolver_objective_0_n_in(void) { return 2;}

CASADI_SYMBOL_EXPORT casadi_int FORCESNLPsolver_objective_0_n_out(void) { return 2;}

CASADI_SYMBOL_EXPORT casadi_real FORCESNLPsolver_objective_0_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* FORCESNLPsolver_objective_0_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* FORCESNLPsolver_objective_0_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* FORCESNLPsolver_objective_0_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* FORCESNLPsolver_objective_0_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s2;
    case 1: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int FORCESNLPsolver_objective_0_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 2;
  if (sz_res) *sz_res = 2;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

/* FORCESNLPsolver_dynamics_0:(i0[8],i1[0])->(o0[6],o1[6x8,27nz]) */
static int casadi_f1(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a4, a5, a6, a7, a8, a9;
  a0=arg[0]? arg[0][2] : 0;
  a1=1.6666666666666668e-03;
  a2=arg[0]? arg[0][5] : 0;
  a3=2.;
  a4=5.0000000000000001e-03;
  a5=arg[0]? arg[0][0] : 0;
  a6=2.9999999999999999e-02;
  a7=(a5/a6);
  a8=arg[0]? arg[0][4] : 0;
  a9=sin(a8);
  a10=(a7*a9);
  a11=(a4*a10);
  a11=(a2-a11);
  a11=(a3*a11);
  a11=(a2+a11);
  a12=(a5/a6);
  a13=arg[0]? arg[0][7] : 0;
  a14=(a4*a13);
  a14=(a8+a14);
  a15=sin(a14);
  a16=(a12*a15);
  a17=(a4*a16);
  a17=(a2-a17);
  a17=(a3*a17);
  a11=(a11+a17);
  a17=1.0000000000000000e-02;
  a18=(a5/a6);
  a19=arg[0]? arg[0][1] : 0;
  a20=1.4300000000000000e-05;
  a21=(a19/a20);
  a22=(a4*a21);
  a22=(a13+a22);
  a23=(a4*a22);
  a23=(a8+a23);
  a24=sin(a23);
  a25=(a18*a24);
  a26=(a17*a25);
  a26=(a2-a26);
  a11=(a11+a26);
  a11=(a1*a11);
  a0=(a0+a11);
  if (res[0]!=0) res[0][0]=a0;
  a0=arg[0]? arg[0][3] : 0;
  a11=arg[0]? arg[0][6] : 0;
  a26=-9.8100000000000005e+00;
  a27=(a5/a6);
  a28=cos(a8);
  a29=(a27*a28);
  a29=(a26+a29);
  a30=(a4*a29);
  a30=(a11+a30);
  a30=(a3*a30);
  a30=(a11+a30);
  a31=(a5/a6);
  a32=cos(a14);
  a33=(a31*a32);
  a33=(a26+a33);
  a34=(a4*a33);
  a34=(a11+a34);
  a34=(a3*a34);
  a30=(a30+a34);
  a34=(a5/a6);
  a35=cos(a23);
  a36=(a34*a35);
  a36=(a26+a36);
  a37=(a17*a36);
  a37=(a11+a37);
  a30=(a30+a37);
  a30=(a1*a30);
  a0=(a0+a30);
  if (res[0]!=0) res[0][1]=a0;
  a22=(a3*a22);
  a22=(a13+a22);
  a0=(a19/a20);
  a30=(a4*a0);
  a30=(a13+a30);
  a37=(a3*a30);
  a22=(a22+a37);
  a37=(a19/a20);
  a38=(a17*a37);
  a38=(a13+a38);
  a22=(a22+a38);
  a22=(a1*a22);
  a22=(a8+a22);
  if (res[0]!=0) res[0][2]=a22;
  a16=(a3*a16);
  a10=(a10+a16);
  a25=(a3*a25);
  a10=(a10+a25);
  a25=(a5/a6);
  a30=(a17*a30);
  a30=(a8+a30);
  a16=sin(a30);
  a22=(a25*a16);
  a10=(a10+a22);
  a10=(a1*a10);
  a2=(a2-a10);
  if (res[0]!=0) res[0][3]=a2;
  a33=(a3*a33);
  a29=(a29+a33);
  a36=(a3*a36);
  a29=(a29+a36);
  a5=(a5/a6);
  a6=cos(a30);
  a36=(a5*a6);
  a26=(a26+a36);
  a29=(a29+a26);
  a29=(a1*a29);
  a11=(a11+a29);
  if (res[0]!=0) res[0][4]=a11;
  a0=(a3*a0);
  a21=(a21+a0);
  a37=(a3*a37);
  a21=(a21+a37);
  a19=(a19/a20);
  a21=(a21+a19);
  a21=(a1*a21);
  a13=(a13+a21);
  if (res[0]!=0) res[0][5]=a13;
  a13=3.3333333333333336e+01;
  a9=(a13*a9);
  a21=(a4*a9);
  a21=(a3*a21);
  a15=(a13*a15);
  a19=(a4*a15);
  a19=(a3*a19);
  a21=(a21+a19);
  a24=(a13*a24);
  a19=(a17*a24);
  a21=(a21+a19);
  a21=(a1*a21);
  a21=(-a21);
  if (res[1]!=0) res[1][0]=a21;
  a28=(a13*a28);
  a21=(a4*a28);
  a21=(a3*a21);
  a32=(a13*a32);
  a19=(a4*a32);
  a19=(a3*a19);
  a21=(a21+a19);
  a35=(a13*a35);
  a19=(a17*a35);
  a21=(a21+a19);
  a21=(a1*a21);
  if (res[1]!=0) res[1][1]=a21;
  a15=(a3*a15);
  a9=(a9+a15);
  a24=(a3*a24);
  a9=(a9+a24);
  a16=(a13*a16);
  a9=(a9+a16);
  a9=(a1*a9);
  a9=(-a9);
  if (res[1]!=0) res[1][2]=a9;
  a32=(a3*a32);
  a28=(a28+a32);
  a35=(a3*a35);
  a28=(a28+a35);
  a13=(a13*a6);
  a28=(a28+a13);
  a28=(a1*a28);
  if (res[1]!=0) res[1][3]=a28;
  a28=1.7482517482517483e+00;
  a13=cos(a23);
  a6=(a28*a13);
  a6=(a18*a6);
  a35=(a17*a6);
  a35=(a1*a35);
  a35=(-a35);
  if (res[1]!=0) res[1][4]=a35;
  a23=sin(a23);
  a28=(a28*a23);
  a28=(a34*a28);
  a35=(a17*a28);
  a35=(a1*a35);
  a35=(-a35);
  if (res[1]!=0) res[1][5]=a35;
  a35=3.4965034965034967e+00;
  if (res[1]!=0) res[1][6]=a35;
  a6=(a3*a6);
  a32=cos(a30);
  a9=(a35*a32);
  a9=(a25*a9);
  a6=(a6+a9);
  a6=(a1*a6);
  a6=(-a6);
  if (res[1]!=0) res[1][7]=a6;
  a28=(a3*a28);
  a30=sin(a30);
  a35=(a35*a30);
  a35=(a5*a35);
  a28=(a28+a35);
  a28=(a1*a28);
  a28=(-a28);
  if (res[1]!=0) res[1][8]=a28;
  a28=6.9930069930069953e+02;
  if (res[1]!=0) res[1][9]=a28;
  a28=1.;
  if (res[1]!=0) res[1][10]=a28;
  if (res[1]!=0) res[1][11]=a28;
  a35=cos(a8);
  a7=(a7*a35);
  a35=(a4*a7);
  a35=(a3*a35);
  a6=cos(a14);
  a9=(a12*a6);
  a16=(a4*a9);
  a16=(a3*a16);
  a35=(a35+a16);
  a16=(a18*a13);
  a24=(a17*a16);
  a35=(a35+a24);
  a35=(a1*a35);
  a35=(-a35);
  if (res[1]!=0) res[1][12]=a35;
  a8=sin(a8);
  a27=(a27*a8);
  a8=(a4*a27);
  a8=(a3*a8);
  a14=sin(a14);
  a35=(a31*a14);
  a24=(a4*a35);
  a24=(a3*a24);
  a8=(a8+a24);
  a24=(a34*a23);
  a15=(a17*a24);
  a8=(a8+a15);
  a8=(a1*a8);
  a8=(-a8);
  if (res[1]!=0) res[1][13]=a8;
  if (res[1]!=0) res[1][14]=a28;
  a9=(a3*a9);
  a7=(a7+a9);
  a16=(a3*a16);
  a7=(a7+a16);
  a16=(a25*a32);
  a7=(a7+a16);
  a7=(a1*a7);
  a7=(-a7);
  if (res[1]!=0) res[1][15]=a7;
  a35=(a3*a35);
  a27=(a27+a35);
  a24=(a3*a24);
  a27=(a27+a24);
  a24=(a5*a30);
  a27=(a27+a24);
  a27=(a1*a27);
  a27=(-a27);
  if (res[1]!=0) res[1][16]=a27;
  if (res[1]!=0) res[1][17]=a17;
  if (res[1]!=0) res[1][18]=a28;
  if (res[1]!=0) res[1][19]=a17;
  if (res[1]!=0) res[1][20]=a28;
  a6=(a4*a6);
  a12=(a12*a6);
  a6=(a4*a12);
  a6=(a3*a6);
  a13=(a4*a13);
  a18=(a18*a13);
  a13=(a17*a18);
  a6=(a6+a13);
  a6=(a1*a6);
  a6=(-a6);
  if (res[1]!=0) res[1][21]=a6;
  a14=(a4*a14);
  a31=(a31*a14);
  a14=(a4*a31);
  a14=(a3*a14);
  a4=(a4*a23);
  a34=(a34*a4);
  a4=(a17*a34);
  a14=(a14+a4);
  a14=(a1*a14);
  a14=(-a14);
  if (res[1]!=0) res[1][22]=a14;
  if (res[1]!=0) res[1][23]=a17;
  a12=(a3*a12);
  a18=(a3*a18);
  a12=(a12+a18);
  a32=(a17*a32);
  a25=(a25*a32);
  a12=(a12+a25);
  a12=(a1*a12);
  a12=(-a12);
  if (res[1]!=0) res[1][24]=a12;
  a31=(a3*a31);
  a3=(a3*a34);
  a31=(a31+a3);
  a17=(a17*a30);
  a5=(a5*a17);
  a31=(a31+a5);
  a1=(a1*a31);
  a1=(-a1);
  if (res[1]!=0) res[1][25]=a1;
  if (res[1]!=0) res[1][26]=a28;
  return 0;
}

CASADI_SYMBOL_EXPORT int FORCESNLPsolver_dynamics_0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f1(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int FORCESNLPsolver_dynamics_0_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int FORCESNLPsolver_dynamics_0_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void FORCESNLPsolver_dynamics_0_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int FORCESNLPsolver_dynamics_0_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void FORCESNLPsolver_dynamics_0_release(int mem) {
}

CASADI_SYMBOL_EXPORT void FORCESNLPsolver_dynamics_0_incref(void) {
}

CASADI_SYMBOL_EXPORT void FORCESNLPsolver_dynamics_0_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int FORCESNLPsolver_dynamics_0_n_in(void) { return 2;}

CASADI_SYMBOL_EXPORT casadi_int FORCESNLPsolver_dynamics_0_n_out(void) { return 2;}

CASADI_SYMBOL_EXPORT casadi_real FORCESNLPsolver_dynamics_0_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* FORCESNLPsolver_dynamics_0_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* FORCESNLPsolver_dynamics_0_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* FORCESNLPsolver_dynamics_0_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* FORCESNLPsolver_dynamics_0_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s4;
    case 1: return casadi_s5;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int FORCESNLPsolver_dynamics_0_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 2;
  if (sz_res) *sz_res = 2;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

/* FORCESNLPsolver_objective_1:(i0[8],i1[0])->(o0,o1[1x8,2nz]) */
static int casadi_f2(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a2, a3;
  a0=arg[0]? arg[0][2] : 0;
  a1=5.0000000000000000e-01;
  a0=(a0-a1);
  a2=casadi_sq(a0);
  a3=arg[0]? arg[0][3] : 0;
  a3=(a3-a1);
  a1=casadi_sq(a3);
  a2=(a2+a1);
  if (res[0]!=0) res[0][0]=a2;
  a0=(a0+a0);
  if (res[1]!=0) res[1][0]=a0;
  a3=(a3+a3);
  if (res[1]!=0) res[1][1]=a3;
  return 0;
}

CASADI_SYMBOL_EXPORT int FORCESNLPsolver_objective_1(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f2(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int FORCESNLPsolver_objective_1_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int FORCESNLPsolver_objective_1_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void FORCESNLPsolver_objective_1_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int FORCESNLPsolver_objective_1_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void FORCESNLPsolver_objective_1_release(int mem) {
}

CASADI_SYMBOL_EXPORT void FORCESNLPsolver_objective_1_incref(void) {
}

CASADI_SYMBOL_EXPORT void FORCESNLPsolver_objective_1_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int FORCESNLPsolver_objective_1_n_in(void) { return 2;}

CASADI_SYMBOL_EXPORT casadi_int FORCESNLPsolver_objective_1_n_out(void) { return 2;}

CASADI_SYMBOL_EXPORT casadi_real FORCESNLPsolver_objective_1_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* FORCESNLPsolver_objective_1_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* FORCESNLPsolver_objective_1_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* FORCESNLPsolver_objective_1_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* FORCESNLPsolver_objective_1_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s2;
    case 1: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int FORCESNLPsolver_objective_1_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 2;
  if (sz_res) *sz_res = 2;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
