import numpy
import ctypes

name = "FORCES_NLP_solver"
requires_callback = True
lib = "lib/libFORCES_NLP_solver.so"
lib_static = "lib/libFORCES_NLP_solver.a"
c_header = "include/FORCES_NLP_solver.h"
nstages = 50

# Parameter             | Type    | Scalar type      | Ctypes type    | Numpy type   | Shape     | Len
params = \
[("xinit"               , "dense" , ""               , ctypes.c_double, numpy.float64, (  6,   1),    6),
 ("x0"                  , "dense" , ""               , ctypes.c_double, numpy.float64, (400,   1),  400),
 ("reinitialize"        , ""      , "FORCES_NLP_solver_int", ctypes.c_int   , numpy.int32  , (  0,   1),    1)]

# Output                | Type    | Scalar type      | Ctypes type    | Numpy type   | Shape     | Len
outputs = \
[("x01"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  8,),    8),
 ("x02"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  8,),    8),
 ("x03"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  8,),    8),
 ("x04"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  8,),    8),
 ("x05"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  8,),    8),
 ("x06"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  8,),    8),
 ("x07"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  8,),    8),
 ("x08"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  8,),    8),
 ("x09"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  8,),    8),
 ("x10"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  8,),    8),
 ("x11"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  8,),    8),
 ("x12"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  8,),    8),
 ("x13"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  8,),    8),
 ("x14"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  8,),    8),
 ("x15"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  8,),    8),
 ("x16"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  8,),    8),
 ("x17"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  8,),    8),
 ("x18"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  8,),    8),
 ("x19"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  8,),    8),
 ("x20"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  8,),    8),
 ("x21"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  8,),    8),
 ("x22"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  8,),    8),
 ("x23"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  8,),    8),
 ("x24"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  8,),    8),
 ("x25"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  8,),    8),
 ("x26"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  8,),    8),
 ("x27"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  8,),    8),
 ("x28"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  8,),    8),
 ("x29"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  8,),    8),
 ("x30"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  8,),    8),
 ("x31"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  8,),    8),
 ("x32"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  8,),    8),
 ("x33"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  8,),    8),
 ("x34"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  8,),    8),
 ("x35"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  8,),    8),
 ("x36"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  8,),    8),
 ("x37"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  8,),    8),
 ("x38"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  8,),    8),
 ("x39"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  8,),    8),
 ("x40"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  8,),    8),
 ("x41"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  8,),    8),
 ("x42"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  8,),    8),
 ("x43"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  8,),    8),
 ("x44"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  8,),    8),
 ("x45"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  8,),    8),
 ("x46"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  8,),    8),
 ("x47"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  8,),    8),
 ("x48"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  8,),    8),
 ("x49"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  8,),    8),
 ("x50"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  8,),    8)]

# Info Struct Fields
info = \
[("it", ctypes.c_int),
 ("res_eq", ctypes.c_double),
 ("rsnorm", ctypes.c_double),
 ("pobj", ctypes.c_double),
 ("solvetime", ctypes.c_double),
 ("fevalstime", ctypes.c_double),
 ("QPtime", ctypes.c_double)]

# Dynamics dimensions
#   nvar    |   neq   |   dimh    |   dimp    |   diml    |   dimu    |   dimhl   |   dimhu    
dynamics_dims = [
	(8, 6, 0, 0, 2, 2, 0, 0), 
	(8, 6, 0, 0, 2, 2, 0, 0), 
	(8, 6, 0, 0, 2, 2, 0, 0), 
	(8, 6, 0, 0, 2, 2, 0, 0), 
	(8, 6, 0, 0, 2, 2, 0, 0), 
	(8, 6, 0, 0, 2, 2, 0, 0), 
	(8, 6, 0, 0, 2, 2, 0, 0), 
	(8, 6, 0, 0, 2, 2, 0, 0), 
	(8, 6, 0, 0, 2, 2, 0, 0), 
	(8, 6, 0, 0, 2, 2, 0, 0), 
	(8, 6, 0, 0, 2, 2, 0, 0), 
	(8, 6, 0, 0, 2, 2, 0, 0), 
	(8, 6, 0, 0, 2, 2, 0, 0), 
	(8, 6, 0, 0, 2, 2, 0, 0), 
	(8, 6, 0, 0, 2, 2, 0, 0), 
	(8, 6, 0, 0, 2, 2, 0, 0), 
	(8, 6, 0, 0, 2, 2, 0, 0), 
	(8, 6, 0, 0, 2, 2, 0, 0), 
	(8, 6, 0, 0, 2, 2, 0, 0), 
	(8, 6, 0, 0, 2, 2, 0, 0), 
	(8, 6, 0, 0, 2, 2, 0, 0), 
	(8, 6, 0, 0, 2, 2, 0, 0), 
	(8, 6, 0, 0, 2, 2, 0, 0), 
	(8, 6, 0, 0, 2, 2, 0, 0), 
	(8, 6, 0, 0, 2, 2, 0, 0), 
	(8, 6, 0, 0, 2, 2, 0, 0), 
	(8, 6, 0, 0, 2, 2, 0, 0), 
	(8, 6, 0, 0, 2, 2, 0, 0), 
	(8, 6, 0, 0, 2, 2, 0, 0), 
	(8, 6, 0, 0, 2, 2, 0, 0), 
	(8, 6, 0, 0, 2, 2, 0, 0), 
	(8, 6, 0, 0, 2, 2, 0, 0), 
	(8, 6, 0, 0, 2, 2, 0, 0), 
	(8, 6, 0, 0, 2, 2, 0, 0), 
	(8, 6, 0, 0, 2, 2, 0, 0), 
	(8, 6, 0, 0, 2, 2, 0, 0), 
	(8, 6, 0, 0, 2, 2, 0, 0), 
	(8, 6, 0, 0, 2, 2, 0, 0), 
	(8, 6, 0, 0, 2, 2, 0, 0), 
	(8, 6, 0, 0, 2, 2, 0, 0), 
	(8, 6, 0, 0, 2, 2, 0, 0), 
	(8, 6, 0, 0, 2, 2, 0, 0), 
	(8, 6, 0, 0, 2, 2, 0, 0), 
	(8, 6, 0, 0, 2, 2, 0, 0), 
	(8, 6, 0, 0, 2, 2, 0, 0), 
	(8, 6, 0, 0, 2, 2, 0, 0), 
	(8, 6, 0, 0, 2, 2, 0, 0), 
	(8, 6, 0, 0, 2, 2, 0, 0), 
	(8, 6, 0, 0, 2, 2, 0, 0), 
	(8, 6, 0, 0, 2, 2, 0, 0)
]