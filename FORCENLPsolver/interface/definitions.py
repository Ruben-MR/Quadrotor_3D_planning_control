import numpy
import ctypes

name = "FORCENLPsolver"
requires_callback = True
lib = "lib/libFORCENLPsolver.so"
lib_static = "lib/libFORCENLPsolver.a"
c_header = "include/FORCENLPsolver.h"
nstages = 50

# Parameter             | Type    | Scalar type      | Ctypes type    | Numpy type   | Shape     | Len
params = \
[("xinit"               , "dense" , ""               , ctypes.c_double, numpy.float64, ( 13,   1),   13),
 ("x0"                  , "dense" , ""               , ctypes.c_double, numpy.float64, (850,   1),  850),
 ("all_parameters"      , "dense" , ""               , ctypes.c_double, numpy.float64, (450,   1),  450),
 ("reinitialize"        , ""      , "FORCENLPsolver_int", ctypes.c_int   , numpy.int32  , (  0,   1),    1)]

# Output                | Type    | Scalar type      | Ctypes type    | Numpy type   | Shape     | Len
outputs = \
[("x01"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x02"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x03"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x04"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x05"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x06"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x07"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x08"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x09"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x10"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x11"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x12"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x13"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x14"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x15"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x16"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x17"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x18"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x19"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x20"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x21"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x22"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x23"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x24"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x25"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x26"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x27"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x28"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x29"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x30"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x31"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x32"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x33"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x34"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x35"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x36"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x37"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x38"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x39"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x40"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x41"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x42"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x43"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x44"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x45"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x46"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x47"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x48"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x49"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x50"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17)]

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
	(17, 13, 1, 9, 1, 1, 1, 0), 
	(17, 13, 1, 9, 1, 1, 1, 0), 
	(17, 13, 1, 9, 1, 1, 1, 0), 
	(17, 13, 1, 9, 1, 1, 1, 0), 
	(17, 13, 1, 9, 1, 1, 1, 0), 
	(17, 13, 1, 9, 1, 1, 1, 0), 
	(17, 13, 1, 9, 1, 1, 1, 0), 
	(17, 13, 1, 9, 1, 1, 1, 0), 
	(17, 13, 1, 9, 1, 1, 1, 0), 
	(17, 13, 1, 9, 1, 1, 1, 0), 
	(17, 13, 1, 9, 1, 1, 1, 0), 
	(17, 13, 1, 9, 1, 1, 1, 0), 
	(17, 13, 1, 9, 1, 1, 1, 0), 
	(17, 13, 1, 9, 1, 1, 1, 0), 
	(17, 13, 1, 9, 1, 1, 1, 0), 
	(17, 13, 1, 9, 1, 1, 1, 0), 
	(17, 13, 1, 9, 1, 1, 1, 0), 
	(17, 13, 1, 9, 1, 1, 1, 0), 
	(17, 13, 1, 9, 1, 1, 1, 0), 
	(17, 13, 1, 9, 1, 1, 1, 0), 
	(17, 13, 1, 9, 1, 1, 1, 0), 
	(17, 13, 1, 9, 1, 1, 1, 0), 
	(17, 13, 1, 9, 1, 1, 1, 0), 
	(17, 13, 1, 9, 1, 1, 1, 0), 
	(17, 13, 1, 9, 1, 1, 1, 0), 
	(17, 13, 1, 9, 1, 1, 1, 0), 
	(17, 13, 1, 9, 1, 1, 1, 0), 
	(17, 13, 1, 9, 1, 1, 1, 0), 
	(17, 13, 1, 9, 1, 1, 1, 0), 
	(17, 13, 1, 9, 1, 1, 1, 0), 
	(17, 13, 1, 9, 1, 1, 1, 0), 
	(17, 13, 1, 9, 1, 1, 1, 0), 
	(17, 13, 1, 9, 1, 1, 1, 0), 
	(17, 13, 1, 9, 1, 1, 1, 0), 
	(17, 13, 1, 9, 1, 1, 1, 0), 
	(17, 13, 1, 9, 1, 1, 1, 0), 
	(17, 13, 1, 9, 1, 1, 1, 0), 
	(17, 13, 1, 9, 1, 1, 1, 0), 
	(17, 13, 1, 9, 1, 1, 1, 0), 
	(17, 13, 1, 9, 1, 1, 1, 0), 
	(17, 13, 1, 9, 1, 1, 1, 0), 
	(17, 13, 1, 9, 1, 1, 1, 0), 
	(17, 13, 1, 9, 1, 1, 1, 0), 
	(17, 13, 1, 9, 1, 1, 1, 0), 
	(17, 13, 1, 9, 1, 1, 1, 0), 
	(17, 13, 1, 9, 1, 1, 1, 0), 
	(17, 13, 1, 9, 1, 1, 1, 0), 
	(17, 13, 1, 9, 1, 1, 1, 0), 
	(17, 13, 1, 9, 1, 1, 1, 0), 
	(17, 13, 1, 9, 1, 1, 1, 0)
]