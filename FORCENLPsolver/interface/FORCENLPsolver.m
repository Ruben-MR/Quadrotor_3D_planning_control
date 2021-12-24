% FORCENLPsolver - a fast solver generated by FORCESPRO v5.1.0
%
%   OUTPUT = FORCENLPsolver(PARAMS) solves a multistage problem
%   subject to the parameters supplied in the following struct:
%       PARAMS.lb - column vector of length 160
%       PARAMS.ub - column vector of length 160
%       PARAMS.xinit - column vector of length 13
%       PARAMS.x0 - column vector of length 680
%       PARAMS.all_parameters - column vector of length 240
%       PARAMS.reinitialize - scalar
%
%   OUTPUT returns the values of the last iteration of the solver where
%       OUTPUT.x01 - column vector of size 17
%       OUTPUT.x02 - column vector of size 17
%       OUTPUT.x03 - column vector of size 17
%       OUTPUT.x04 - column vector of size 17
%       OUTPUT.x05 - column vector of size 17
%       OUTPUT.x06 - column vector of size 17
%       OUTPUT.x07 - column vector of size 17
%       OUTPUT.x08 - column vector of size 17
%       OUTPUT.x09 - column vector of size 17
%       OUTPUT.x10 - column vector of size 17
%       OUTPUT.x11 - column vector of size 17
%       OUTPUT.x12 - column vector of size 17
%       OUTPUT.x13 - column vector of size 17
%       OUTPUT.x14 - column vector of size 17
%       OUTPUT.x15 - column vector of size 17
%       OUTPUT.x16 - column vector of size 17
%       OUTPUT.x17 - column vector of size 17
%       OUTPUT.x18 - column vector of size 17
%       OUTPUT.x19 - column vector of size 17
%       OUTPUT.x20 - column vector of size 17
%       OUTPUT.x21 - column vector of size 17
%       OUTPUT.x22 - column vector of size 17
%       OUTPUT.x23 - column vector of size 17
%       OUTPUT.x24 - column vector of size 17
%       OUTPUT.x25 - column vector of size 17
%       OUTPUT.x26 - column vector of size 17
%       OUTPUT.x27 - column vector of size 17
%       OUTPUT.x28 - column vector of size 17
%       OUTPUT.x29 - column vector of size 17
%       OUTPUT.x30 - column vector of size 17
%       OUTPUT.x31 - column vector of size 17
%       OUTPUT.x32 - column vector of size 17
%       OUTPUT.x33 - column vector of size 17
%       OUTPUT.x34 - column vector of size 17
%       OUTPUT.x35 - column vector of size 17
%       OUTPUT.x36 - column vector of size 17
%       OUTPUT.x37 - column vector of size 17
%       OUTPUT.x38 - column vector of size 17
%       OUTPUT.x39 - column vector of size 17
%       OUTPUT.x40 - column vector of size 17
%
%   [OUTPUT, EXITFLAG] = FORCENLPsolver(PARAMS) returns additionally
%   the integer EXITFLAG indicating the state of the solution with 
%       1 - OPTIMAL solution has been found (subject to desired accuracy)
%       0 - Timeout - maximum number of iterations reached
%      -7 - Method could not progress. Problem may be infeasible. Run FORCESdiagnostics on your problem to check for most common errors in the formulation.
%      -8 - QP solver failed. The QP solver failed to converge. Check your variable bounds and inequality constraints. Consider scaling your problem.
%    -100 - License error
%
%   [OUTPUT, EXITFLAG, INFO] = FORCENLPsolver(PARAMS) returns 
%   additional information about the last iterate:
%       INFO.it        - number of iterations that lead to this result
%       INFO.res_eq    - max. equality constraint residual
%       INFO.res_ineq  - max. inequality constraint residual
%       INFO.pobj      - primal objective
%       INFO.solvetime - Time needed for solve (wall clock time)
%       INFO.fevalstime - Time needed for function evaluations (wall clock time)
%       INFO.QPtime - Cumulative time spent in QP solver (wall clock time)
%
% See also COPYING
