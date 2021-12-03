clc;clear;close all;
path = [1,4,2,7,15, 2, 8];

n_order       = 7;% order of poly
n_seg         = size(path,2)-1;% segment number
n_poly_perseg = (n_order+1); % coef number of perseg

ts = zeros(n_seg, 1);
% % calculate time distribution in proportion to distance between 2 points
% dist     = zeros(n_seg, 1);
% dist_sum = 0;
% T        = 25;
% t_sum    = 0;
% 
% for i = 1:n_seg
%     dist(i) = sqrt((path(i+1, 1)-path(i, 1))^2 + (path(i+1, 2) - path(i, 2))^2);
%     dist_sum = dist_sum+dist(i);
% end
% for i = 1:n_seg-1
%     ts(i) = dist(i)/dist_sum*T;
%     t_sum = t_sum+ts(i);
% end
% ts(n_seg) = T - t_sum;

% simply set all time distribution as 1
for i = 1:n_seg
    ts(i) = 1.0;
end

poly_coef_x = MinimumSnapQPSolver(path, ts, n_seg, n_order);
poly_coef_v = polyder(poly_coef_x);
poly_coef_a = polyder(poly_coef_v);
poly_coef_j = polyder(poly_coef_a);
poly_coef_s = polyder(poly_coef_j);

% display the trajectory
X = []; V = []; A = []; J = []; S = [];
T = [];
k = 1;
tstep = 0.01;
for i=0:n_seg-1
    %#####################################################
    % STEP 3: get the coefficients of i-th segment of both x-axis
    % and y-axis
    Pxi = flipud(poly_coef_x(i*(n_order+1)+1:(i+1)*(n_order+1)));
    Pvi = polyder(Pxi);
    Pai = polyder(Pvi);
    Pji = polyder(Pai);
    Psi = polyder(Pji);
    for t = 0:tstep:ts(i+1)
        X(k)  = polyval(Pxi, t);
        V(k)  = polyval(Pvi, t);
        A(k)  = polyval(Pai, t);
        J(k)  = polyval(Pji, t);
        S(k)  = polyval(Psi, t);
        
        T(k) = t + sum(ts(1:i));
        k = k + 1;
    end
end

% plot
plot(T, X);
hold on
plot(T, V);
hold on
plot(T, A);
hold on
plot(T, J);
hold on
plot(T, S);
% hold on
scatter(0:n_seg, path);
legend('pos','vel','acc','jerk','snap','points')


function poly_coef = MinimumSnapQPSolver(waypoints, ts, n_seg, n_order)
    start_cond = [waypoints(1), 0, 0, 0];
    end_cond   = [waypoints(end), 0, 0, 0];
    %#####################################################
    % STEP 1: compute Q of p'Qp
    Q = getQ(n_seg, n_order, ts);
    %#####################################################
    % STEP 2: compute Aeq and beq 
    [Aeq, beq] = getAbeq(n_seg, n_order, waypoints, ts, start_cond, end_cond);
    f = zeros(size(Q,1),1);
    poly_coef = quadprog(Q,f,[],[],Aeq, beq);
end