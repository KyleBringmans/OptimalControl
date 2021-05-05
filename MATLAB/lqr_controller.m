function K = lqr_controller(a)
%Design of an lqr controller

%input: a, the structure with the system matrices
% as returned by att_lin_system
%output: gain K


%Parameter matrices Q and R
% The larger Q is, the more aggressive the system is
Q = diag([1e3 1e3 1e3 1e-4 1e-4 1e-4 1e-5 1e-5 1e-5]);
% The larger R is, the slower/smoother the system is
R = eye(3);


%Calculation of gain matrix K
K = -dlqr(a.Ad ,a.Bd , Q , R );

%Check if stable
assert ( all (abs( eig( a.Ad + a.Bd*K )) < 1 - 0.000001) ,...
    'A+BK not stable');








