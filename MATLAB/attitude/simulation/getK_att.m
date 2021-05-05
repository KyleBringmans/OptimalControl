function K = getK_att(system)
%Design of an lqr controller

%input: system, the structure with the system matrices
% as returned by att_lin_system
%output: gain K 


%Parameter matrices Q and R
quat        = 1e4;
omega       = 1e-2;
motorsignal = 1e-0;
input       = 1e-0;

% The larger Q is, the more aggressive the system is
Q = diag([quat quat quat omega omega 1e-3 ...
    motorsignal motorsignal motorsignal]);

% The larger R is, the slower/smoother the system is
R = diag([input input input]) ;


%Calculation of gain matrix K
K = -dlqr(system.Ad ,system.Bd , Q , R );

%Check if stable
assert ( all (abs( eig( system.Ad + system.Bd*K )) < 1 - 0.000001) ,...
    'A+BK not stable');

% Elaboration parameters:

% quat: increasing causes more abrubt rotation, ss-error increases
% omega: increasing causes less omega oscillation
% motorsignal: increasing causes smoother rotation (less abrubt)
% input: increasing causes smoother rotation (less abrubt)








