function [Kx, Kz] = getK_nav(na)
% Design of lqi controller for the altitude controller.

% input: al, the structure with the system matrices
% as returned by alt_lin_system
% output: gains K

%Parameter matrices Q and R
% The larger Q is, the more aggressive the system is
q = 1e-1;
coord = 1e1;
v = 1/50;
y = 1e-5;
Q = diag([q q coord coord v v y y]);
% The larger R is, the slower/smoother the system is
R = 800 * eye(2);

%Calculation of gain matrix K
syst = na.sysd;
K = -lqi(syst, Q, R);
Kx = K(:,1:6);
Kz = K(:,7:8);

%Check if stable
assert ( all (abs( eig( na.Ad + na.Bd*Kx )) < 1 - 0.000001) ,...
    'A+BK not stable');

% PARAMETERS THAT KEEP DRONE LOITERING IN 5 CM ZONE (LAMBDA = 1)
%{
    %Parameter matrices Q and R
% The larger Q is, the more aggressive the system is
q = 1e-1;
coord = 1e1;
v = 1/50;
y = 1e0;
Q = diag([q q coord coord v v y y]);
% The larger R is, the slower/smoother the system is
R = 80 * eye(2) ;
%}
