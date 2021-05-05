function [Kx, Kz] = getK_alt(al)
% Design of lqi controller for the altitude controller.

% input: al, the structure with the system matrices
% as returned by alt_lin_system
% output: gains K

%Parameter matrices Q and R
% The larger Q is, the more aggressive the system is
nt = 1e1;
z = 1e3;
vz = 20;
y = 1/5;
Q = diag([nt z vz y]);
% The larger R is, the slower/smoother the system is
R = 1e-2;

%Calculation of gain matrix K
syst = al.sysd;
K = -lqi(syst, Q, R);
Kx = K(:,1:3);
Kz = K(:,4);

%Check if stable
assert ( all (abs( eig( al.Ad + al.Bd*Kx )) < 1 - 0.000001) ,...
    'A+BK not stable');

% Elaboration parameters:

% nt: increasing makes nt behave smoother (less abrubt)
% z: increasing flattens out z (less oscillation)
% vz: increasing causes smoother motion (more oscillation)
% y: increasing makes z converge faster (more oscillation)