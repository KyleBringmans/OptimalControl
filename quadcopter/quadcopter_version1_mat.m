clear;
load 'references_21';
%%
% massa quadcopter
mass = 0.5;
% radius quadcopter
L = 0.25;
% Propeller lift coefficient 
k = 3*10^(-6);
% Propeller drag coefficient
b = 10^(-7);
% Acceleration of gravity
g = 9.81;
% Air friction coefficient
Kd = 0.25;
% Quadcopter inertia about the x-axis
Ixx = 5*10^(-3);
% Quadcopter inertia about the y-axis
Iyy = 5*10^(-3);
% Quadcopter inertia about the z-axis
Izz = 1*10^(-2);
% Motor constant
cm = 10^4;
% sum of all u_i
UU = 163.5;
% Sampling time
Ts = 1/20;
%%

A = zeros(12);
A(1,4) = 1;
A(2,5) = 1;
A(3,6) = 1;
A(4,4) = -Kd/mass;
A(4,8) = k*cm/mass*UU;
A(5,5) = -Kd/mass;
A(5,7) = -k*cm/mass*UU;
A(6,6) = -Kd/mass;
A(7,10) = 1;
A(8,11) = 1;
A(9,12) = 1;

B = zeros(12,4);
B(6,:) = k*cm/mass;
B(10,1) = L*k*cm/Ixx;
B(10,3) = -L*k*cm/Ixx;
B(11,2) = L*k*cm/Iyy;
B(11,4) = -L*k*cm/Iyy;
B(12,1) = b*cm/Izz;
B(12,2) = -b*cm/Izz;
B(12,3) = b*cm/Izz;
B(12,4) = -b*cm/Izz;

C = zeros(6,12);
C(1,1)=1;
C(2,2)=1;
C(3,3)=1;
C(4,7)=1;
C(5,8)=1;
C(6,9)=1;

D = zeros(6,4);

Q = diag([10 10 1000 10^-4 10^-4 10^-1 10 10 10 10^-1 10^-1 10^-2]);
R = eye(4,4)*0.1;

sys_c = ss(A,B,C,zeros(6,4));
sys = c2d(sys_c,Ts,'zoh');

K1 = lqr(sys,Q,R);

N_tot = [sys.A-eye(12) sys.B; sys.C sys.D] \ [zeros(12,6) ; eye(6)];
N_x = N_tot(1:12,:);
N_u = N_tot(13:end,:);

%%
%LQI

Q_i = diag([100 100 10 10 10 10 100 100 100 1 1 10 100 100 100]);
R_i = eye(4,4)*0.001;

Ki = lqi_custom(sys,Q_i,R_i);

%%
% Signal noise covariance
% Or just use Q from LQR? see p. 182 in course notes since B_1 = I
% Qk = I * kalman_var
kalman_var = 1e-4;
% Process noise covariance
Rk = diag([2.5e-5 2.5e-5 2.5e-5 7.57e-5 7.57e-5 7.57e-5]);

%%
% Pole placement
% Todo: pole placement for K, Nu and Nx

% Get continuous poles of the system
p_c = pole(sys_c);
p_c(1) = 1e-5;
p_c(2) = 1e-5;
p_c(3) = 1e-5;
% Rule of thumb
p_c = 3*p_c;
% Make them discrete
p_d = exp(p_c.*Ts);
% Place the poles (Sylvester)
L = place(sys.A',sys.C',p_d).';

sys_pp = ss(sys.A-L*sys.C, [sys.B-L*sys.D L], eye(12,12), zeros(12,10));



