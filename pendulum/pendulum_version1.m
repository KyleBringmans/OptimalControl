clear;
close all;
%%

% constants
Rm = 2.6;
Km = 0.00767;
Kb = 0.00767;
Kg = 3.7;
%M in pdf
Cart_m = 0.455;
%l in pdf
Rod_l  = 0.305;
%m in pdf
Rod_m  = 0.210;
%r in pdf
motor_radius = 0.00635;
g = 9.81;
%sampling time for 200Hz
Ts = 1/200;

%build matrices
A = [0 0 1 0;
    0 0 0 1;
    0 -Rod_m*g/Cart_m -(Kg^2)*Km*Kb/(Cart_m*Rm*motor_radius^2) 0;
    0 (Rod_m+Cart_m)*g/(Cart_m*Rod_l) (Kg^2)*Km*Kb/(Cart_m*Rm*motor_radius^2*Rod_l) 0];
B = [0;0;Km*Kg/(Cart_m*Rm*motor_radius);-Km*Kg/(Cart_m*Rm*motor_radius*Rod_l)];

C = [1 0 0 0
     0 1 0 0];

D = [0
     0];
 
% stability and controlability analysis
CO = ctrb(A,B);
% rank(CO) % rank(CO) > length(A)
% rank(A)
% observability
% OO=obsv(A,C);
% rank(OO) % < length(A) ?
% [V,d]=eig(A);
% minimal
sys=ss(A,B,C,D,-1);
%of
%sysm = ss(ssSys,'minimal')
%controller 
q1 = 10^(3);
q2 = 10^(3);
n_states = 4;
n_inputs = 1;
n_outputs = 2;
CC = [0.456/4.41 0; 0 90/6.328];

Ts = 1/200;
omega_c = 0.7*pi;

% Lower cutoff frequency => more stable derivatives => can handle higher
% noise levels


% more stable alpha (row 2) than x (row 1)
Q = [0.5 0 0 0;0 15 0 0;0 0 0 0;0 0 0 0];
% smaller R leads to quicker rection
R = 0.003;

sys =ss(A,B,C,D,Ts);
K = lqr(A,B,Q,R);
%%
% New (simpler) model for question 2

C_ez = eye(4);
D_ez = zeros(4,1);

%%

% Testing for report

Q2 = [0.25 0 0 0;0 15 0 0;0 0 0 0;0 0 0 0];
%smaller R leads to quicker rection
R2 = 0.005;
K2 = lqr(A,B,Q2,R2);

Q3 = [0.25 0 0 0;0 4 0 0;0 0 0 0;0 0 0 0];
%smaller R leads to quicker rection
R3 = 0.003;
K3 = lqr(A,B,Q3,R3);

%%
% step comparison

Ac = A-B*K;
Bc = B*K*[1;0;0;0];
Cc = sys.C-sys.D*K;
Dc = zeros(2,1)*K*[1;0;0;0];

sys_c1 = ss(Ac,Bc,Cc,Dc);
ans1 = stepinfo(sys_c1);

Ac = A-B*K2;
Bc = B*K2*[1;0;0;0];
Cc = sys.C-sys.D*K2;
Dc = zeros(2,1)*K2*[1;0;0;0];

sys_c2 = ss(Ac,Bc,Cc,Dc);
ans2 = stepinfo(sys_c2);

Ac = A-B*K3;
Bc = B*K3*[1;0;0;0];
Cc = sys.C-sys.D*K3;
Dc = zeros(2,1)*K3*[1;0;0;0];

sys_c3 = ss(Ac,Bc,Cc,Dc);
ans3 = stepinfo(sys_c3);

step(sys_c1); hold on; step(sys_c2); hold on; step(sys_c3); hold on;
legend('closed loop 2', 'closed loop 1', 'reference');


