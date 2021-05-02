%clear all
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
A = [0 0 1 0;0 0 0 1;0 -Rod_m*g/Cart_m -(Kg^2)*Km*Kb/(Cart_m*Rm*motor_radius^2) 0
    0 (Rod_m+Cart_m)*g/(Cart_m*Rod_l) (Kg^2)*Km*Kb/(Cart_m*Rm*motor_radius^2*Rod_l) 0];
B = [0;0;Km*Kg/(Cart_m*Rm*motor_radius);-Km*Kg/(Cart_m*Rm*motor_radius*Rod_l)];
C = [1 0 0 0
 0 1 0 0];
D = [0
 0];
%Q = [0.25 0 0 0;0 4 0 0;0 0 0 0;0 0 0 0];
%R = 0.003;
%more stable alpha (row 2) than x (row 1)
Q = [0.25 0 0 0;0 400 0 0;0 0 0 0;0 0 0 0];
%smaller R leads to quicker rection
R = 0.003;
%stability and controlability analysis
eig(A) 
CO = ctrb(A,B)
rank(CO) % rank(CO) > length(A)
rank(A)
% observability
OO=obsv(A,C);
rank(OO) % < length(A) ?
[V,d]=eig(A);
C*V
%minimal
sys=ss(A,B,C,D,-1);
sysm=minreal(sys);
[Am,Bm,Cm,Dm]=ssdata(sysm);
%of
%sysm = ss(ssSys,'minimal')
%controller 
q1 = 10^(3);
q2 = 10^(3);
n_states = 4
n_inputs = 1
n_outputs = 2
Q1 = q1*eye(n_states);
Q2 = q2*eye(n_states);
%R  = eye(n_inputs);
%K = lqr(A,B,Q1,R);
K2 = lqr(A,B,Q2,R);
CC = eye(n_outputs)
%sys1 = ss(A-B2*K1,B1(:,1),[C2;C1;-D1*K1;-K1],0);

%CC = [1 0]
%xdesired
xd = 1;
%differential
Ts = 1/200;
omega_c = 2*2*pi;



sys =ss(A,B,C,D,Ts);
K = lqr(A,B,Q,R);


