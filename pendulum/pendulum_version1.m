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
Q = [0.25 0 0 0;0 4 0 0;0 0 0 0;0 0 0 0];
R = 0.003;
%stability and controlability analysis
eig(A)
CO = ctrb(A,B)
rank(CO) % rank(CO) > length(A)
rank(A)

OO=obsv(A,C);
rank(OO) % < length(A) ?
[V,d]=eig(A);
C*V


sys =ss(A,B,C,D,Ts);
K = lqr(A,B,Q,R);


