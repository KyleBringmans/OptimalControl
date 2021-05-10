
clear all
load 'references_21'
%massa quadcopter
mass = 0.5
%radius quadcopter
L = 0.25
%Propeller lift coefficient 
k = 3*10^(-6)
%Propeller drag coefficient
b = 10^(-7)
%Acceleration of gravity
g = 9.81
%Air friction coefficient
Kd = 0.25
%Quadcopter inertia about the x-axis
Ixx = 5*10^(-3)
%Quadcopter inertia about the y-axis
Iyy = 5*10^(-3)
%Quadcopter inertia about the z-axis
Izz = 1*10^(-2)
%Motor constant
cm = 10^4
%sum of all u_i
UU = 163.5;

A = zeros(12);
A(1,4) = 1;A(2,5) = 1;A(3,6) = 1;A(4,4) = -Kd/mass;A(4,8) =k*cm/mass*UU ;
A(5,5) = -Kd/mass;A(5,9) =k*cm/mass*UU ;A(6,6) = -Kd/mass;
A(7,10) = 1;A(8,11) = 1;A(9,12) = 1;%A(10,11) = (Iyy-Izz)/Ixx;A(10,12) = (Iyy-Izz)/Ixx;
%A(11,10) = (Izz-Ixx)/Iyy;A(11,12) = (Izz-Ixx)/Iyy;A(12,10) = (Ixx-Iyy)/Izz;
%A(12,11) = (Ixx-Iyy)/Izz
%comment because the omega are zero, see latex.

B = zeros(12,4);
B(6,:) = k*cm/mass;
B(10,1) = L*k*cm/Ixx;B(10,3) = -L*k*cm/Ixx;
B(11,2) = L*k*cm/Iyy;B(11,4) = -L*k*cm/Iyy;
B(12,1) = b*cm/Izz;B(12,2) = -b*cm/Izz;B(12,3) = b*cm/Izz;B(12,4) = -b*cm/Izz;

C = zeros(6,12);
C(1,1)=1;C(2,2)=1;C(3,3)=1;C(4,7)=1;C(5,8)=1;C(6,9)=1;
D = zeros(6,4);
CC = eye(6);

Q = zeros(12,12);
Q(1,1) = 0.1;Q(2,2) = 0.1;Q(3,3) = 1000;
Q(4,4) = 10^(-4);Q(5,5) = 10^(-4);Q(6,6) = 10^(-1);
Q(7,7) = 10;Q(8,8) = 10;Q(9,9) = 10^(-6);
Q(10,10) = 10^(-1);Q(11,11) = 10^(-1);Q(12,12) = 10^(-2);
%Q(
%Q = eye(12,12);
%Q(1,1) = 10^(-4);Q(2,2) = 10^(-4);Q(3,3) = 10;
%Q(7,7) = 10;Q(8,8) = 10;Q(9,9) = 10^(-6);
R = eye(4,4)*0.1;
sys = ss(A,B,C,zeros(6,4))

K1 = lqr(sys,Q,R);
K1 = lqr(A,B,Q,R);
Q_i = eye(18,18);
%Q(1,1) = 1;Q(2,2) = 1;Q(3,3) = 100;
R_i = eye(4,4)*0.1;
%integral_act = lqi(sys,Q_i,R_i)

K = zeros(4,6)
K(:,1:3) = K1(:,1:3)
K(:,4:6) = K1(:,7:9)
NA = [zeros(6,6) C; zeros(12,6) A]
NB = [D;B]
lqi(sys,Q_i,R_i)






