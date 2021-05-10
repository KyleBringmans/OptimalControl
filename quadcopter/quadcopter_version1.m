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
