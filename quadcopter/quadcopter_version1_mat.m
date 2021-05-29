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
%%
%%discretization
eig(A)
% stability and controlability analysis
CO = ctrb(A,B);
rank(CO)
% rank(CO) % rank(CO) > length(A)
 %rank(A)
% observability
OO=obsv(A,C);
 rank(OO) % < length(A) ?
 length(A)
% [V,d]=eig(A);
% minimal
%sys=ss(A,B,C,D,-1)
%minreal(sys)




%%
%Q R,...



%Q = diag([1 1 4 10^-4 10^-4 10^-4 20 20 2 10^-1 10^-1 10^-2]);
%R = eye(4,4)*0.1;

%Q&R for pure LQR zonder kalman
%Q = diag([32 32 1500 3 3 4 100 100 1 10^-1 10^-1 10^-2]);%=>1.457
%R = eye(4,4)*0.1;
%Q&R for lqr + lqi zonder kalman
%Q = diag([32 32 150 3 3 4 100 100 1 10^-1 10^-1 10^-2]);
%R = eye(4,4)*0.1;
%for lqg
Q = diag([122 122 150 25 25 1 100 100 1 10^-1 10^-1 10^-2]);
R = eye(4,4)*0.1;
sys_c = ss(A,B,C,zeros(6,4));
sys = c2d(sys_c,Ts,'zoh');

K1 = lqr(sys,Q,R);

N_tot = [sys.A-eye(12) sys.B; sys.C sys.D] \ [zeros(12,6) ; eye(6)];
N_x = N_tot(1:12,:);
N_u = N_tot(13:end,:);

%%
%LQI
%LQI z pos == 10 for pole placement, else: 100
% Increasing integral gains (last 3 params) by too much (>=100)
% creates small instantaneous oscillations on z-axis
%earlier
%Q_i = diag([100 100 100 10 10 10 100 100 100 1 1 1 100 100 100]);
%Q_i = diag([5 5 8 3 3 3 80 80 100 1 1 1 150 150 100]*15);
%R_i = eye(4,4)*0.001;
%new better, that combined with Q = diag([32 32 150 3 3 4 100 100 1 10^-1 10^-1 10^-2]);
%produces a time of 1.707 (for massa). but without kalman
%Q_i = diag([50 50 15 2 2 4 100 100 100 10^-1 10^-1 10^-2 290 290 320]);
%R_i = eye(4,4)*0.001;
%with kalman
Q_i = diag([30 30 15 1 1 1 100 100 100 10^-1 10^-1 10^-2 380 380 350]);
R_i = eye(4,4)*0.001;

Ki = lqi_custom(sys,Q_i,R_i);

%%
% Signal noise covariance
% Or just use Q from LQR? see p. 182 in course notes since B_1 = I
% Qk = I * kalman_var
%Qk = eye(12)*1e-4;
%Qk = diag([1 1 1 1 1 1 1 1 1 1 1 1])*1e-4;
%Qk = diag([0.5 0.5 0.5 10^-4 10^-4 10^-4 10 10 10 10^-1 10^-1 10^-2]); 2.2
%shit symmetry....
Qk = diag([100 100 0.001 200 200 10^-1 55 55 35 10^-1 10^-1 10^-1]*10);
%Q(6,6) = 100;
% Process noise covariance
Rk = diag([2.5e-5 2.5e-5 2.5e-5 7.57e-5 7.57e-5 7.57e-5]);

%%
% Pole placement (LQR controller alternative)

% Initial w_n = 1.8/0.5
% Initial zeta = 0.2279 aka 4.6/(3*w_n) because we wanted t_s = 3

% Peaks in controller and twisted flight paths
% zeta_2 = .... bij taking t_s = 10

% rise time of t_r => calc w_n
t_r = 0.6;
w_n = 1.8/t_r;
% settling time t_s => calc zeta
t_s = 12;
zeta = 4.6/(t_s*w_n);

tfunc = tf(w_n^2, [1 2*zeta*w_n w_n^2]);

dom_poles = pole(tfunc);

other_poles = 0.1*rand(10,1) + ones(10,1)*3*real(dom_poles(1));

p_c = [dom_poles;other_poles];

% Make them discrete
p_d = exp(p_c.*Ts);
% Place the poles (Sylvester)
%K1 = place(sys.A,sys.B,p_d);

%%
% Pole placement (State estimator)

%p_d = eig(sys.A-sys.B*K1); cheating

p_est = p_c*5;
p_est_d = exp(p_est.*Ts);
L = place(sys.A',sys.C',p_est_d).';
sys_L = ss(sys.A-L*sys.C, [sys.B L], eye(12,12), zeros(12,10));



