% the system with reference tracking and integral action for a constant
% disturbance

% Define the system dimensions
nx = 9; % number of states
nu = 3; % number of inputs
ny = 6; % number of outputs

% Define the symbols xSymb (states) and uSymb (inputs)
xSymb = sym('x', [nx, 1], 'real'); 
uSymb = sym('u', [nu, 1], 'real'); 

% Define the system parameters
params = quat_params;
q0 = real(sqrt(1-xSymb(1)^2-xSymb(2)^2-xSymb(3)^2));

% Define the linearized plant
P = att_lin_system(params);

% Define a constant disturbance:
dist = 0.2;
d = [0;dist;0;0;    0;0;0;    0;0;0];

% Define the non lineair system dyamics with disturbance:
f = quat_dynamics([q0 ;xSymb]+d, uSymb, params);

% We construct a function handle out of the symbolic function `f`
fun = matlabFunction(f(2:10), 'Vars', {xSymb , uSymb});

%///////////////// controller calculations /////////////////%

% Q (nx+ny x nx+ny) and R matrices
Q = diag([1e4 1e4 1e4 1e-4 1e-4 1e-4 1e-5 1e-5 1e-5 1e-5 1e-5 1e-5 1e-4 1e-4 1e-4]);
R = eye(3)*1e3;

% Define new C matrix for integral action
C = [eye(3) zeros(3,6); zeros(3,9)];

% calculate the G matrix for reference tracking
W = [P.Ad-eye(nx), P.Bd
     P.C,         P.D];
 
G = W\[zeros(nx, ny); eye(ny)];

% Calculate the gain martix Kz and Kx
System = ss(P.Ad, P.Bd, P.C, P.D, P.Ts);
Kcontroller = -lqi(System,Q,R);
Kx = Kcontroller(:,1:9);
Kz = Kcontroller(:,10:15);

% is the gain stable?
assert( all(abs( eig(P.Ad + P.Bd* Kcontroller(1:nu,1:nx) )) < 1-1e-6) ,'A+BK not stable ' );

% We define the initial condition
q  = [0.5;0.3;0.7];
omega = [0;0;0];
n = [0;0;0];
x = [q;omega;n];
z = zeros(6,1);

% simulation time
T = 700;

% Initialize cache for the state. We will
% store the sequence of states in a matrix to
% be able to plot them afterwards. Here, we pre-allocate memory.
xCache = zeros(nx, T);  
rCache = zeros(ny, T);
dCache = zeros(1,T);

% Put the initial state in the cache.
xCache(:,1) = x;
rCache(:,1) = zeros(6,1);

for k = 1:T
    % reference function:
    if (k<200)
        r = zeros(6,1);
    elseif (k>=200 && k<400)
        r = [0.5;0.25;0;0;0;0];
    else
        r = zeros(6,1);
    end
    rCache(:,k+1) = r;
    
    %reference tracking
    xue = G * r;
    x_e = xue(1:nx);
    u_e = xue(nx + 1 : nu + nx);
    
    % compute the control action
    % First three elements of x are quaternions, so no normal substraction
    % Construct q0_e, in order to use quatmultiply
    q0_e = real(sqrt(1 - x_e(1)^2 - x_e(2)^2 - x_e(3)^2));
    % Invert q_e
    x_e_T = transpose(x_e);
    q_e_inv = quatinv([q0_e , x_e_T(1:3)]);
    % Calculate q0
    q0_x = real(sqrt(1 - x(1)^2 - x(2)^2 - x(3)^2));
    % Take the difference from the quaternion part of x and x_e
    sub_q_q_e = quatmultiply([q0_x,x(1),x(2),x(3)],q_e_inv);
    % Construct the complete substraction vector (normal substraction for
    % other state variables)
    sub_q_q_e_T = transpose(sub_q_q_e);
    sub_x_x_e = [sub_q_q_e_T ; x(4:9)-x_e(4:9)];
    
    u = u_e + Kx*sub_x_x_e(2:10) + Kz*z;
    
    % Simulate the nonlinear system using ode45 starting from x
    % and for time Ts.
    [~,xi] = ode45(@(t,s) fun(s, u), [0, P.Ts], x);
    x = xi(end,:)';
    
    % Take the system output
    y = P.C * x + P.D * u;
    
    % Integral dynamics
    z = z + r - y;
    
    % Store the states in the cache
    xCache(:,k+1) = x;
end

close;
xCacheTrans = xCache';
rCacheTrans = rCache';

subplot(2,2,1)
ylim([-1,1])
set(0,'DefaultAxesFontSize',14)
plot(rCacheTrans(:,1:3),'linewidth', 0.5)
hold on
plot(xCacheTrans(:,1:3),'linewidth', 2)
axis tight
xlabel('time');
ylabel('states');
legend('ref1','ref2','ref3','q1', 'q2', 'q3');
grid on;

subplot(2,2,2)
set(0,'DefaultAxesFontSize',14)
plot(xCacheTrans(:,4:6),'linewidth', 2)
axis tight
xlabel('time');
ylabel('states');
legend('\omega_x', '\omega_y', '\omega_z');
grid on;

subplot(2,2,3)
set(0,'DefaultAxesFontSize',14)
plot(xCacheTrans(:,7:9),'linewidth', 2)
axis tight
xlabel('time');
ylabel('states');
legend('nx', 'ny', 'nz');
grid on;