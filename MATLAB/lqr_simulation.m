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

% Define the non lineair system dyamics with disturbance:
f = quat_dynamics([q0 ;xSymb], uSymb, params);

% Define the system
a = att_lin_system(params);

% Calculate the gain
Kcontroller = lqr_controller(a);

% We construct a function handle out of the symbolic function `f`
fun = matlabFunction(f(2:10), 'Vars', {xSymb , uSymb});

% We define the initial condition
q  = [0.002;0.003;-0.0002];
omega = [0;0;0];
n = [0;0;0];
x = [q;omega;n];

% simulation time
T = 100;

% Initialize cache for the state. We will
% store the sequence of states in a matrix to
% be able to plot them afterwards. Here, we pre-allocate memory.
xCache = zeros(nx, T);  

% Put the initial state in the cache.
xCache(:,1) = x;

% We simulate the system...
for k=1:T
    % Compute the control action using the state
    u = Kcontroller * x;
    
    % Simulate the nonlinear system using ode45 starting from x
    % and for time Ts.
    [~,xi] = ode45(@(t,s) fun(s, u), [0, a.Ts], x);
    x = xi(end,:)';
    
    % Take the system output
    y = a.Cd * x + a.Dd * u;
    
    % Store the states in the cache
    xCache(:,k+1) = x;
end

close;
xCacheTrans = xCache';

subplot(2,2,1)
set(0,'DefaultAxesFontSize',14)
plot(xCacheTrans(:,1:3),'linewidth', 2)
axis tight
xlabel('time');
ylabel('states');
legend('q1', 'q2', 'q3');
grid on;

subplot(2,2,2)
set(0,'DefaultAxesFontSize',14)
plot(xCacheTrans(:,4:6),'linewidth', 2)
axis tight
xlabel('time');
ylabel('states');
legend('omega_x', 'omega_y', 'omega_z');
grid on;

subplot(2,2,3)
xCacheTrans = xCache';
set(0,'DefaultAxesFontSize',14)
plot(xCacheTrans(:,7:9),'linewidth', 2)
axis tight
xlabel('time');
ylabel('states');
legend('nx', 'ny', 'nz');
grid on;