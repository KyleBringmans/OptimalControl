% Navigation simulation.
% The outputs are the x and y coordinates (similar to altitude).

% TO DO: given target (xT,yT) and current (x,y) --> get qref needed to
% get to target.

% Define the system dimensions
nx = 6; % number of states (q1,q2,x,y,vx,vy)
nu = 2; % number of inputs (q1r,q2r)?
ny = 2; % number of outputs (x,y)

% Define the symbols xSymb (states) and uSymb (inputs)
xSymb = sym('x', [nx, 1], 'real'); 
uSymb = sym('u', [nu, 1], 'real'); 

% Define the system parameters
params = quat_params;

% Define system LINEAR dyamics
f = nav_linear_dyn(xSymb,uSymb, params);

% Define the system
na = nav_lin_system(params);

% We construct a function handle out of the symbolic function `f`
fun = matlabFunction(f, 'Vars', {xSymb , uSymb});

% Gain matrix
[Kx, Kz] = getK_nav(na);

% G matrix
G = getG_nav(na);

% We define the initial condition
q1 = 0;
q2 = 0;
xcoord = 1;
ycoord = -2;
vx = 0;
vy = 0;
x = [q1;q2;xcoord;ycoord;vx;vy];

z_int = zeros(ny,1);
y = zeros(ny,1);
u = zeros(nu,1);
r = [0;0];

% simulation time
T = 100;

% Initialise caches for the state and its estimates. We will
% store the sequence of states and state estimates in a matrix to
% be able to plot them afterwards. Here, we pre-allocate memory.
xCache = zeros(nx, T); 
uCache = zeros(nu, T);

% Put the initial state and the initial state estimate in the cache.
xCache(:,1) = x;
uCache(:,1) = u;

% We simulate the system...
for k=1:T
    
    % Update integral windup
    z_int = z_int + r - y;
    
    % Compute the control action using the state + ref tracking
    % Calculate the equilibrium x and u
    xue = G*r;
    x_e = xue(1:nx);
    u_e = xue(nx + 1 : nu + nx);
    
    % Control action
    u = u_e + Kx * (x - x_e) + Kz * z_int;
    
    % Simulate the linear system using ode45 starting from x
    % and for time Ts.
    [~,xi] = ode45(@(t,s) fun(s, u), [0, na.Ts], x);
    x = xi(end,:)';
    
    % Take the system output
    y = na.Cd * x + na.Dd * u;
    
    % Store the states in the cache
    xCache(:,k+1) = x;
    uCache(:,k+1) = u;
end

close;
xCacheTrans = xCache';
uCacheTrans = uCache';

x_plot = linspace(0,(T+1)*na.Ts,T+1);

subplot(3,1,1)
set(0,'DefaultAxesFontSize',14)
plot(x_plot, xCacheTrans(:,1:2),'linewidth', 2)
axis tight
xlabel('time [s]');
ylabel('Quaternions');
lgd = legend('q1', 'q2');
lgd.Location = 'eastoutside';
grid on;

subplot(3,1,2)
set(0,'DefaultAxesFontSize',14)
plot(x_plot, xCacheTrans(:,3:4),'linewidth', 2)
axis tight
xlabel('time [s]');
ylabel('Locations');
lgd = legend('x', 'y');
lgd.Location = 'eastoutside';
grid on;

subplot(3,1,3)
set(0,'DefaultAxesFontSize',14)
plot(x_plot, xCacheTrans(:,5:6),'linewidth', 2)
axis tight
xlabel('time [s]');
ylabel('Velocities');
lgd = legend('v_x', 'v_y');
lgd.Location = 'eastoutside';
grid on;