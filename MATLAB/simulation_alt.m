% Define the system dimensions
nx = 3; % number of states
nu = 1; % number of inputs (ut)
ny = 1; % number of outputs (z)

% Define the symbols xSymb (states) and uSymb (inputs)
xSymb = sym('x', [nx, 1], 'real'); 
uSymb = sym('u', [nu, 1], 'real'); 

% Define the system parameters
params = quat_params;
actualParams = actual_params;

% Define system dyamics
% f = alt_lin_dyn(xSymb, uSymb, params);
% We assume that the drone is stable (attitude states = 0)
f = alt_dynamics([1;zeros(9,1);xSymb], [zeros(3,1); uSymb], quat_params);

% Define the system
al = alt_lin_system(params);

% Calculate the gain
[Kx, Kz] = getK_alt(al);

% Next we get the Kalman filter:
L = getL_alt(al);

% Reference tracking
% Calculate the G matrix
G = getG_alt(al);

% We construct a function handle out of the symbolic function `f`
fun = matlabFunction(f(11:13), 'Vars', {xSymb , uSymb});

% We define the initial condition and initial state estimate
nt = 0;
z = 0.8;
vz = 0;
x = [nt;z;vz];
z_int = 0;
xEst = x;
u = 0;
y = 0;

%medfilter
m = 7;
meas_data = zeros(1,m);

% simulation time
T = 150;

% Initialise caches for the state and its estimates. We will
% store the sequence of states and state estimates in a matrix to
% be able to plot them afterwards. Here, we pre-allocate memory.
xCache = zeros(nx, T); 
xEstCache = zeros(nx,T); 
rCache = zeros(ny, T);
%yCache = zeros(ny,T);

% Put the initial state and the initial state estimate in the cache.
xCache(:,1) = x;
xEstCache(:,1) = xEst;
rCache(:,1) = 0.8;
yCache(:,1) = z;
r = 0.8;

% We specify !!DIFFERENT!! covariances for `v` and `w`
% v represents measurement noise
% (1e-2 on the first three columns, 1e-1 for the last three)
% covarSensorsReal = 1e-2;
% w represents noise introduced by actuator
% covarDynamicsReal = 1e-5;

% covarDynamicsReal = params.covarDynamics_alt;
% covarSensorsReal = params.covarSensors_alt;

% We simulate the system...
for k=1:T
    if (k<50)
        r = 0.8;
    else
        r = 1;
    end
   
    % Noises
    w = mvnrnd(zeros(nu,1), params.covarDynamics_alt)';
    v = mvnrnd(zeros(ny,1), params.covarSensors_alt)';
    
    % 1) update state estimate
    % Take the system output
    xEst = al.Ad * xEst + L * (al.Cd * xEst + al.Dd * u - y) + al.Bd * u;
    
    
    % 2) update control signal
    % a) update integral windup
    z_int = z_int + r - y;
    
    % b) update control signal
    % Compute the control action using the estimated state + ref tracking
    % Calculate the equilibrium x and u
    xue = G*r;
    x_e = xue(1:nx);
    u_e = xue(nx + 1 : nu + nx);
    
    u = u_e + Kx * (xEst - x_e) + Kz*z_int;
    
    % 2) clip control signal:
    u = clip_control_signal(u);
    
    % Simulate the nonlinear system using ode45 starting from x
    % and for time Ts. The actuation that is applied to the system
    % is u + w (perturbed by random noise)
    [~,xi] = ode45(@(t,s) fun(s, u + w), [0, al.Ts], x);
    x = xi(end,:)';    
    
    y = al.Cd * x + al.Dd * u + v;
    % medfilter
    % 1) store the data
    yCache(:,k+1) = y;
    % 2) compute median
    y = median_filter(yCache,m);
    
    % Store the states in the cache
    rCache(:,k+1) = r;
    xEstCache(:,k+1) = xEst;
    xCache(:,k+1) = x;
    
end

% median filter for y
%yCache = medfilt1(yCache, 7);

close;
xCacheTrans = xCache';
xEstCacheTrans = xEstCache';
rCacheTrans = rCache';
yCacheTrans = yCache';

x_plot = linspace(0,(T+1)/476,T+1);

subplot(1,2,1)
plot(x_plot,xCacheTrans(:,1),'linewidth', 3)
hold on
plot(x_plot,xEstCacheTrans(:,1),'--', 'linewidth', 2)
hold on
xlabel('time [s]');
ylabel('Thrust');
legend('n_t', 'est_1');
grid on;

subplot(1,2,2)
plot(x_plot,xCacheTrans(:,2),'linewidth', 3)
hold on
plot(x_plot,xCacheTrans(:,3),'linewidth', 1.5)
hold on
plot(x_plot,xEstCacheTrans(:,2),'--', 'linewidth', 2)
hold on
plot(x_plot,xEstCacheTrans(:,3),'--', 'linewidth', 1.5)
hold on
plot(x_plot,rCacheTrans, ':', 'linewidth',2)
hold on
% plot(x_plot,yCacheTrans, 'linewidth',.5)
axis tight
xlabel('time [s]');
ylabel('States');
legend('z','v_z','est_2','est_3','ref');
grid on;










