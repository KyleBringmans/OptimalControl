% Define the system dimensions
nx = 9; % number of states
nu = 3; % number of inputs (ux,uy,uz)
ny = 6; % number of outputs (q1,q2,q3,omega_x,omega_y,omega_z)

% Define the symbols xSymb (states) and uSymb (inputs)
xSymb = sym('x', [nx, 1], 'real'); 
uSymb = sym('u', [nu, 1], 'real'); 

% Define the system parameters
params = quat_params;

% Define q0, since we only use unit quaternions 
q0 = real(sqrt(1-xSymb(1)^2-xSymb(2)^2-xSymb(3)^2));

% Define system dyamics
% Use aparms to check robustness
f = quat_dynamics([q0; xSymb], uSymb, params);

% Define the system
a = att_lin_system(params);

% Calculate the gain
Kcontroller = getK_att(a);

% Next we build the Kalman filter:
Lobserver = getL_att(a);

% Let us make sure that A+Lobserver*C is a stable matrix
assert ( all (abs( eig(a.Ad+ Lobserver *a.Cd )) < 1-1e-6) , ...
'A+LC not stable ');

% Reference tracking
% Calculate the G matrix
G = getG_att(a);

% We construct a function handle out of the symbolic function `f`
fun = matlabFunction(f(2:10), 'Vars', {xSymb , uSymb});

% We define the initial condition and initial state estimate
q  = [0;0;0];
omega = [0;0;0];
n = [0;0;0];
x = [q;omega;n];
xEst = x;

% simulation time
T = 1000;

% Initialise caches for the state and its estimates. We will
% store the sequence of states and state estimates in a matrix to
% be able to plot them afterwards. Here, we pre-allocate memory.
xCache = zeros(nx, T); 
rCache = zeros(6,T);
xEstCache = zeros(nx,T); 
rDegCache = zeros(ny, T);
% lambda calculation
lCache = zeros(1,T);

% Put the initial state and the initial state estimate in the cache.
xCache(:,1) = x;
xEstCache(:,1) = xEst;
rDegCache(:,1) = 0;
rCache(:,1) = 0;
r = [0;0;0;0;0;0];
% lambda calculation
lCache(:,1) = log((x(1) - 0.105)/(-0.105));

% ************************************************************************
% We specify DIFFERENT covariances for `v` and `w`
% v represents measurement noise
% (1e-2 on the first three columns, 1e-3 or 1e-1 for the last three)
covarSensorsReal = diag([1e-2 1e-2 1e-2 1e-1 1e-1 1e-1]);
% covarSensorsReal = diag([0.0069^2, 0.007^2, 0.0069^2, 0.002^2, 0.002^2, 0.002^2]);
% w represents noise introduced by actuator
% covarDynamicsReal = 0.0001^2 * eye(nu);
covarDynamicsReal = 1e-4 * eye(3);
% ************************************************************************

% SIMULATION
for k=1:T
    if (k<100)
        r = zeros(6,1);
        rDeg = r;
    elseif (k>=100 && k<600)
        r = [0.105;0;0;0;0;0];
        rDeg(1) = 12.018;           % x
        rDeg(2) = 0;                % y
        rDeg(3) = 0;                % z
    end
    rDegCache(:,k+1) = rDeg;
    rCache(:,k+1) = r;
    
    % Noises
    w = mvnrnd(zeros(nu,1), params.covarDynamics_att)';
    v = mvnrnd(zeros(ny,1), params.covarSensors_att)';
    
    % Compute the control action using the estimated state + ref tracking
    % Calculate the equilibrium x and u
    xue = G*r;
    x_e = xue(1:nx);
    u_e = xue(nx + 1 : nu + nx);
    
    % compute the control action
    % First three elements of x are quaternions, so no normal substraction
    % Construct q0_e, in order to use quatmultiply
    q0_e = real(sqrt(1 - x_e(1)^2 - x_e(2)^2 - x_e(3)^2));
    % Invert q_e
    q_e_inv = quatinv([q0_e,x_e(1),x_e(2),x_e(3)]);
    % Calculate q0
    q0_xEst = real(sqrt(1 - xEst(1)^2 - xEst(2)^2 - xEst(3)^2));
    % Take the difference from the quaternion part of x and x_e
    sub_qEst_q_e = quatmultiply([q0_xEst,xEst(1),xEst(2),xEst(3)],q_e_inv);
    % Construct the complete substraction vector (normal substraction for
    % other state variables)
    sub_xEst_x_e = [(sub_qEst_q_e(2:4).') ; xEst(4:9)-x_e(4:9)];
    % Finally calculate the control action
    u = u_e + Kcontroller * sub_xEst_x_e;
    
    % Clipping
    u = clip_control_signal(u,0.5);
    
    % Simulate the nonlinear system using ode45 starting from x
    % and for time Ts. The actuation that is applied to the system
    % is u + w (perturbed by random noise)
    [~,xi] = ode45(@(t,s) fun(s, u + w), [0, a.Ts], x);
    x = xi(end,:)';
    
    % Let's normalize the states just to be sure
    q0 = real(sqrt(1 - x(1)^2 - x(2)^2 - x(3)^2));
    normalized = quatnormalize([q0 x(1) x(2) x(3)]);
    x(1:3) = normalized(2:4);
    
    % Take the system output
    y = a.Cd * x + a.Dd * u + v;
    
    % Update the state estimates
    xEst = a.Ad * xEst + Lobserver * (a.Cd * xEst - y) + a.Bd * u;
    
    % Store the states in the cache
    xEstq0 = real(sqrt(1-xEst(1)^2-xEst(2)^2-xEst(3)^2));
    %xEstAngles = quat2deg(xEstq0, xEst(1), xEst(2), xEst(3));
    %xEst(1:3) = xEstAngles;
    xEstCache(:,k+1) = xEst;
    xCache(:,k+1) = x;
    % lambda calculation
    lCache(:,k+1) = log((x(1) - 0.105)/(-0.105));
    
    % Convert the quaternions to degrees
    % angles = quat2deg(q0, x(1), x(2), x(3));
    % xCache(1:3,k+1) = angles;
    % xEstCache(1:3,k+1) = xEstAngles;
end

% ************************************************************************
% PLOTS
close;
xCacheTrans = xCache';
xEstCacheTrans = xEstCache';
rCacheTrans = rCache';

x_plot = linspace(0,(T+1)/476,T+1);

lbeg = 100;
lend = 280;

subplot(3,1,1)
set(0,'DefaultAxesFontSize',14)
plot(x_plot, xCacheTrans(:,1:3),'linewidth', 2)
hold on
plot(x_plot, xEstCacheTrans(:,1:3),'--', 'linewidth', 1.5)
hold on
plot(x_plot,rCacheTrans(:,1:3), ':', 'linewidth',2)
hold on
line([lbeg/476 lbeg/476],[-.1 .1], 'linewidth', 2, 'Color', 'red');
hold on
line([lend/476 lend/476],[-.1 .1], 'linewidth', 2, 'Color', 'red');
axis tight
xlabel('time [s]');
ylabel('Angles [°]');
lgd = legend('roll', 'pitch', 'yaw', 'est_r_o_l_l' ,'est_p_i_t_c_h',...
    'est_y_a_w','ref_r_o_l_l','ref_p_i_t_c_h','ref_y_a_w');
lgd.NumColumns = 3;
lgd.Location = 'eastoutside';
grid on;

subplot(3,1,2)
set(0,'DefaultAxesFontSize',14)
plot(x_plot, xCacheTrans(:,4:6),'linewidth', 2)
hold on
plot(x_plot, xEstCacheTrans(:,4:6),'--','linewidth', 1.5)
axis tight
xlabel('time [s]');
ylabel('Angular velocities');
lgd = legend('\omega_x', '\omega_y', '\omega_z', 'est_\omega_x', ...
    'est_\omega_y','est_\omega_z');
lgd.NumColumns = 2;
lgd.Location = 'eastoutside';
grid on;

subplot(3,1,3)
set(0,'DefaultAxesFontSize',14)
plot(x_plot, xCacheTrans(:,7:9),'linewidth', 2)
hold on 
plot(x_plot, xEstCacheTrans(:,7:9),'--', 'linewidth',1.5)
axis tight
xlabel('time [s]');
ylabel('Angular thrusts');
lgd = legend('n_x', 'n_y', 'n_z', 'est_n_x', 'est_n_y','est_n_z');
lgd.NumColumns = 2;
lgd.Location = 'eastoutside';
grid on;

% Calculation lambda

lCacheTrans = lCache';
fi = fit(x_plot(lbeg:lend)', lCacheTrans(lbeg:lend),'poly1');
g = fi(x_plot(lbeg:lend));
coefficients = polyfit(x_plot(lbeg:lend)',g , 1);
slope = coefficients(1);
lambda = -slope;

figure(3);
plot(x_plot(lbeg:lend), lCacheTrans(lbeg:lend), '.')
hold on
plot(x_plot(lbeg:lend),fi(x_plot(lbeg:lend)), 'linewidth', 2);

