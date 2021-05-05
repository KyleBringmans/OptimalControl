% simulation of attitude and altitude simultaniously

% initialisation
thrust = 0.5;
plant = plant();
controller = init_controller(plant);

% loop, om de 23.8 ook altitude

% Define the system dimensions
nx = 12; % number of states
nu = 4; % number of inputs
ny = 7; % number of outputs

% Attitude
nxAtt = 9; % q1, q2, q3, w1, w2, w3, nx, ny, nz
nuAtt = 3; % ux, uy, uz
nyAtt = 6; % q1, q2, q3, w1, w2, w3
% Altitude
nxAlt = 3; % nt, z, vz
nuAlt = 1; % ut
nyAlt = 1; % z

% Define the symbols xSymb (states) and uSymb (inputs)
xSymb = sym('x', [nx, 1], 'real'); 
uSymb = sym('u', [nu, 1], 'real'); 

% Define the system parameters
params = quat_params;
actualParams = actual_params;

% Define system dyamics
% f = alt_lin_dyn(xSymb, uSymb, params);
% We assume that the drone is stable (attitude states = 0)
q0 = real(sqrt(1-xSymb(1)^2-xSymb(2)^2-xSymb(3)^2));
f = alt_dynamics([q0;xSymb], uSymb, actualParams);

% We construct a function handle out of the symbolic function `f`
fun = matlabFunction(f(2:13), 'Vars', {xSymb , uSymb});

% We define the initial condition and initial state estimate
q  = [0;0;0];
omega = [0;0;0];
n = [0;0;0];
nt = 0;
z = 0.8;
vz = 0;
x = [q;omega;n;nt;z;vz];

z_int = 0;
xEst = x;
u = zeros(nu,1);
y = zeros(ny,1);

% simulation time
T = 4000;

% Initialise caches for the state and its estimates. We will
% store the sequence of states and state estimates in a matrix to
% be able to plot them afterwards. Here, we pre-allocate memory.
xCache = zeros(nx, T); 
xEstCache = zeros(nx,T); 
rCache = zeros(ny, T);
yCache = zeros(ny,T);
zCache = zeros(1,T);

% Put the initial state and the initial state estimate in the cache.
xCache(:,1) = x;
xEstCache(:,1) = xEst;
yCache(:,1) = y;
refAtt = [0;0;0;0;0;0];
refAlt = 0.8;
r = [refAtt; refAlt];
rCache(:,1) = r;
zCache(:,1) = 0;

% ************************************************************************
% We specify !!DIFFERENT!! covariances for `v` and `w` to test robustness
covarSensorsReal_att = diag([0.0005 0.0005 0.0005 0.00007 0.00007 0.00007]);
covarDynamicsReal_att = 1e-3 * eye(3);
covarSensorsReal_alt = 1e-2*2;
covarDynamicsReal_alt = 1e-2/2;
% ************************************************************************

% SIMULATION
for k=1:T
    
    % Setting references
    
    % REFERENCE 1: block every quaternion, then rise 20 cm
    
    
    if (k>=100 && k<500)
        refAtt = [0.1;-0.07;0.05;0;0;0];
        refAlt = 0.8;
        r = [refAtt; refAlt];
    elseif (k>=500 && k<900)
        refAtt = [0;0;0;0;0;0];
        refAlt = 0.8;
        r = [refAtt; refAlt];
    elseif (k>=900)
        refAtt = [0;0;0;0;0;0];
        refAlt = 1;
        r = [refAtt; refAlt];
    end
    
    % REFERENCE 2: wobble roll
    
    %{
    if (k==120)
        refAtt = [0.3;0;0;0;0;0];
        r = [refAtt; refAlt];
    end
    if (rem(k,120) == 0)
        refAtt = refAtt * -1;
        r = [refAtt; refAlt];
    end
    %}
    
    %Noises
    wAtt = mvnrnd(zeros(nuAtt,1), covarDynamicsReal_att)';
    vAtt = mvnrnd(zeros(nyAtt,1), covarSensorsReal_att)';
    
    wAlt = mvnrnd(zeros(nuAlt,1), covarDynamicsReal_alt)';
    vAlt = mvnrnd(zeros(nyAlt,1), covarSensorsReal_alt)';
    
    w = [wAtt;wAlt];
    v = [vAtt;vAlt];
    
    [xEst(1:9), u(1:3)] = controller_attitude(y(1:6),refAtt,thrust,...
        xEst(1:9),u(1:3),plant,controller);
    
    if (rem(k,24) == 0)
        [xEst(10:12),u(4),z_int] = controller_altitude(y(7),refAlt,thrust,...
            xEst(10:12),u(4),plant, z_int,controller);
        
    end
    
    %Simulate the nonlinear system using ode45 starting from x
    %and for time Ts. The actuation that is applied to the system
    %is u + w (perturbed by random noise)
    [~,xi] = ode45(@(t,s) fun(s, u + w), [0, plant.attitude.Ts], x);
    x = xi(end,:)';
    
    y(1:6) = plant.attitude.Cd * x(1:9) + plant.attitude.Dd * u(1:3) + v(1:6);
    y(7) = plant.altitude.Cd * x(10:12) + plant.altitude.Dd * u(4) + v(7);
    
    %Store the states in the cache
    rCache(:,k+1) = r;
    xEstCache(:,k+1) = xEst;
    xCache(:,k+1) = x;
    yCache(:,k+1) = y;
    zCache(:,k+1) = z_int;
end

% median filter for y
yCache(7,:) = medfilt1(yCache(7,:), 7);

% ************************************************************************
% PLOTS

close;
xCacheTrans = xCache';
xEstCacheTrans = xEstCache';
rCacheTrans = rCache';
yCacheTrans = yCache';
zCacheTrans = zCache';

x_plot = linspace(0,(T+1)/476,T+1);

figure(1)
subplot(3,1,1)
set(0,'DefaultAxesFontSize',14)
plot(x_plot, xCacheTrans(:,1:3),'linewidth', 2)
hold on
plot(x_plot, xEstCacheTrans(:,1:3),'--', 'linewidth', 1.5)
hold on
plot(x_plot,rCacheTrans(:,1:3), ':', 'linewidth',2)
axis tight
xlabel('time [s]');
ylabel('Quaternions');
lgd = legend('roll', 'pitch', 'yaw', 'est_r_o_l_l' ,'est_p_i_t_c_h',...
    'est_y_a_w','ref_r_o_l_l','ref_p_i_t_c_h','ref_y_a_w');
lgd.NumColumns = 2;
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

figure(2)
subplot(1,2,1)
plot(x_plot,xCacheTrans(:,10),'linewidth', 3)
hold on
plot(x_plot,xEstCacheTrans(:,10),'--', 'linewidth', 2)
hold on
xlabel('time [s]');
ylabel('Thrust');
lgd = legend('n_t', 'est_n_t');
lgd.Location = 'eastoutside';
grid on;

subplot(1,2,2)
plot(x_plot,xCacheTrans(:,11),'linewidth', 3)
hold on
plot(x_plot,xCacheTrans(:,12),'linewidth', 1.5)
hold on
plot(x_plot,xEstCacheTrans(:,11),'--', 'linewidth', 2)
hold on
plot(x_plot,xEstCacheTrans(:,12),'--', 'linewidth', 1.5)
hold on
plot(x_plot,rCacheTrans(:,7), ':', 'linewidth',2)
hold on
plot(x_plot,yCacheTrans(:,7), 'linewidth',.5)
axis tight
xlabel('time [s]');
ylabel('States');
lgd = legend('z','v_z','est_z','est_v_z','ref','y');
lgd.Location = 'eastoutside';
grid on;

figure(3)
subplot(3,1,1)
set(0,'DefaultAxesFontSize',14)
plot(x_plot, xCacheTrans(:,1:3),'linewidth', 2)
hold on
plot(x_plot,rCacheTrans(:,1:3), ':', 'linewidth',2)
axis tight
xlabel('time [s]');
ylabel('Quaternions');
lgd = legend('roll', 'pitch', 'yaw');
lgd.Location = 'eastoutside';
grid on;

subplot(3,1,2)
plot(x_plot,xCacheTrans(:,11),'linewidth', 3)
hold on
plot(x_plot,xCacheTrans(:,12),'linewidth', 1.5)
hold on
plot(x_plot,rCacheTrans(:,7), ':', 'linewidth',2)
axis tight
xlabel('time [s]');
ylabel('States');
lgd = legend('z','v_z','ref');
lgd.Location = 'eastoutside';
grid on;

subplot(3,1,3)
plot(x_plot,zCacheTrans, 'linewidth',2)
axis tight
xlabel('time [s]');
ylabel('Integrator');
lgd = legend('z_i_n_t');
lgd.Location = 'eastoutside';
grid on;


