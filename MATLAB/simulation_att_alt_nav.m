% simulation of attitude, altitude and navigation simultaniously

% initialisation
thrust = 0.5;
plant = plant();
controller = init_controller(plant);

% loop, om de 23.8 ook altitude

% Define the system dimensions
nx = 16; % number of states 
nu = 6; % number of inputs 
ny = 9; % number of outputs 
% Attitude
nxAtt = 9; % q1, q2, q3, w1, w2, w3, nx, ny, nz
nuAtt = 3; % ux, uy, uz
nyAtt = 6; % q1, q2, q3, w1, w2, w3
% Altitude
nxAlt = 3; % nt, z, vz
nuAlt = 1; % ut
nyAlt = 1; % z
% Navigation
nxNav = 6; % q1, q2, x, y, vx, vy
nuNav = 2; % q1r, q2r
nyNav = 2; % x, y

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
f = nav_dynamics([q0;xSymb], uSymb, params);

% We construct a function handle out of the symbolic function `f`

% Function for Att and  Alt
fun = matlabFunction(f(2:17), 'Vars', {xSymb, uSymb});

% We define the initial condition and initial state estimate
q  = [0;0;0];
omega = [0;0;0];
n = [0;0;0];
nt = 0;
z = 0.8;
vz = 0;
xcoord = 0;
ycoord = 0;
vx = 0;
vy = 0;
x = [q;omega;n;nt;z;vz;xcoord;ycoord;vx;vy];

z_int_alt = 0;
z_int_nav = zeros(2,1);
% for now, navigation uses no estimates
xEst = x;
u = zeros(nu,1);
y = zeros(ny,1);

% simulation time
T = 15000;

% Initialise caches for the state and its estimates. We will
% store the sequence of states and state estimates in a matrix to
% be able to plot them afterwards. Here, we pre-allocate memory.
xCache = zeros(nx, T); 
xEstCache = zeros(nx,T); 
rCache = zeros(ny, T);
yCache = zeros(ny,T);

% Put the initial state and the initial state estimate in the cache.
xCache(:,1) = x;
xEstCache(:,1) = xEst;
yCache(:,1) = y;
refAtt = [0;0;0;0;0;0];
refAlt = 0.8;
refNav = [0;0];
r = [refAtt; refAlt; refNav];
rCache(:,1) = r;

% ************************************************************************
% We specify !!DIFFERENT!! covariances for `v` and `w` to test robustness
covarSensorsReal_att = diag([0.0005 0.0005 0.0005 0.00007 0.00007 0.00007]);
covarDynamicsReal_att = 1e-3 * eye(3);
covarSensorsReal_alt = 1e-2*2;
covarDynamicsReal_alt = 1e-2/2;
covarSensorsReal_nav = 1e-2 * eye(2);
covarDynamicsReal_nav = 1e-4 * eye(2);
% ************************************************************************

% SIMULATION
for k=1:T
    
    % Setting references
    %{
    % PATH 1: from (0,0) to (x,y)
    if (k==500)
        refNav = [0;0];
    end
    refAtt = [u(5);u(6);0;0;0;0];
    r = [refAtt;refAlt;refNav];
    %}
    % PATH 2: from (0,0) to (x1,y2), to (x2,y2) and back
    if (k==1000)
        refNav = [1;-2];
    end
    if (k==6000)
        refNav = [1.5;3];
    end
    if (k==11000)
        refNav = [0;0];
    end
    refAtt = [u(5);u(6);0;0;0;0];
    r = [refAtt;refAlt;refNav];
    
    %Noises
    
    wAtt = mvnrnd(zeros(nuAtt,1), covarDynamicsReal_att)';
    vAtt = mvnrnd(zeros(nyAtt,1), covarSensorsReal_att)';
    
    wAlt = mvnrnd(zeros(nuAlt,1), covarDynamicsReal_alt)';
    vAlt = mvnrnd(zeros(nyAlt,1), covarSensorsReal_alt)';
    
    wNav = mvnrnd(zeros(nuNav,1), covarDynamicsReal_nav)';
    vNav = mvnrnd(zeros(nyNav,1), covarDynamicsReal_nav)';
    
    w = [wAtt;wAlt;wNav];
    v = [vAtt;vAlt;vNav];
    
    [xEst(1:9), u(1:3)] = controller_attitude(y(1:6),refAtt,thrust,...
        xEst(1:9),u(1:3),plant,controller);
    
    % Update altitude (every 24 cycles)
    if (rem(k,24) == 0)
        [xEst(10:12),u(4), z_int_alt] = controller_altitude(y(7),refAlt,thrust,...
            xEst(10:12),u(4),plant, z_int_alt,controller);
    end
    
    % Update navigation (every 32 cycles)
    if (rem(k,32) == 0)
        [xEst([1:2, 13:16]), u(5:6), z_int_nav] = controller_navigation(y(8:9),refNav,...
            xEst([1:2, 13:16]), u(5:6), plant, z_int_nav,controller);
    end
    
    %Simulate the nonlinear system using ode45 starting from x
    %and for time Ts. The actuation that is applied to the system
    %is u + w (perturbed by random noise)
    [~,xi] = ode45(@(t,s) fun(s, u + w), [0, plant.attitude.Ts], x);
    x = xi(end,:)';
    
    % Calculation outputs
    y(1:6) = plant.attitude.Cd * x(1:9) + plant.attitude.Dd * u(1:3) + v(1:6);
    y(7) = plant.altitude.Cd * x(10:12) + plant.altitude.Dd * u(4) + v(7);
    y(8:9) = plant.navigation.Cd * x([1:2, 13:16]) ... 
        + plant.navigation.Dd * u(5:6) + v(8:9);
    
    %Store the states in the cache
    rCache(:,k+1) = r;
    xEstCache(:,k+1) = xEst;
    xCache(:,k+1) = x;
    yCache(:,k+1) = y;
end

% median filter for y
yCache(7,:) = medfilt1(yCache(7,:), 7);

% ************************************************************************

%PLOTS

close;
xCacheTrans = xCache';
xEstCacheTrans = xEstCache';
rCacheTrans = rCache';
yCacheTrans = yCache';

x_plot = linspace(0,(T+1)/476,T+1);

% Attitude states
figure(1)
subplot(3,1,1)
set(0,'DefaultAxesFontSize',14)
plot(x_plot, xCacheTrans(:,1:3),'linewidth', 2)
hold on
plot(x_plot, xEstCacheTrans(:,1:3),'--', 'linewidth', 1.5)
hold on
plot(x_plot,rCacheTrans(:,1:3), ':', 'linewidth',2)
hold on
% plot(x_plot,yCacheTrans(:,1), 'linewidth',.5)
axis tight
xlabel('time [s]');
ylabel('Quaternions');
lgd = legend('roll', 'pitch', 'yaw', 'est_r_o_l_l' ,'est_p_i_t_c_h',...
    'est_y_a_w','ref_r_o_l_l','ref_p_i_t_c_h','ref_y_a_w');
lgd.NumColumns = 3;
lgd.Location = 'eastoutside';
grid on;

subplot(3,1,2)
set(0,'DefaultAxesFontSize',14)
plot(x_plot, xCacheTrans(:,4:6),'linewidth', 2)
axis tight
xlabel('time [s]');
ylabel('Angular velocities');
lgd = legend('\omega_x', '\omega_y', '\omega_z');
lgd.Location = 'eastoutside';
grid on;

subplot(3,1,3)
set(0,'DefaultAxesFontSize',14)
plot(x_plot, xCacheTrans(:,7:9),'linewidth', 2)
axis tight
xlabel('time [s]');
ylabel('Angular thrusts');
lgd = legend('n_x', 'n_y', 'n_z');
lgd.Location = 'eastoutside';
grid on;

% Altitude states
figure(2)
subplot(1,2,1)
plot(x_plot,xCacheTrans(:,10),'linewidth', 3)
hold on
xlabel('time [s]');
ylabel('Thrust');
lgd = legend('n_t');
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
% plot(x_plot,yCacheTrans(:,7), 'linewidth',.5)
axis tight
xlabel('time [s]');
ylabel('States');
lgd = legend('z','v_z','est_z','est_v_z','ref');
lgd.Location = 'eastoutside';
grid on;

% Navigation states
figure(3)
subplot(3,1,1)
set(0,'DefaultAxesFontSize',14)
plot(x_plot, xCacheTrans(:,1:2),'linewidth', 2)
hold on
plot(x_plot,rCacheTrans(:,1:2), ':', 'linewidth',2)
axis tight
xlabel('time [s]');
ylabel('Quaternions');
lgd = legend('q1', 'q2', 'ref_q_1', 'ref_q_2');
lgd.Location = 'eastoutside';
grid on;

subplot(3,1,2)
set(0,'DefaultAxesFontSize',14)
plot(x_plot, xCacheTrans(:,13:14),'linewidth', 2)
hold on
plot(x_plot, xEstCacheTrans(:,13:14),'--','linewidth', 1.5)
hold on
plot(x_plot,rCacheTrans(:,8:9), ':', 'linewidth',2)
axis tight
xlabel('time [s]');
ylabel('Locations');
lgd = legend('x', 'y', 'est_x', 'est_y', 'ref_x', 'ref_y');
lgd.Location = 'eastoutside';
grid on;

subplot(3,1,3)
set(0,'DefaultAxesFontSize',14)
plot(x_plot, xCacheTrans(:,15:16),'linewidth', 2)
hold on
plot(x_plot, xEstCacheTrans(:,15:16),'--','linewidth', 1.5)
axis tight
xlabel('time [s]');
ylabel('Velocities');
lgd = legend('v_x', 'v_y', 'est_v_x', 'est_v_y');
lgd.Location = 'eastoutside';
grid on;

% Navigation map
figure(4)
plot(0,0,'k.','Markersize', 25);
hold on
plot(1,-2,'g.','Markersize', 25);
hold on
plot(1.5,3,'g.','Markersize', 25);
hold on
plot(xCacheTrans(:,13),xCacheTrans(:,14), 'linewidth', 2, 'Color', 'b');
xlabel('x [m]');
ylabel('y [m]');
grid on



