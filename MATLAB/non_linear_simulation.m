% Let us now simulate the non-linear system starting from 
q  = [1;0;0;0];
omega = [-107.5;0;0];
n = [0;0;0];
x0 = [q;omega;n];
% and using the input signal (quat_dynamics: 'control actions')
u = @(t,x)[exp(-3*t);0;0]; 
% The system dynamics is described by 
p = quat_params();
F = @(t,x) quat_dynamics(x, u(t,x), p);
% Using ode45 we have: 
[t,x] = ode45(@(t,x) F(t,x), [0 2], x0); 

% Plot the quaternion states
figure(1)
plot(t, x(:,1:4),'linewidth', 1);
legend('q0','q1','q2','q3');
grid on; xlabel('Time'); ylabel('State');

% Plot the omega states
figure(2)
plot(t, x(:,5:7),'linewidth', 1);
legend('omega_x','omega_y','omega_z');
grid on; xlabel('Time'); ylabel('State');

% Plot the n states
figure(3)
plot(t, x(:,8:10),'linewidth', 1);
legend('nx','ny','nz');
grid on; xlabel('Time'); ylabel('State');