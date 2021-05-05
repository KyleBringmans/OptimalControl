% Let us now simulate the linear system starting from 
q  = [0;0;0];
omega = [-107.5;0;0];
n = [0;0;0];
x0 = [q;omega;n];
% and using the input signal
u = @(t,x)[exp(-3*t);0;0];
% The system dynamics is described by 
p = quat_params();
F = @(t,x) quat_linear_dyn(x, u(t,x), p);
% Using ode45 we have: 
[t,x] = ode45(@(t,x) F(t,x), [0 2], x0); 

% Plot the vector part of the quaternion states
figure(1)
plot(t, x(:,1:3),'linewidth', 1);
legend('q1','q2','q3');
grid on; xlabel('Time'); ylabel('State');

% Plot the omega states
figure(2)
plot(t, x(:,4:6),'linewidth', 1);
legend('omega_x','omega_y','omega_z');
grid on; xlabel('Time'); ylabel('State');

% Plot the n states
figure(3)
plot(t, x(:,7:9),'linewidth', 1);
legend('nx','ny','nz');
grid on; xlabel('Time'); ylabel('State');