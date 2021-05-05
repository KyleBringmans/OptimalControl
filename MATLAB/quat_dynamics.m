function xdot = quat_dynamics(x, u, p)
%QUAT_DYNAMICS describes the quadcopter dynamics using the state vector
% x= [q 
%     omega 
%     n] (column vector)
%where q = (q0, q1, q2, q3) is the quaternion describing the orientation of
%the quadcopter, omega  = (omega_x, omega_y, omega_z) is its angular velocity 
%and n is an internal state used to model the dynamics of the motors and
%the propellers. Then, u = (ux, uy, uz) is the vector of control signals.
%
%Syntax
% xdot = quat_dynamics(x, u, p)
%
%Input arguments:
% x     system state
% u     control actions
% p     quadcopter parameters as returned by quat_params
%
%See also
%quat_params, quat_linear_dyn

idx_q = 1:4; idx_omega = 5:7;  idx_n = 8:10;
q = x(idx_q); omega = x(idx_omega); n = x(idx_n);
xdot = zeros(10,1,'sym');
xdot(idx_q) = 0.5*quatmultiply(q', [0, omega'])';
xdot(idx_omega) = p.gamma_n*n + p.gamma_u*u ...
                - p.I\cross(omega, p.I*omega);
xdot(idx_n) = p.k2 * (p.k1 * u - n);


