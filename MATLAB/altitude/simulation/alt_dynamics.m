function xdot = alt_dynamics(x,u,p)
% ALT_DYNAMICS describes the altitude quadcopter dynamics using
% the state vector:
% x =   [attitude states
%        nt
%        z
%        vz]

% where nt = nc - nh. nc is the vertical thrust and n_h is the
% thrust needed for hovering. z refers to the vertical position and
% vz is the vertical speed. u = ut is the vector of control signals.

%Input arguments:
% x     system state
% u     control actions
% p     quadcopter parameters as returned by quat_params

% We assume nd = 0 since there is not much tilt.

% attitude indices
idx_q = 1:4; 
idx_omega = 5:7;
idx_n = 8:10;

% altitude indices
idx_nt = 11;
idx_z = 12;
idx_vz = 13;

% assigning variables
q = x(idx_q);
omega = x(idx_omega);
n = x(idx_n);
nt = x(idx_nt);
vz = x(idx_vz);

% dynamics
xdot = zeros(13,1,'sym');
xdot(idx_q) = 0.5*quatmultiply(q', [0, omega'])';
xdot(idx_omega) = p.gamma_n*n + p.gamma_u*u(1:3) ...
                - p.I\cross(omega, p.I*omega);
xdot(idx_n) = p.k2 * (p.k1 * u(1:3) - n);
xdot(idx_nt) = -p.k2*nt + p.k1*p.k2*u(4);
xdot(idx_z) = vz;
xdot(idx_vz) = p.kt*((nt+p.nh)^2 + x(8)^2 + x(9)^2 + x(10)^2)* ...
    (1-2*(q(2)^2 + q(3)^2))-p.g;

end