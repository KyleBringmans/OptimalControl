function xdot = nav_dynamics(x,u,p)
% NAV_DYNAMICS describes the navigation quadcopter dynamics using
% the state vector:
% x =   [attitude states
%        altitude states
%        x
%        y
%        vx
%        vy]

% where nt = nc - nh. nc is the vertical thrust and n_h is the
% thrust needed for hovering.
% x and y refer to positions in x and y directions and
% vx and vy are the velocities in the x and y directions.

%Input arguments:
% x     system state
% u     control actions
% p     quadcopter parameters as returned by quat_params

% attitude indices
idx_q = 1:4; 
idx_omega = 5:7;
idx_n = 8:10;

% altitude indices
idx_nt = 11;
idx_z = 12;
idx_vz = 13;

% navigation indices
idx_x = 14;
idx_y = 15;
idx_vx = 16;
idx_vy = 17;

% assigning each variable
q = x(idx_q);
omega = x(idx_omega);
n = x(idx_n);
nt = x(idx_nt);
vz = x(idx_vz);
vx = x(idx_vx);
vy = x(idx_vy);
nd = n(1)^2 + n(2)^2 + n(3)^2;

% DYNAMICS
% Attitude states
xdot = zeros(15,1,'sym');
xdot(idx_q) = 0.5*quatmultiply(q', [0, omega'])';
xdot(idx_omega) = p.gamma_n*n + p.gamma_u*u(1:3) ...
                - p.I\cross(omega, p.I*omega);
xdot(idx_n) = p.k2 * (p.k1 * u(1:3) - n);

% Altitude states
xdot(idx_nt) = -p.k2*nt + p.k1*p.k2*u(4);
xdot(idx_z) = vz;
xdot(idx_vz) = p.kt*((nt+p.nh)^2 + x(8)^2 + x(9)^2 + x(10)^2)* ...
    (1-2*(q(2)^2 + q(3)^2))-p.g;

% Navigation states
xdot(idx_x) = vx;
xdot(idx_y) = vy;
xdot(idx_vx) = 2*p.kt*(nd + (nt + p.nh)^2)*(q(1)*q(3) + q(2)*q(4));
xdot(idx_vy) = 2*p.kt*(nd + (nt + p.nh)^2)*(q(3)*q(4) - q(1)*q(2));

end