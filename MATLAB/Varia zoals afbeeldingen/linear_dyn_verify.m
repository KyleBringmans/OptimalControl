% Define the system dimensions
n_q = 4; % number of q states
n_omega = 3; % number of omega states
n_n = 3; % number of n states
n_x = n_q + n_omega + n_n; % number of total states
n_u = 3; % number of inputs
n_y = 6; % number of outputs

% Define the system parameters
p = quat_params();

% Define the symbols xSymb (states) and uSymb (inputs)
qsymb  = sym('q', [n_q, 1], 'real');
omegasymb = sym('omega', [n_omega, 1], 'real');
nsymb = sym('n', [n_n, 1], 'real');
uSymb = sym('u', [n_u, 1], 'real');
qsymb(1)

qsymb(1) = sqrt(1-qsymb(2)^2-qsymb(3)^2-qsymb(4)^2);

Msymb = [qsymb(1) -qsymb(4) qsymb(3);
    qsymb(4) qsymb(1) -qsymb(2);
    -qsymb(3) qsymb(2) qsymb(1)];

% Define the nonlinear dynamics
idx_q = 1:4; idx_omega = 5:7;  idx_n = 8:10;
q = x(idx_q); omega = x(idx_omega); n = x(idx_n);
xdot = zeros(10,1);
xdot(idx_q) = 0.5 * Msymb * omegasymb;
xdot(idx_omega) = p.gamma_n*nsymb + p.gamma_u*uSymb ...
                - p.I\cross(omegasymb, p.I*omegasymb);
xdot(idx_n) = p.k2 * (p.k1 * uSymb - nsymb);

% Define matrices C and D
C = p.C;
D = p.D;

% Symbolic Jacobians:
Jfx = jacobian(f, xSymb); % Jacobian with respect to x (xSymb)
Jfu = jacobian(f, uSymb); % Jacobian with respect to u (uSymb)

% Linearise f at the origin:
A = double(subs(Jfx, [xSymb; uSymb], zeros(nx+nu,1)));
B = double(subs(Jfu, [xSymb; uSymb], zeros(nx+nu,1)));

% Define the linearised system using `ss`.
% This is a continuous-time system
linearisedSystem = ss(A, B, C, D);

% Discretise the linearised system:
Ts = 0.4;
ZOH = c2d(linearisedSystem, Ts,'zoh');
[Ad, Bd, Cd, Dd] = ssdata(ZOH);

% We choose matrices R and Q to be diagonal. R is a scalar
% since nu = 1 and Q is allowed to have zeros on its diagonal
% (it is only required to be positive semidefinite)
R  = 1;
Q  = diag([1 0.1 1 0.1]);
Kcontroller  = -dlqr(Ad, Bd, Q, R); % mind the minus sign
assert( all(abs(eig(Ad+Bd*Kcontroller)) < 1-1e-6), ...
    'A+BK not stable');


% We specify the covariances for `v` and `w`
covarSensors  = 0.02^2 * eye(ny);
covarDynamics = 0.0001^2 * eye(nu);

% Next we build the Kalman filter:
kalmanSystem      = ss(Ad, [Bd Bd], C, [D D], -1);
[~, Lobserver]   = kalman(kalmanSystem, covarDynamics, covarSensors, 0);
Lobserver = -Lobserver;

% Let us make sure that A+Lobserver*C is a stable matrix
assert( all(abs(eig(Ad+Lobserver*Cd)) < 1-1e-6), ...
    'A+LC not stable');