function L = getL_att(a)

p = quat_params;

E = diag([100 100 100 10 10 10 1 1 1]);

kalmanSystem = ss(a.Ad, [a.Bd a.Bd E], a.C, [a.D a.D zeros(6,9)], -1);

% We could multiply covarDyn or covarSen by a factor to rely more
% on measurements or the system model
[~, Lobserver ] = kalman(kalmanSystem , p.covarDynamics_att, ...
                 p.covarSensors_att , 0);
L = -Lobserver;

% Let us make sure that A+Lobserver*C is a stable matrix
assert ( all (abs( eig(a.Ad+ L *a.Cd )) < 1-1e-6) , ...
'A+LC not stable ');
end


