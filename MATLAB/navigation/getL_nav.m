function L = getL_nav(na)

p = quat_params;
E = diag([100 100 10 10 1 1]);
kalmanSystem = ss(na.Ad , [na.Bd na.Bd E], na.C, [na.D na.D zeros(2,6)], -1);
[~, Lobserver ] = kalman(kalmanSystem , 1e3*p.covarDynamics_nav, ...
                 p.covarSensors_nav, 0);
L = -Lobserver;

% Let us make sure that A+Lobserver*C is a stable matrix
assert ( all (abs( eig(na.Ad+ L *na.Cd )) < 1-1e-6) , ...
'A+LC not stable ');

end