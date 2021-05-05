function L = getL_alt(al)

p = quat_params;

%{
kalmanSystem = ss(al.Ad , [al.Bd al.Bd], al.C, [al.D al.D], -1);
[~, Lobserver ] = kalman(kalmanSystem , p.covarDynamics_alt, ...
                 p.covarSensors_alt , 0);
L = -Lobserver;
%}

E = diag([1 1000 1]);
kalmanSystem = ss(al.Ad , [al.Bd al.Bd E], al.C, [al.D al.D zeros(1,3)], -1);
[~, Lobserver ] = kalman(kalmanSystem , p.covarDynamics_alt, ...
                 p.covarSensors_alt , 0);
L = -Lobserver;

% Let us make sure that A+Lobserver*C is a stable matrix
assert ( all (abs( eig(al.Ad+ L *al.Cd )) < 1-1e-6) , ...
'A+LC not stable ');

end