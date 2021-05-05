function na = nav_lin_system(p)
% Navigation system returns the matrices and state space models of the
% linearised system, both continuous and discrete, in a structure.

na.A = [-p.lambda 0 0 0 0 0;
       0 -p.lambda 0 0 0 0;
       0 0 0 0 1 0;
       0 0 0 0 0 1;
       0 2*p.g 0 0 0 0;
       -2*p.g 0 0 0 0 0;];
   
na.B = [p.lambda 0;
        0 p.lambda;
        0 0;
        0 0;
        0 0;
        0 0;];
    
na.C = [0 0 1 0 0 0;
        0 0 0 1 0 0];

na.D = zeros(2,2);

na.sys = ss(na.A,na.B,na.C,na.D);

% Now we're going to discretize the above system
% So we get the four matrices of the linearised discrete system and the
% whole system

na.Ts = 1/15;
na.sysd = c2d(na.sys,na.Ts,'zoh');
[ na.Ad , na.Bd , na.Cd , na.Dd ] = ssdata(na.sysd);














