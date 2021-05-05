function al = alt_lin_system(p)
% altitude_system returns the matrices and state space models of the
% linearised system, both continuous and discrete, in a structure.

% The four matrices of the linearised continuous system and the whole system

% 3x3 matrix
al.A = [-p.k2 0 0;
         0 0 1;
         2*p.nh*p.kt 0 0];

% 3x1 matrix
al.B = [p.k1*p.k2; 0; 0];

% 1x3 matrix
al.C = [0 1 0];

%1x1 matrix
al.D = 0;

al.sys = ss(al.A,al.B,al.C,al.D);

%Now we're going to discretize the above system
%So we get the four matrices of the linearised discrete system and the
%whole system

al.Ts = 1/20;
al.sysd = c2d(al.sys,al.Ts,'zoh');
[ al.Ad , al.Bd , al.Cd , al.Dd ] = ssdata(al.sysd);