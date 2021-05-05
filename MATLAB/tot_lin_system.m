function t = tot_lin_system(p)
% tot_system returns the matrices and state space models of the
% linearised system, both continuous and discrete, in a structure.

% The four matrices of the linearised continuous system and the whole system

% 9x9 matrix
attitudePart_A = [zeros(3,3) 1/2*eye(3,3) zeros(3,3);
                  zeros(3,3) zeros(3,3) p.gamma_n;
                  zeros(3,3) zeros(3,3) -(p.k2)*eye(3)];

% 3x3 matrix
altitudePart_A = [-p.k2 0 0;
                  0 0 1;
                  2*p.nh*p.kt 0 0];

% 12x12 matrix
t.A = [attitudePart_A zeros(9,3);
             zeros(3,9) altitudePart_A];

% 9x3 matrix
attitudePart_B = [zeros(3,3); p.gamma_u; (p.k2)*(p.k1)*eye(3)];    

% 3x1 matrix
altitudePart_B = [p.k1*p.k2; 0; 0];

% 12x4 matrix
t.B = [attitudePart_B zeros(9,1);
             zeros(3,3) altitudePart_B];

% 1x12 matrix
t.C = [zeros(1,10) 1 0];

%1x4 matrix
t.D = [0 0 0 0];

t.sys = ss(t.A,t.B,t.C,t.D);

%Now we're going to discretize the above system
%So we get the four matrices of the linearised discrete system and the
%whole system

t.Ts = 1/476;
t.sysd = c2d(t.sys,t.Ts,'zoh');
[ t.Ad , t.Bd , t.Cd , t.Dd ] = ssdata(t.sysd);
