function a = att_lin_system(p)
%attitude_system returns the matrices and state space models of the
%linearised system, both continuous and discrete, in a structure.


%The four matrices of the linearised continuous system and the whole system

a.A = [zeros(3,3) 0.5*eye(3,3) zeros(3,3); zeros(3,3) zeros(3,3) ...
    p.gamma_n; zeros(3,3) zeros(3,3) -(p.k2)*eye(3)];

a.B = [zeros(3,3); p.gamma_u; (p.k2)*(p.k1)*eye(3)];

a.C = [eye(6) zeros(6,3)];

a.D = zeros(6,3);

a.sys = ss(a.A,a.B,a.C,a.D);

%Now we're going to discretize the above system
%So we get the four matrices of the linearised discrete system and the
%whole system

a.Ts = 1/476;
a.sysd = c2d(a.sys,a.Ts,'zoh');
[ a.Ad , a.Bd , a.Cd , a.Dd ] = ssdata(a.sysd);
