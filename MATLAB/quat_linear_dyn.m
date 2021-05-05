function xdotlin = quat_linear_dyn(x, u, p)
% QUAT_LINEAR_DYN describes the quadcopter linear dynamics using the state 
% vector

xdotlin_A = [zeros(3,3) 1/2*eye(3,3) zeros(3,3);
    zeros(3,3) zeros(3,3) p.gamma_n;
    zeros(3,3) zeros(3,3) -(p.k2)*eye(3)];

xdotlin_B = [zeros(3,3); p.gamma_u; (p.k2)*(p.k1)*eye(3)];

xdotlin = xdotlin_A*x + xdotlin_B*u;
end