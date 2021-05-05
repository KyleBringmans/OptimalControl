function xdotlin = nav_linear_dyn(x, u, p)
% NAV_LINEAR_DYN describes the quadcopter linear dynamics using the state 
% vector

xdotlin = [-p.lambda 0 0 0 0 0;
           0 -p.lambda 0 0 0 0;
           0 0 0 0 1 0;
           0 0 0 0 0 1;
           0 2*p.g 0 0 0 0;
           -2*p.g 0 0 0 0 0;] * x + [p.lambda 0;
                                     0 p.lambda;
                                     0 0;
                                     0 0;
                                     0 0;
                                     0 0;] * u;

end