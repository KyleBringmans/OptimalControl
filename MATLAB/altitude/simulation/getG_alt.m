function G = getG_alt(a)
% Calculates matrix G for reference tracking
% Give structure with attitude system matrices as input
    W = [a.Ad-eye(3), a.Bd
         a.Cd,        a.Dd];

    G = W\[zeros(3, 1); 1];

end