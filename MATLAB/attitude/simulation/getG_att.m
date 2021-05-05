function G = getG_att(a)
% Calculates matrix G for reference tracking
% Give structure with attitude system matrices as input
    W = [a.Ad-eye(9), a.Bd
         a.Cd,        a.Dd];

    G = W\[zeros(9, 6); eye(6)];

end