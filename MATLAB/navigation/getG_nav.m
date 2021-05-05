function G = getG_nav(na)
% Calculates matrix G for reference tracking
% Give structure with attitude system matrices as input
    W = [na.Ad-eye(6), na.Bd
         na.Cd,        na.Dd];

    G = W\[zeros(6, 2); eye(2)];
end