function result = quatSubstract(A, B);
% performs a quaterions substraction on two, 3 dimensional
% quaterions

A_comp = quatComplete(A);
B_comp = quatComplete(B);

%/////
B_conj = quatconj(B_comp');
difference = quatmultiply(A_comp',B_conj);
result = difference(2:4)';