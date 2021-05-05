function result = quatComplete(A)
% completes the reduced (3d) unit quaternion.
% the result is the 4d quaternion

q0 = real(sqrt(1 - A(1)^2 - A(2)^2 - A(3)^2));
result = [q0;A];