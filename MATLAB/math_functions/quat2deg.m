function angles = quat2deg(q0,q1,q2,q3)
% quat2deg converts a quaternion to Euler angles
% It works with both the vector part or the whole quaternion
if nargin == 3
    qreal =  real(sqrt(1 - q0^2 - q1^2 - q2^2));
    [r,p,y] = quat2angle([qreal q1 q2 q3], 'XYZ');
    yaw = rad2deg(y);
    pitch = rad2deg(p);
    roll = rad2deg(r);
    angles = [roll,pitch,yaw];
else
    [r,p,y] = quat2angle([q0 q1 q2 q3], 'XYZ');
    yaw = rad2deg(y);
    pitch = rad2deg(p);
    roll = rad2deg(r);
    angles = [roll,pitch,yaw];

end

   


