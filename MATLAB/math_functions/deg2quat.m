function q = deg2quat(yaw, pitch, roll)
% In ZYX order
q = quatnormalize(angle2quat(deg2rad(yaw), deg2rad(pitch), deg2rad(roll)));