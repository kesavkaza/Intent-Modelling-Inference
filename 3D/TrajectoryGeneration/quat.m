function [vec] = quat(x,y,z,tta)
% generates the quaternion representation and outputs [x,y,z,qw,qx,qy,qz]
    vec = [x, y, z, cosd(tta/2), x*sind(tta/2), y*sind(tta/2), z*sind(tta/2)];
end