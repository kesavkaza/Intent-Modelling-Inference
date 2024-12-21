function [roll, pitch, yaw] = calculateEulerAngles(x, y, z)
    % Calculate the magnitude of the vector
    mag = sqrt(x^2 + y^2 + z^2);
    
    % Normalize the vector
    x = x / mag;
    y = y / mag;
    z = z / mag;
    
    % Calculate the Euler angles
    roll = atan2(y, z);
    pitch = atan2(-x, sqrt(y^2 + z^2));
    yaw = atan2(sqrt(x^2 + y^2), z);
    
    % Convert the angles from radians to degrees
    roll = rad2deg(roll);
    pitch = rad2deg(pitch);
    yaw = rad2deg(yaw);
end
