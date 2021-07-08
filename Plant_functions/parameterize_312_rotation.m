function [phi] = parameterize_312_rotation(C)
%UNTITLED4 Summary of this function goes here
%   spits out x,y,z, or roll,pitch,yaw. 
phi = zeros(3,1);

phi(1) = asin(C(2,3)); %roll
phi(2) = atan2(-C(1,3), C(3,3)); %pitch 
phi(3) = atan2(-C(2,1), C(2,2)); %yaw

end

