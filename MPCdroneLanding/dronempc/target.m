function [z, z_dot] = target(t, x, omega, w)
% Symplified 1-D (vertical) model of the wave energy converter
% z = sin(omega*t) + w
% Input t: time (s)
% Input omega: frequency
% Inout w: noise
% Output z: z position

omega = 0.35; %default value for omega 
w = 0;  % default value for noise

z = x - sin(omega*t) + w;
z_dot = omega*cos(omega*t);

end

