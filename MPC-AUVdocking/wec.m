function [z, z_dot] = wec(t, omega, w)
% Symplified 1-D (vertical) model of the wave energy converter
% z = sin(omega*t) + w
% Input t: time (s)
% Input omega: frequency
% Inout w: noise
% Output z: z position

switch nargin
    case 1
        omega = 0.35; %default value for omega 
        w = 0;  % default value for noise
    case 2
        w = 0;
end

z = - sin(omega*t) + w;
z_dot = omega*cos(omega*t);

end

