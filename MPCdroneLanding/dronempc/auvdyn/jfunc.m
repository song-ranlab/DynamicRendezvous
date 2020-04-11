function J = jfunc(phi, theta, psi)

zm = zeros(3,3);

j1 = [cos(psi)*cos(theta) (-sin(psi)*cos(phi))+(cos(psi)*sin(theta)*sin(phi)) (sin(psi)*sin(phi))+(cos(psi)*cos(phi)*sin(theta)); sin(psi)*cos(theta) (cos(psi)*cos(phi))+(sin(phi)*sin(theta)*sin(psi)) (-cos(psi)*sin(phi))+(sin(theta)*sin(psi)*cos(phi)); -sin(theta) cos(theta)*sin(phi) cos(theta)*cos(phi)];
j2 = [1 sin(phi)*tan(theta) cos(phi)*tan(theta); 0 cos(phi) -sin(phi); 0 sin(phi)/cos(theta) cos(phi)/cos(theta)];

J = [j1 zm; zm j2];

%J = zeros(6,6);

end


% phi=1;
% theta=1;
% psi=1;
