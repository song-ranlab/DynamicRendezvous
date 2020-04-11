
function G = gfunc(fw, b, zg, phi, theta, psi)

G = [(fw-b)*sin(theta);-(fw-b)*cos(theta)*sin(phi);-(fw-b)*cos(theta)*cos(phi);zg*fw*cos(theta)*sin(phi);zg*fw*sin(theta);0];

%G=zeros(6,1);

end


% fw=112.8 %N
% b = 114.8 %N
% zg=0.2;
% phi=1;
% theta=1;
% psi=1;