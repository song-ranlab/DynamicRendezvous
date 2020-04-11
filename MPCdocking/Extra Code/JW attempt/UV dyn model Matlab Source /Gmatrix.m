
w=112.8 %N
b = 114.8 %N
zg=0.2;
phi=1;
theta=1;
psi=1;

G = [(w-b)*sin(theta);-(w-b)*cos(theta)*sin(phi);-(w-b)*cos(theta)*cos(phi);zg*w*cos(theta)*sin(theta);zg*w*sin(theta);0]