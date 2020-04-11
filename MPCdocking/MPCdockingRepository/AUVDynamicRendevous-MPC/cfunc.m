%function C = cfunc(u,v,w,p,q,r,xudot,yvdot,zwdot,kpdot,mqdot,nrdot,m,fw,ix,iy,iz)
function C = cfunc(nu,xudot,yvdot,zwdot,kpdot,mqdot,nrdot,m,fw,ix,iy,iz,zg)

u = nu(1);
v = nu(2);
w = nu(3);
p = nu(4);
q = nu(5);
r = nu(6);


ca = [0 0 0 0 zwdot*w 0; 
    0 0 0 -zwdot*w 0 -xudot*u; 
    0 0 0 -yvdot*v xudot*u 0; 
    0 -zwdot*w yvdot*v 0 -nrdot*r mqdot*q; 
    zwdot*w 0 -xudot*u nrdot*r 0 -kpdot*p; 
    -yvdot*v xudot*u 0 -mqdot*q kpdot*p 0];

% crb = [0 0 0 0 m*w 0; 
%        0 0 0 -m*w 0 0; 
%       0 0 0 m*v -m*u 0; 
%       0 m*w -m*v 0 iz*r -iy*q; 
%       -m*w 0 -m*u -iz*r 0 ix*p; 
%       m*v -m*u 0 iy*q -ix*p 0];

crb = [0 0 0 m*zg*r m*w -m*v; 
       0 0 0 -m*w m*zg*r -m*u; 
      0 0 0 -m*(zg*p-v) -m*(zg*q+u) 0; 
      -m*zg*r m*w m*(zg*p-v) 0 iz*r -iy*q; 
      -m*w -m*zg*r m*(zg*q+u) -iz*r 0 ix*p; 
      m*v m*u 0 iy*q -ix*p 0];  
  
C = ca+crb;
%C=zeros(6,6);

end

% fw=112.8 %N
% xudot=-5.5; %kg
% yvdot=-12.7;
% zwdot=-14.57;
% kpdot=-0.12; %kgm^2/rad
% mqdot=-0.12;
% nrdot=-0.12;
% ix=0.16; %kgm^2
% iy=0.16;
% iz=0.16;
% 
% u=1;
% v=1;
% w=1;
% p=1;
% q=1;
% r=1;
% 
% x=1;
% y=1;
% z=1;
% phi=1;
% theta=1;
% psi=1;
