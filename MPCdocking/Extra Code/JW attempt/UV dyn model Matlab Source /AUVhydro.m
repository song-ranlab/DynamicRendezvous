%initialize state vectors
   
 %eta vector components, earth fixed position and euler angles
    x=0;
    y=0;
    z=0;
    phi=0;
    theta=0;
    psi=0;
 %nu vector components, body fixed velocity and angular rates
    u=1;
    v=0;
    w=0;
    p=0;
    q=0;
    r=0;
   

%hydrodynamic coeffiecients
xu=-4.03
yv=-6.22
zw=-5.18
kp=-0.07
mq=-0.07
nr=-0.07
xuu=-18.18
yvv=-21.66
zww=-36.99
kpp=-1.55
mqq=-1.55
nrr=-1.55
xudot=-5.5; %kg
yvdot=-12.7;
zwdot=-14.57;
kpdot=-0.12; %kgm^2/rad
mqdot=-0.12;
nrdot=-0.12;

%physical parameters
m=11.5; % kg
fw=112.8 %N
b = 114.8 %N
zg=0.2; %meters
ix=0.16; %kgm^2
iy=0.16;
iz=0.16;

%Control parameters
ua = 0 
ub = 0
uc = 0
ud = 0
ue = 0
uf = 0

%disturbance para meters
va = 0
vb = 0
vc = 0
vd = 0 
ve = 0
vf = 0

h = 0.1 %time step
l=10 %lenth of sim
tspan = 0:h:10;
n= 0 

%state vector: [x,x2] = [eta,nu]
nu = [u;v;w;p;q;r]; %nu vector
eta = [x;y;z;phi;theta;psi]; %eta vector

%control and Disturbance
U = [ua, ub, uc, ud, ue, uf] %control vector
V = [va, vb, vc, vd, ve, vf] %disturbance vector

%call matrix building functions
M = mfunc(m,zg,xudot,yvdot,zwdot,kpdot,mqdot,nrdot,ix,iy,iz) % mass = ridgid body mass + added mass
C = cfunc(x,y,z,phi,theta,psi,u,v,w,p,q,r,xudot,yvdot,zwdot,kpdot,mqdot,nrdot,m,fw,ix,iy,iz) %coriolis forces = ridgid body + added
D = dfunc(u,v,w,p,q,r,xu,yv,zw,kp,mq,nr,xuu,yvv,zww,kpp,mqq,nrr) %damping forces
G = gfunc(fw,b,zg,phi,theta,psi) %hydrostatic forces
tau = taufunc(U) %control matrix
W = wfunc(V) %distubacne matrix
J = jfunc(phi, theta, psi) %refernce frame transform matrix
% M = zeros(6,6);
% Minv = zeros(6,6);
% C = zeros(6,6);
% D = zeros(6,6);
% G = zeros(6,1);
% tau  = zeros(6,1);

Minv = inv(M) %mass matrix inverse


%equations of state
% nudot = Minv*tau - Minv*(C+D)*nu + Minv*G + W;
% etadot = J*nu;


S = eulerfunc(h,l,nu,eta,Minv,C,D,G,J,tau,W,tspan)

%hold on
figure(1)
plot (S(:,1), S(:,2), 'b-o', S(:,1), S(:,3), 'r-o', S(:,1), S(:,4), 'g-o') %plot x, y, z against t
legend('x','y','z')
%plot (S(:,1), S(:,2), 'b-o')
%plot (S(:,1), S(:,3), 'r-o')
%plot (S(:,1), S(:,4), 'g-o')

figure(2)
plot (S(:,1), S(:,5), 'b-o', S(:,1), S(:,6), 'r-o', S(:,1), S(:,7), 'g-o')
legend('phi','theta','psi')

figure(3)
plot (S(:,1), S(:,8), 'b-o', S(:,1), S(:,9), 'r-o', S(:,1), S(:,10), 'g-o') 
legend('u','v','w')

figure(4)
plot (S(:,1), S(:,11), 'b-o', S(:,1), S(:,12), 'r-o', S(:,1), S(:,13), 'g-o') 
legend('p','q','r')