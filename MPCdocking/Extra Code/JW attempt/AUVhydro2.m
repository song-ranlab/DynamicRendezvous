clc
clear

%initialize state vectors
   
 %eta vector components, earth fixed position and euler angles
    x=0;%m
    y=0;
    z=0;
    phi=0;%rad
    theta=0;
    psi=0;
 %nu vector components, body fixed velocity and angular rates
    u=1;%m/s
    v=0;
    w=0;
    p=0;%rad/s
    q=0;
    r=0;
   

%hydrodynamic coeffiecients
xu= -4.03;
yv=-6.22;
zw=-5.18;
kp=-0.07;
mq=-0.07;
nr=-0.07;
xuu=-18.18;
yvv=-21.66;
zww=-36.99;
kpp=-1.55;
mqq=-1.55;
nrr=-1.55;
xudot=-5.5; %kg
yvdot=-12.7;
zwdot=-14.57;

kpdot=-0.12; %kgm^2/rad
mqdot=-0.12;
nrdot=-0.12;

%physical parameters
m=11.5; % kg
fw=112.8; %N
b= 114.8; %N
zg= 0.02; %meters 0.02
ix=0.16; %kgm^2
iy=0.16;
iz=0.16;

%Control parameters
ua = 0;
ub = 0;
uc = 0;
ud = 0;
ue = 0;
uf = 0;

%disturbance parameters
va = 0;
vb = 0;
vc = 0;
vd = 0;
ve = 0;
vf = 0;

%state vector: [x,x2] = [eta,nu]
nu = [u;v;w;p;q;r]; %nu vector
eta = [x;y;z;phi;theta;psi]; %eta vector

%control and Disturbance
U = [ua, ub, uc, ud, ue, uf]; %control vector
V = [va, vb, vc, vd, ve, vf]; %disturbance vector

M = mfunc(m,zg,xudot,yvdot,zwdot,kpdot,mqdot,nrdot,ix,iy,iz); % mass = ridgid body mass + added mass
Minv = inv(M); %mass matrix inverse

tau = taufunc(U); %control matrix
W = wfunc(V); %distubacne matrix


h = .01; %time step
l=25; %lenth of simulation seconds
ti = 0 % initial time


%while n<i
for i = 1:(l-ti)/h  
    
    t = (i-1)*h;
    S(i,1) = t;
    S(i,2:7) = transpose(eta); % eta'
    S(i,8:13) = transpose(nu);

    
            %call matrix building functions
            C = cfunc(nu,xudot,yvdot,zwdot,kpdot,mqdot,nrdot,m,fw,ix,iy,iz,zg); %coriolis forces = ridgid body + added
            D = dfunc(nu,xu,yv,zw,kp,mq,nr,xuu,yvv,zww,kpp,mqq,nrr); %damping forces
            G = gfunc(fw,b,zg,eta(4),eta(5),eta(6)); %hydrostatic forces
            J = jfunc(eta(4),eta(5),eta(6)); %refernce frame transform matrix
            

    [eta, nu] = eulerfunc2(h,nu,eta,Minv,C,D,G,J,tau,W);

 
   %n=n+1;
end



%hold on
figure(1)
plot (S(:,1), S(:,2), 'b-', S(:,1), S(:,3), 'r-', S(:,1), S(:,4), 'g-') %plot x, y, z against t
legend('x','y','z')
%plot (S(:,1), S(:,2), 'b-o')
%plot (S(:,1), S(:,3), 'r-o')
%plot (S(:,1), S(:,4), 'g-o')

figure(2)
plot (S(:,1), S(:,5), 'b-', S(:,1), S(:,6), 'r-', S(:,1), S(:,7), 'g-')
legend('phi','theta','psi')

figure(3)
plot (S(:,1), S(:,8), 'b-', S(:,1), S(:,9), 'r-', S(:,1), S(:,10), 'g-') 
legend('u','v','w')

figure(4)
plot (S(:,1), S(:,11), 'b-', S(:,1), S(:,12), 'r-', S(:,1), S(:,13), 'g-') 
legend('p','q','r')

figure(5)
clf(5)
hold on
plot (S(:,2), S(:,4), 'b-') 
%quiver(S(:,2), S(:,4),S(:,8),S(:,10))
legend('x-z path')

figure(6)
clf(6)
hold on
plot (S(:,2), S(:,3), 'b-') 
%quiver(S(:,2), S(:,3),S(:,8),S(:,9))
legend('x-y path')