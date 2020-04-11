function xhat = AUVSys(xhat,uopt,h,n)

% Nonlinear rendezvous docking controller
% of a BlueROV 2 Heavy to a floating WEC (constrained to heave).
% Control of u and w.
% Input u: x and z thrust force
% States eta: [x,0,z,0,0,0]
% States nu: [u,0,w,0,0,0]
% Initial state: x(0) = [0 0 0 0 0 0], u(0) = 0
% Tracking reference: z(t) = sin(0.2*t)

eta = xhat(1:6);
nu = xhat(7:12);

tau = zeros(6,1);
tau (1) =  uopt(1);
tau (3) =  uopt(2);

gethydroparams

% %state vector: [x,x2] = [eta,nu]
% nu = [u;v;w;p;q;r]; %nu vector
% eta = [x;y;z;phi;theta;psi]; %eta vector
% 
% %control and Disturbance
% U = [ua, ub, uc, ud, ue, uf]; %control vector
% V = [va, vb, vc, vd, ve, vf]; %disturbance vector

M = mfunc(m,zg,xudot,yvdot,zwdot,kpdot,mqdot,nrdot,ix,iy,iz); % mass = ridgid body mass + added mass
Minv = inv(M); %mass matrix inverse

% tau = taufunc(U); %control matrix
% W = wfunc(V); %distubacne matrix

% h = .01; %time step
% l=25; %lenth of simulation seconds
% ti = 0 % initial time

for i = 1:n  
    
%     t = (i-1)*h;
%     S(i,1) = t;
%     S(i,2:7) = transpose(eta); % eta'
%     S(i,8:13) = transpose(nu);

            %call matrix building functions
            C = cfunc(nu,xudot,yvdot,zwdot,kpdot,mqdot,nrdot,m,fw,ix,iy,iz,zg); %coriolis forces = ridgid body + added
            D = dfunc(nu,xu,yv,zw,kp,mq,nr,xuu,yvv,zww,kpp,mqq,nrr); %damping forces
            G = gfunc(fw,b,zg,eta(4),eta(5),eta(6)); %hydrostatic forces
            J = jfunc(eta(4),eta(5),eta(6)); %refernce frame transform matrix
            
            xhat = eulerfunc2(h,nu,eta,Minv,C,D,G,J,tau);

end


end