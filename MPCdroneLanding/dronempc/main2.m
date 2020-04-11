% Rendezvous landing of a crazyflie quadcopter on a turtlebot oscillating 
%with sinusoidal velocity in 2d using model predictive controland true 
%system dynamics as model for quadrotor 


clear all, close all, clc

addpath 'C:\Users\ranuh\Desktop\JonW\dronempc\python'
addpath 'C:\Users\ranuh\Desktop\JonW\dronempc\quaddyn\spring2020'
addpath 'C:\Users\ranuh\Desktop\JonW\dronempc\viconMATLAB'
addpath 'C:\Users\ranuh\Desktop\JonW\dronempc\results'
%% MPC
mpc_dt = 0.1;    % MPC timestep
int_dt = 0.1;    % Integration timestep
nvar = 12;   % Number of states 

% Parameters MPC
options = optimoptions('fmincon','Algorithm','sqp','Display','iter', ...
    'MaxIterations',100); 
duration = 500;                % Run for 'Duration' time units (second)
getMPCparams                   % MPC parameters

eta0 = [-2 0 1 0 0 0]';         % Initial conditions
nu0 = [1 0 0.25 0 0 0]';

x0n=[eta0; nu0];                % Initial state vector

Results = struct('x', [], 'u',[], 't', [], 'wec', [], 'J', [], 'ct', []);

%% Run MPC

% Prepare variables // Initialization
nt       = (duration/int_dt)+1; % number of (integration) time steps
u_opt     = zeros(2,N);   % Initial guess of optimal control 
x_hat     = x0n; % Updated states
target_hat   = zeros(nvar,1);
xHistory = zeros(nvar,nt); xHistory(:,1) = x0n;
uHistory = zeros(2,nt); uHistory(:,1) = u_opt(:,1);
tHistory = zeros(1,nt); tHistory(1) = 0;
targetHistory = zeros(nvar,nt);
x_tol = 0.05;    % x tolerance for arrival
z_tol = 0.05;    % z tolerance for arrival

loadsdk          %connect to vicon data stream

getsdkframe; %update motion capture data
x_bar = globalposition(1).Translation;
xhat(1:3) = x_bar;
target_bar = globalposition(2).Translation;
        
% Start simulation
fprintf('Simulation started.  It might take a while...\n')
tic
for ct = 1:duration/int_dt+1   
    % exit if docking error is within tolerance
    error_x = abs(x_hat(1)-targetHistory(1,ct));
    error_z = abs(x_hat(3)-targetHistory(3,ct));
    if error_x<x_tol && error_z<z_tol
        fprintf('Arrived!')
        break
    else
        
        % Calcuate WEC states over optimization horizon
        z_t = (ct+1:ct+N).*int_dt; %MPC prediction horizon time window
        [z_horizon, z_dot_horizon] = target(z_t,target_bar(1));
        target_horizon = zeros(nvar,N);
        target_horizon(3,:) = z_horizon;
        target_horizon(9,:) = z_dot_horizon;

        %NMPC with full-state feedback
        COSTFUN = @(u) ObjectiveFCN(u,x_hat,int_dt,N,target_horizon,P,Q,R,Ru);
        CONSFUN = @(u) ConstraintFCN(u,uHistory(:,ct),x_hat,int_dt,N,LBo,UBo,LBdu,UBdu,nvar); 
        u_opt = fmincon(COSTFUN,u_opt,[],[],[],[],LB,UB,CONSFUN,options);

        % Integrate system
        x_hat = QuadSys(x_hat, u_opt(:,1),int_dt/10,10); % Increase time resolution for simulation & keep control constant
        
        x_cf = x_hat; %assign x_cf as the position commands for controller
        
        getsdkframe; %update motion capture data
        x_bar = globalposition(1).Translation; 
        target_bar = globalposition(2).Translation;
        x_hat(1:3) = x_bar; %update position with motion capture data
        
        xHistory(:,ct+1) = x_hat;
        uHistory(:,ct+1) = u_opt(:,1);
        tHistory(:,ct+1) = ct*int_dt;
        targetHistory(:,ct+1) = target_horizon(:,1);
         
        %send control commands via python (ICP/IP port)
        commandStr = sprintf('python C:\\Users\\ranuh\\Desktop\\JonW\\dronempc\\python\\IPpyclientpos.py x=%d,y=%d,z=%d', x_cf(1), x_cf(2), x_cf(3));
        [status, commandOut] = system(commandStr)
%         if status==0
%             fprintf('Python output is %s\n',commandOut);
%         end
        toc
    end
end
tElapsed = toc
fprintf('Simulation finished!\n')

unloadsdk

%% Collect results
Results.eval_time = tElapsed;
Results.target = targetHistory;
Results.x = xHistory;
Results.u = uHistory;
Results.t = tHistory;
Results.ct = ct;
%Results.J = evalObjectiveFCN(Results.u,Results.x,Results.xref,diag(Q),R,Ru);
%% Show results
figure
hold on
plot(xHistory(1, :),'--')
plot(xHistory(3, :))
plot(xHistory(7, :))
plot(xHistory(9, :))
plot(targetHistory(3, :)) 
hold off
legend('x','z','u','w','zwec')
%figure,plot(uHistory)
%figure,plot(Results.J)
%% 





% %Cost
% J = Q.'*ek.'*ek*Q + R.'*dV.'*Vd*R + S.'*tau.'*tau*S
% %constraints
% zMin = LBo;
% zMax = UBo;
% %optimize
% uopt = fmincon(COSTFUN,uopt,[],[],[],[],LB,UB,CONSFUN,options);
% 
% %Intergrate
% xhat = rk4u(@F8Sys,xhat,uopt(1),Ts/10,10,[],0); %runge 