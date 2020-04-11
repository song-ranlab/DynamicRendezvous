% Rendezvous docking of an autonomous underwater vehicle to 
% wave energy converter using model predictive control
% and true system dynamics as model for AUV with simulated dynamics of WEC

clear all, close all, clc

 %SystemModel = 'F8';

% figpath = ['../FIGURES/',SystemModel,'/'];mkdir(figpath)
% datapath = ['../DATA/',SystemModel,'/'];mkdir(datapath)
% addpath('../utils');
%% MPC
dt = 0.1;  % MPC timestep
Ts = 0.1;  % Integration timestep
Nvar = 12;   % Number of states 

% Parameters MPC
options = optimoptions('fmincon','Algorithm','sqp','Display','none', ...
    'MaxIterations',100);
Duration = 50;                  % Run for 'Duration' time units
Ton = 0;                       % Time units when control is turned on
getMPCparams                   % MPC parameters

eta0 = [-1 0 0 0 0 0]';
nu0 = [0 0 0 0 0 0]';

x0n=[eta0; nu0]';                % Initial condition

Results = struct('z', [], 'x',[], 'u', [], 't', [], 'xref', [], 'J', []);

%% Run MPC

% Prepare variables // Initialization
Nt       = (Duration/Ts)+1;
uopt0    = [0; 0];   % Initial guess of optimal control
xhat     = x0n; % Updated states
uopt     = uopt0.*ones(2,Nu);
xHistory = zeros(Nvar,Nt); xHistory(:,1) = xhat;
uHistory = zeros(2,Nt); uHistory(:,1) = uopt(:,1);
tHistory = zeros(1,Nt); tHistory(1) = 0;
zHistory = zeros(1,Nt);

% Start simulation
fprintf('Simulation started.  It might take a while...\n')
tic
for ct = 1:Duration/Ts 
    
    % Set references over optimization horizon
    tref = (ct:ct+N-1).*Ts;
    xxref = WECmodel(tref,Ts,N,Nvar);
    
    % NMPC with full-state feedback
     COSTFUN = @(u) ObjectiveFCN(u,xhat,Ts,N,Nu,xxref,uHistory(:,ct),[],diag(Q),R,Ru);
    %CONSFUN = @(u) ConstraintFCN(u,uHistory(:,ct),xhat,Ts,N,LBo,UBo,LBdu,UBdu,[]);   
      uopt = fmincon(COSTFUN,uopt,[],[],[],[],LB,UB,[],options);
    
    % Integrate system
    %xhat = rk4u(@AUVSys,xhat,uopt(1),Ts/10,10,[],0); % Increase time resolution for simulation & keep control constant
    xhat = AUVSys(xhat, uopt(:,1),Ts/10,10); 
    xHistory(:,ct+1) = xhat;
    uHistory(:,ct+1) = uopt(1);
    tHistory(:,ct+1) = ct*Ts;
    rHistory(:,ct+1) = xxref(1,2);
    
end
tElapsed = toc
fprintf('Simulation finished!\n')

%% Collect results
Results.eval_time = tElapsed;
Results.xxref = rHistory;
Results.x = xHistory;
Results.u = uHistory;
Results.t = tHistory;
%Results.J = evalObjectiveFCN(Results.u,Results.x,Results.xref,diag(Q),R,Ru);
%% Show results
figure
hold on
plot(xHistory(1, :))
plot(xHistory(3, :))
plot(xHistory(5, :))
plot(xHistory(7, :))
plot(xHistory(9, :))
plot(xHistory(11, :)) 
plot(z) 
hold off
legend('x','z','theta','u','w','q','zwec')
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