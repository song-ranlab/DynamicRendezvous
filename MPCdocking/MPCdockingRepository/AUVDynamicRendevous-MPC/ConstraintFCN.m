function [c, ceq] = ConstraintFCN(u,uold,x,int_dt,N,LBo,UBo,LBdu,UBdu,nvar)
%% Constraint function of WEC dock
%
% Inputs:
%   u:      optimization variable, from time k to time k+N-1 
%   uold:   latest applied control input
%   x:      current state at time k
%   int_dt: controller sample time
%   N:      prediction horizon length
%   LBo:    Lower bound of output x
%   UBo:    Upper bound of output x
%   LBdu:   Lower bound for input difference uk - uk-1
%   UBdu:   Upper bound for input difference uk - uk-1
%   p:      Parameters for model
%
% Output:
%   c:      inequality constraints applied across prediction horizon
%   ceq:    equality constraints  (empty)
%

%% Nonlinear MPC design parameters
% range of angle of attack
zMin = LBo;
zMax = UBo;

%% Inequality constraints calculation
c = zeros(2*2*N,1); 
cy = zeros(2*nvar*N,1);

% Apply 2N constraints across prediction horizon, from time k+1 to k+N
xk = x;
uk = u(:,1);
duk = uk-uold;
for ct=1:N
    % Obtain new state at next prediction step
%     xk1 = rk4u(@F8Sys,xk,uk,int_dt,1,[],p);
    xk1 = AUVSys(xk,u(:,ct),int_dt,1);

    % -z + zMin < 0 % lower bound
    cy(2*nvar*(ct-1)+1:2*nvar*(ct-1)+nvar) = -xk1+zMin;
    c(2*2*(ct-1)+1:2*2*(ct-1)+2) = -duk+LBdu; 
    
    % z - zMax < 0 % upper bound
    cy(2*nvar*(ct-1)+nvar+1:2*nvar*ct) = xk1-zMax;
    c(2*2*(ct-1)+2+1:2*2*ct) = duk-UBdu;
    
    % Update plant state and input for next step
    xk = xk1;
    if ct<N
        uk = u(:,ct+1);
        duk = u(:,ct+1)-u(:,ct);
    end
end


c = [c;
     cy];

%% No equality constraints
ceq = [];
