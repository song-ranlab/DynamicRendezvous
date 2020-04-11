function J = ObjectiveFCN(u,x,int_dt,N,wec,P,Q,R,Ru)
%% Cost function of nonlinear MPC for WEC-Dock system
%
% Inputs:
%   u:      optimization variables (X,Z), from time k to time k+N-1 
%   x:      current state at time k
%   Ts:     controller sample time
%   N:      prediction horizon (number of intervals)
%   Nu:     control horizon
%   z:      WEC states, varying from time k+1 to k+N
%   u0:     previous controller output at time k-1
%   P:      terminal weights
%   Q:      State weights
%   R:      Penalization/weights on rate of change in u, uk - uk-1 
%   Ru:     Control input u penalization/weights
%
% Output:
%   J:      objective function cost
%

%% Nonlinear MPC design parameters

%% Cost Calculation
% Set initial plant states, controller output and cost
% xk = x;
% uk = u(:,1);
J = 0;


% Loop through each prediction step except for the final step
for ct=1:N-1
    
    % Obtain plant state at next prediction step
    %xk1 = rk4u(@F8Sys,xk,uk,Ts,1,[],p);
    x = AUVSys(x,u(:,ct),int_dt,1);
       
    % Accumulate state tracking cost from x(k+1) to x(k+N-1)
    J = abs(J + (x-wec(:,ct))'*Q*(x-wec(:,ct))); 
        
    % Accumulate control cost from x(k+1) to x(k+N-1)
    J = abs(J + u(:,ct)'*Ru*u(:,ct));
    
    % Accumulate rate of change cost from u(k) to u(k+N-1)
    J = abs(J + (u(:,ct+1) - u(:,ct))'./int_dt*R*(u(:,ct+1) - u(:,ct))./int_dt);
    

    
    % Update xk and uk for the next prediction step
%     xk = xk1;
%     if ct<N
%         uk = u(:,ct+1);
%     end
end

% Add terminal cost
x = AUVSys(x,u(:,N),int_dt,1);
J = abs(J + (x-wec(N))'*P*(x-wec(N)));

end
