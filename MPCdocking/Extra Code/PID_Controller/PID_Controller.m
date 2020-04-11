%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Written By: Mohammad Y. Saadeh, on 02/15/2012  %
% University of Nevada Las Vegas {UNLV}          %
%                                                %%%%%%%%%%%%%%%%%%%%%%%
% Build and define a MATLAB based PID controller using simple routines %
% Might be useful in cases of Hardware-In-Loop (HIL) applications      %
% where access to Real Time Workshop (RTW) is not possible.            %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%A%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;clear;close all
tic                 % start timer to calculate CPU time

desired = 1;        % desired output, or reference point
feed1 = 1;          % can be replaced with damping coefficient B or (B/Mass)
feed2 = 1;          % can be replaced with spring coefficient K or (K/Mass)
B = feed1;K = feed2;
Kp = 1;             % proportional term Kp
Ki = 0.01;           % Integral term Ki
Kd = 0.01;           % derivative term Kd
dt = 0.01;          % sampling time
Time = 20;          % total simulation time in seconds
n = round(Time/dt); % number of samples

% pre-assign all the arrays to optimize simulation time
Prop(1:n+1) = 0; Der(1:n+1) = 0; Int(1:n+1) = 0; I(1:n+1) = 0;
PID(1:n+1) = 0;
FeedBack(1:n+1) = 0;
Output(1:n+1) = 0;
Error(1:n+1) = 0;
state1(1:n+1) = 0; STATE1(1:n+1) = 0;
state2(1:n+1) = 0; STATE2(1:n+1) = 0;

for i = 1:n
    Error(i+1) = desired - FeedBack(i); % error entering the PID controller
    
    Prop(i+1) = Error(i+1);% error of proportional term
    Der(i+1)  = (Error(i+1) - Error(i))/dt; % derivative of the error
    Int(i+1)  = (Error(i+1) + Error(i))*dt/2; % integration of the error
    I(i+1)    = sum(Int); % the sum of the integration of the error
    
    PID(i+1)  = Kp*Prop(i) + Ki*I(i+1)+ Kd*Der(i); % the three PID terms
    
    %% You can replace the follwoing five lines with your system/hardware/model
    STATE1(i+1) = sum(PID); % sum PID term to calculate the first integration
    state2(i+1) = (STATE1(i+1) + STATE1(i))*dt/2; % output after the first integrator
    STATE2(i+1) = sum(state2); % sum output of first integrator to calculate the second integration
    Output(i+1) = (STATE2(i+1) + STATE2(i))*dt/2; % output of the system after the second integrator
    FeedBack(i+1) = state2(i+1)*feed1 + Output(i+1)*feed2;
end

tsim = toc % simulation time

% plot results
T = 0:dt:Time;
Reference = desired*ones(1,i+1);
plot(T,Reference,'r',T,Output,'b')
xlabel('Time (sec)')
legend('Desired','Simulated')

