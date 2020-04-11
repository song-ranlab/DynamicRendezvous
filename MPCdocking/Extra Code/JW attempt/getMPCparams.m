% MPC parameters
N  = 10;                        % Prediction horizon (number of iterations)
Nu  = N;                        % Control horizon (number of iterations)

Q = zeros(12,1);                % State weights
Q(1,1) = 25;                    % x weight            
Q(3,1) = 5;                    % z weight
Q(7,1) = 5;                    % u weight
Q(9,1) = 5;                    % w weight

R = 0.05;                          % du weights, 0
Ru = 0.05;                         % u weights, 0.1

% Lower and upper bound of control input, 
%-34 N is 80% of max output for one motor @ average voltage, 
%0.707 scaling factor for 45° angle times n number of motors
LB = 4*-34*0.707*ones(Nu,1);         
UB = 4*34*0.707*ones(Nu,1);           

LBdu = -0.1;                    % Lower bound of control input rate, -0.1
UBdu = 0.1;                     % Upper bound of control input rate, 0.1

LBo = zeros(12,1);              % Lower bound of output
LBo(1,1) = -40;                 % Lower bound of x output            
LBo(3,1) = -10;                 % Lower bound of z output
LBo(7,1) = -3;                  % Lower bound of u output
LBo(9,1) = -3;                  % Lower bound of w output

LBo = zeros(12,1);              % Upper bound of output
LBo(1,1) = 10;                  % Upper bound of x output            
LBo(3,1) = 10;                  % Upper bound of z output
LBo(7,1) = 3;                   % Upper bound of u output
LBo(9,1) = 3;                   % Upper bound of w output

% Reference trajectory
time_control = 0:dt:Duration;

z = sin(0.2*time_control);
    %figure,plot(time_control,z,'-k'), hold on

zrefFUN = @(t) sin(0.2*t);
    %plot(time_control,zrefFUN(time_control),'--r')