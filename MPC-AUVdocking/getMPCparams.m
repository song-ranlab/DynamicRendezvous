% MPC parameters
N  = 20;                       % Prediction horizon (number of iterations)
Nu  = N;                        % Control horizon (number of iterations)

P = 50*eye(12); %50

Q = 10*eye(12);                % State error weights 10

R = 1*eye(2);                          % du weights, 1
Ru = 0.1*eye(2);                         % u weights, 0.1

% Lower and upper bound of control input, 
%-34 N is 80% of max output for one motor @ average voltage, 
%0.707 scaling factor for 45° angle times n number of motors
LB = [-4*34*0.707; -4*34].*ones(2,N);
UB = [4*34*0.707; 4*34].*ones(2,N);           

LBdu = -100*ones(2,1);                    % Lower bound of control input rate, -0.1
UBdu = 100*ones(2,1);                     % Upper bound of control input rate, 0.1

LBo = zeros(12,1);              % Lower bound of output
LBo(1,1) = -5;                 % Lower bound of x output            
LBo(3,1) = -5;                 % Lower bound of z output
LBo(7,1) = -3;                  % Lower bound of u output
LBo(9,1) = -3;                  % Lower bound of w output

UBo = zeros(12,1);              % Upper bound of output
UBo(1,1) = 5;                  % Upper bound of x output            
UBo(3,1) = 5;                  % Upper bound of z output
UBo(7,1) = 3;                   % Upper bound of u output
UBo(9,1) = 3;                   % Upper bound of w output

% % Reference trajectory
% t_control = 0:dt:Duration;
% 
% z = sin(0.2*time_control);
%     %figure,plot(time_control,z,'-k'), hold on
% 
% zrefFUN = @(t) sin(0.2*t);
%     %plot(time_control,zrefFUN(time_control),'--r')