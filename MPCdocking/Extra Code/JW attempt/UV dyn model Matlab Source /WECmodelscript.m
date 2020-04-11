clc
clear
% time parameters
l=100;
h=0.1;
ti=0;
S = zeros(3,(l-ti)/h)+1;

%generate motion model over time horizon
for kk = 1:(l-ti)/h  
%time and z position 
    t = (kk-1)*h;
    z = sin(0.2*t);
    zpre = sin(0.2*(t-h));
    
    %velocity (analytic solution)
    v = (z-zpre)/h;

    
    S(kk,1) = t;
    S(kk,2) = z;
    S(kk,3) = v;
end

figure
plot(S(:,1),S(:,2),'r',S(:,1),S(:,3),'g')
legend ('position', 'velocity')

