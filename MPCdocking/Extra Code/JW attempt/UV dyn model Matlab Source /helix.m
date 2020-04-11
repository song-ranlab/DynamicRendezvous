clc 
clear


r = 5;
c = 5;

h = .1; %time step
l = 10; %lenth of simulation seconds
ti = 0 % initial time


for i = 1:(l-ti)/h  
    
    t = (i-1)*h;
    
    x = r*cos(t);
    y = r*sin(t);
    z = c*t;
  
    A(i,1) = x;
    A(i,2) = y;
    A(i,3) = z;
    A(i,4) = t;
end

figure (7)
plot3( A(:,1), A(:,2), A(:,3))
grid on