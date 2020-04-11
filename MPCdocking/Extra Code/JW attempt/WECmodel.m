function [S] = WECmodel(t,h,N,Nvar)

S = zeros(N,Nvar);

%generate motion model over time horizon
%for kk = 1:(l-ti)/h  
%time and z position 
   %t = (kk-1)*h;
    z = sin(0.2*t);
    zpre = sin(0.2*(t-h));
    
    %velocity
    v = (z-zpre)/h;
    
    %S(kk,1) = t;
    S(:,3) = z;
    S(:,9) = v;
    %end

end

