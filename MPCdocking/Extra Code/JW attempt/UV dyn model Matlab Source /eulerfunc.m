function [S] = eulerfunc(h,l,nu,eta,Minv,C,D,G,J,tau,W,tspan)

i  = length(tspan)
n=0

while n <= i
    
    t = n/l
    
    S(n+1,1) = t
    S(n+1,2:7) = transpose(eta)
    S(n+1,8:13) = transpose(nu)
    
    nu = nu + (h*(Minv*(tau + W - (C + D) * nu - G)));

    eta = eta + (h * (J * nu));

%     nu(t+1) = nu(t) + h*Minv*(tau(t) + W(t) - (C(nu(t)) + D(nu(t)))*nu(t) - G(eta(t)));
% 
%     eta(t+1) = eta(t) + h*(J(eta(t)))*nu(t+1);

    n = n + 1;
end

end