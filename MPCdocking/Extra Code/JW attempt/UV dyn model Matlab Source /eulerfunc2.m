function [eta, nu] = eulerfunc2(h,nu,eta,Minv,C,D,G,J,tau,W)

   nu = nu + (h*(Minv*(tau + W - (C + D) * nu - G)));
    eta = eta + (h * (J * nu));


end
