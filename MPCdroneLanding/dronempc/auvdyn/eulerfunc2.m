function xhat = eulerfunc2(h,nu,eta,Minv,C,D,G,J,tau)

    nu = nu + (h*(Minv*(tau - (C + D) * nu - G)));
    eta = eta + (h * (J * nu));

    xhat = [eta; nu];
    
end
