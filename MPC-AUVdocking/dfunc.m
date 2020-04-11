%function D = dfunc(u,v,w,p,q,r,xu,yv,zw,kp,mq,nr,xuu,yvv,zww,kpp,mqq,nrr)
function D = dfunc(nu,xu,yv,zw,kp,mq,nr,xuu,yvv,zww,kpp,mqq,nrr)

u = nu(1);
v = nu(2);
w = nu(3);
p = nu(4);
q = nu(5);
r = nu(6);


d = -[xu+(xuu*abs(u)) yv+(yvv*abs(v)) zw+(zww*abs(w)) kp+(kpp*abs(p)) mq+(mqq*abs(q)) nr+(nrr*abs(r))];
%D = zeros(6);
D = diag(d);

end

% u=1;
% v=1;
% w=1;
% p=1;
% q=1;
% r=1;
% 
% xu=-4.03
% yv=-6.22
% zw=-5.18
% kp=-0.07
% mq=-0.07
% nr=-0.07
% 
% xuu=-18.18
% yvv=-21.66
% zww=-36.99
% kpp=-1.55
% mqq=-1.55
% nrr=-1.55
