m=11.5; % kg
zg=0.2; %meters
xudot=-5.5; %kg
yvdot=-12.7;
zwdot=-14.57;
kpdot=-0.12; %kgm^2/rad
mqdot=-0.12;
nrdot=-0.12;
ix=0.16; %kgm^2
iy=0.16;
iz=0.16;

%M = [m-xu 0 0 0 -m*zg 0; 0 m-yv 0 -m*zg 0 0; 0 0 m-zw 0 0 0; 0 -m*zg 0 ix-kp 0 0; -m*zg 0 0 0 iy-mq 0; 0 0 0 0 0 iz-nr];

ma = zeros(6);
a = -[xudot yvdot zwdot kpdot mqdot nrdot];
ma = diag(a)

mrb = [m 0 0 0 -m*zg 0; 0 m 0 -m*zg 0 0; 0 0 m 0 0 0; 0 -m*zg 0 ix 0 0; -m*zg 0 0 0 iy 0; 0 0 0 0 0 iz];

M = ma + mrb