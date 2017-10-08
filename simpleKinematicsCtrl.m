function xdot = simpleKinematicsCtrl(t,x)
kx = 1;
ky = 1;
kt = 1;

u(1) = -kx*x(1);
u(2) = -ky*x(2);
u(3) = -kt*x(3);

xdot(1) = x(2);
xdot(2) = u(1);
xdot(3) = x(4);
xdot(4) = u(2);
xdot(5) = x(6);
xdot(6) = u(3);
xdot = xdot';
% xdot(1) =  u(1) * cos(x(2));
% xdot(2) = -u(1) * sin(x(2)) / x(1) - u(2);
% xdot(3) =  u(1) * sin(x(2)) / x(1);
end
