function [xdot] = LookaheadBasedSteeringWithDynamics(t, states , wptList, currentWpt, nextWpt, lookAheadDistance)

% Fossen book (p261)
% Function returns : xdot = [dx,dy,de]'

% Inputs:
% psi = heading angle we assume that the input is EXACTLY and INSTANTLY
% the craft heading


% State variables:
% x = position of craft on x axis
% y = position of craft on y axis
% psi = heading of craft (is controlled by a PI)
% r = turning rate
% delta = nozzle angle (is controlled by a PD)
% CTerr = min distance craft-track

global U beta phiBeta T K k1 k2 kp ki kpr kdr Tdelta


if isempty(lookAheadDistance)
    lookAheadDistance = 1;      % Look-ahead distance
end
if isempty(currentWpt)
    currentWpt = 1;
end
if isempty(nextWpt)
    nextWpt = 2;
end


% Get nearest waypoint
x_k = wptList(currentWpt,1);
x_k_1 = wptList(nextWpt,1);
y_k = wptList(currentWpt,2);
y_k_1 = wptList(nextWpt,2);


% Calculate heading angle
alphak = atan2( y_k_1 - y_k , x_k_1 - x_k );

x = states(1); %+beta*cos(phiBeta);
y = states(2); %+beta*sin(phiBeta);
psi = states(3);
r = states(4);
delta = states(5);
%deltaC = states(6);
%e = states(7);
% CTerr= -(x-x_k)*sin(gammak)+(y- y_k)*cos(gammak); % WRONG: x_k and y_k should be the projection on the trajectories.
% CTerr
% v = U*sin(beta);
% beta = asin(v/U);
IE = states(6);

% Measured values
xM = x; % +beta*cos(phiBeta);
yM = y; % +beta*sin(phiBeta); 

CTerr = -(xM - x_k)*sin(alphak) + (yM - y_k)*cos(alphak);
chiD = alphak + atan(-kp*CTerr - ki*IE);
psiD = chiD - phiBeta;

Vc = sqrt(U^2+beta^2);
edot = Vc*sin(psi - alphak + phiBeta); % U*sin(psi)*cos(gammak) - U*cos(psi)*sin(gammak);
%psiDdot = -edot * lookAheadDistance / (lookAheadDistance^2 + CTerr^2);

deltaC = -kpr*(psi-psiD) - kdr*r;


% Set derivatives
xdot(1) = U*cos(psi) - beta*sin(psi);% + beta*cos(phiBeta); % x dot
xdot(2) = U*sin(psi) + beta*cos(psi);% + beta*sin(phiBeta); % y dot
xdot(3) = r; % psi dot
xdot(4) = -k1*r + k2*delta; % r dot
xdot(5) = 1/Tdelta*(deltaC-delta); % delta dot
%xdot(6) = -kp*(r - psiDdot) - ki*(psi - psiD);
xdot(6) = CTerr; % integral action
xdot(7) = xdot(2)*cos(alphak) - xdot(1)*sin(alphak); % e dot

xdot = xdot';
end
