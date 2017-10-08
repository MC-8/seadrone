function [xdot] = LookaheadBasedSteeringWithDynamicsAndBigTurn(t, states , wptList, currentWpt, nextWpt, lookAheadDistance)

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

global U beta phiBeta


if isempty(lookAheadDistance)
    lookAheadDistance = 1;      % Look-ahead distance
end
if isempty(currentWpt)
    currentWpt = 1;
end
if isempty(nextWpt)
    nextWpt = 2;
end
U = 10;
% Define parameteres
T = 1.72;  % From thesis Lukas
K = 2.06;  % From thesis Lukas
k1 = 1/T;
k2 = K/T; 
kp = 1/lookAheadDistance;   % Proportional action heading
ki = 0.1/lookAheadDistance; % Integral action heading
kpr = 10; % Proportional action rudder
kdr = 5;  % Derivative action rudder
Tdelta = 0.33; % "It takes 1 second to go from far left to far right" Thesis Casper Ole 2012

% Get nearest waypoint
x_k = wptList(currentWpt,1);
x_k_1 = wptList(nextWpt,1);
y_k = wptList(currentWpt,2);
y_k_1 = wptList(nextWpt,2);


% Calculate heading angle
gammak = atan2( y_k_1 - y_k , x_k_1 - x_k );

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
xM = x;%+beta*cos(phiBeta);
yM = y;%+beta*sin(phiBeta);

CTerr = -(xM - x_k)*sin(gammak) + (yM - y_k)*cos(gammak);
chiD = gammak + atan(-kp*CTerr - ki*IE);
psiD = chiD;% - phiBeta;


edot = U*sin(psi - gammak);% + phiBeta); % U*sin(psi)*cos(gammak) - U*cos(psi)*sin(gammak);
%psiDdot = -edot * lookAheadDistance / (lookAheadDistance^2 + CTerr^2);

deltaC = -kpr*(psi-psiD) - kdr*r;
Vc = sqrt(U^2+beta^2);
if nextWpt == 4
     xC = (x_k+x_k_1)/2; % Center of a circle radius thing
     yC = (y_k+y_k_1)/2; % Center of a circle radius thing
     CTerr = sqrt((xM - xC)^2+(yM - yC)^2) - sqrt((x_k-xC)^2+(y_k-yC)^2);
     edot = (xM - xC + yM - yC)/sqrt((xM - xC)^2+(yM - yC)^2);
     gammak = atan2(yM-yC,xM-xC) + 3*pi/2;
     chiD = gammak + atan(-kp*CTerr - ki*IE);
     psiD = chiD;% - phiBeta;
     deltaC = -kpr*(psi-psiD) - kdr*r;
     %psi = gammak;
end
% Set derivatives
xdot(1) = U*cos(psi); %+beta*cos(phiBeta);% + beta*cos(phiBeta); % x dot
xdot(2) = U*sin(psi); %+beta*sin(phiBeta);% + beta*sin(phiBeta); % y dot
xdot(3) = r; % psi dot
xdot(4) = -k1*r + k2*delta; % r dot
xdot(5) = 1/Tdelta*(deltaC-delta); % delta dot
%xdot(6) = -kp*(r - psiDdot) - ki*(psi - psiD);
xdot(6) = CTerr; % integral
xdot(7) = edot; % e dot

xdot = xdot';
end