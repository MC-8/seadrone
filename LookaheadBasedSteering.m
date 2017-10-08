function [xdot] = LookaheadBasedSteering(t, states , wptList, currentWpt, nextWpt, lookAheadDistance)

% Fossen book (p261)
% Function returns : xdot = [dx,dy,de]'

% Inputs:
% psi = heading angle we assume that the input is EXACTLY and INSTANTLY
% the craft heading


% State variables:
% x = position of craft on x axis
% y = position of craft on y axis
% e = min distance craft-track

U = 5;          % Forward speed


if isempty(lookAheadDistance)
    lookAheadDistance = 1;      % Look-ahead distance
end
if isempty(currentWpt)
    currentWpt = 1;
end
if isempty(nextWpt)
    nextWpt = 2;
end

% Get PI parameters
ki = 0.1/lookAheadDistance; % Integral action term
Kp = 1/lookAheadDistance;
% Get nearest waypoint
x_k = wptList(currentWpt,1);
x_k_1 = wptList(nextWpt,1);
y_k = wptList(currentWpt,2);
y_k_1 = wptList(nextWpt,2);

% Calculate heading angle
gammak = atan2( y_k_1 - y_k , x_k_1 - x_k );
e = states(3);
IE = states(4);
psi = gammak + atan(-e/lookAheadDistance -ki*IE);

% Set derivatives
xdot(1) = U*cos(psi);
xdot(2) = U*sin(psi);
xdot(3) = xdot(2)*cos(gammak)- xdot(1)*sin(gammak);
xdot(4) = e;

xdot = xdot';

end

