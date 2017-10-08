function [statesOut,success] = lookAheadPathFollowerD(x0,y0,psi0,wptList,acceptanceR,lookAheadDistance)
%% Lookahead-Based Steering ( Fossen 261 )
% wptList is in the form [x0,y0;x1,y1;..xn;yn]
% x0 y0 in the parameters are initial position of the craft

numWpt = length(wptList(:,1));
wptX0 = wptList(1,1);
wptY0 = wptList(1,2);
wptX1 = wptList(2,1);
wptY1 = wptList(2,2);

% Initial conditions for the ode solver are x0 (given) y0 (given) and e0

alpha = atan2(wptY1 - wptY0, wptX1 - wptX0);
e0 = -(x0-wptX0)*sin(alpha)+(y0-wptY0)*cos(alpha);
r0 = 0;
delta0 = 0;
Ierr = 0;

%clear wptX0 wptY0 wptX1 wptY1 alpha

wstart = 0;
wfinal = 150;
wout = wstart;
%x0out = x0;
%y0out = y0;
%e0out = e0;
statesOut = [x0 y0 psi0 r0 delta0 Ierr e0];
currentWpt = 1;
nextWpt = 2;
initialConditions = [x0 y0 psi0 r0 delta0 Ierr e0];
for i = 1:numWpt-1
    % Solve differential equations
    [w,states] = ode45(@LookaheadBasedSteeringWithDynamicsAndBigTurn, [wstart wfinal], initialConditions ,[], wptList, currentWpt, nextWpt, lookAheadDistance);
    
    % Find the point when the craft entered in the acceptance radius
    distFromNextWaypoint = sqrt((wptList(nextWpt,1)-states(:,1)).^2+(wptList(nextWpt,2)-states(:,2)).^2);
    j = find( distFromNextWaypoint' < acceptanceR,1);
    
    % Append variables
    wstart=w(j);
    wfinal = wfinal+wstart;
    wout = [wout  ;w(2:j)];
    statesOut = [statesOut ; states(2:j,:)];
    
    % Waypoint missed or not reached
    if (min(distFromNextWaypoint) > acceptanceR)
        fprintf('Waypoint %d missed or not reached',nextWpt)
        success = 0;
        statesOut = [statesOut ; states(2:end,:)];
        break
    end
    
    % Select the next waypoint
    currentWpt = currentWpt+1;
    nextWpt = nextWpt+1;
    if ( nextWpt > numWpt ) % All waypoints have been done
        success = 1;
        break
    end
    
    % Set new initial conditions
    x0 = states(j,1);
    y0 = states(j,2);
    psi0 = states(j,3);
    r0 = states(j,4);
    delta0 = states(j,5);
    Ierr = states(j,6);
    % deltaC0 = states(j,6);
    
    % The lateral error is recalculated based on the new set of waypoints
    wptX0 = wptList(currentWpt,1);
    wptY0 = wptList(currentWpt,2);
    wptX1 = wptList(nextWpt,1);
    wptY1 = wptList(nextWpt,2);
    
    alpha = atan2(wptY1 - wptY0, wptX1 - wptX0);
    if nextWpt == 4
        xC = (wptX0+wptX1)/2; % Center of a circle radius thing
        yC = (wptY0+wptY1)/2; % Center of a circle radius thing
        e0 = - sqrt((x0 - xC)^2+(y0 - yC)^2) + sqrt((wptX0-xC)^2+(wptY0-yC)^2);

    else
        e0 = -(x0-wptX0)*sin(alpha)+(y0-wptY0)*cos(alpha);
    end
    initialConditions = [x0 y0 psi0 r0 delta0 Ierr e0];
    
end
%clear wptX0 wptY0 wptX1 wptY1 m q e0 wstart currentWpt i j e0 y0 x0 nextWpt distFromNextWaypoint nw

%figure;
%plot(statesOut(:,1),statesOut(:,2),'-.b','linewidth',3);
