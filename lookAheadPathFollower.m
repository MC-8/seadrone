function [statesOut,success] = lookAheadPathFollower(x0,y0,wptList,acceptanceR,lookAheadDistance)
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

%clear wptX0 wptY0 wptX1 wptY1 alpha

wstart = 0;
wfinal = 500;
wout = wstart;
%x0out = x0;
%y0out = y0;
%e0out = e0;
i0 = 0; % integral term
statesOut = [x0 y0 e0 i0];
currentWpt = 1;
nextWpt = 2;

for i = 1:numWpt-1
    % Solve differential equations
    [w,states] = ode45(@LookaheadBasedSteering, [wstart wfinal], [x0,y0,e0,i0],[], wptList, currentWpt, nextWpt, lookAheadDistance);
    
    % Find the point when the craft entered in the acceptance radius
    distFromNextWaypoint = sqrt((wptList(nextWpt,1)-states(:,1)).^2+(wptList(nextWpt,2)-states(:,2)).^2);
    j = find(min(distFromNextWaypoint)==distFromNextWaypoint,1);
    
    % Append variables
    wstart=w(j);
    wfinal = wfinal+wstart;
    wout = [wout;w(2:j)];
    statesOut = [statesOut; states(2:j,:)];
    
    % Waypoint missed or not reached
    if (min(distFromNextWaypoint) > acceptanceR)
        fprintf('Waypoint %d missed or not reached\n',nextWpt)
        success = 0;
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
    i0 = states(j,4);
    % Plot new starting point
    %figure(1);
    %plot(x0,y0,'ro');
    
    % The lateral error is recalculated based on the new set of waypoints
    wptX0 = wptList(currentWpt,1);
    wptY0 = wptList(currentWpt,2);
    wptX1 = wptList(nextWpt,1);
    wptY1 = wptList(nextWpt,2);
    
    alpha = atan2(wptY1 - wptY0, wptX1 - wptX0);
    e0 = -(x0-wptX0)*sin(alpha)+(y0-wptY0)*cos(alpha);
    
end
%clear wptX0 wptY0 wptX1 wptY1 m q e0 wstart currentWpt i j e0 y0 x0 nextWpt distFromNextWaypoint nw

%figure;
%plot(statesOut(:,1),statesOut(:,2),'-.b','linewidth',3);
