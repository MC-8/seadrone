function [alphak,crossTrackError] = getHeadingReference(xMeas, yMeas, xTraj, yTraj, lookAheadDistance)
% coder.varsize('xTraj', [1201 1], [1 0]);
% coder.varsize('yTraj', [1201 1], [1 0]);


% Calculate target angle
xT =  xTraj(end);
yT =  yTraj(end);
PsiWpts = atan2(yT-yMeas, xT-xMeas);

% Project a point ahead (the particle that follows the track)
pointAheadX = xMeas+lookAheadDistance*cos(PsiWpts);
pointAheadY = yMeas+lookAheadDistance*sin(PsiWpts);
nPoints = 1;
dist = 0;
ind  = 1;

% Get Error from particle to track
dx = xTraj - pointAheadX;
dy = yTraj - pointAheadY;
[~, indexClosestPoint] = min(hypot(dx, dy));
alphak = atan2(yTraj(indexClosestPoint)-yMeas, xTraj(indexClosestPoint)-xMeas);

% Get Error from craft to track
dx = xTraj - xMeas;
dy = yTraj - yMeas;
[dist, ind] = min(hypot(dx, dy));
% end


% Find out if the craft is on one side or the other
angle1 = atan2(yTraj(ind)-yTraj(1) , xTraj(ind)-xTraj(1));
angle2 = atan2(yMeas-yTraj(1) , xMeas-xTraj(1));

if (angle2<angle1) % Craft on one side of the curve
    crossTrackError = -dist;
else
    crossTrackError = dist;
end


end
