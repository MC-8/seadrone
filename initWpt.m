global u v phiBeta Tr K k1 k2 kp ki kpr kdr Tdelta lookAheadDistance acceptanceR wptList

% Define data structures
wpt   = struct('pos',[],'vel',[],'acc',[],'time',[],'numWpt',[],'current',1);
craft = struct('pos',[],'vel',[],'acc',[],'time',[],'numWpt',[],'current',1);
paths = struct('pos',[],'vel',[],'acc',[],'time',[],'numWpt',[]);

% Define waypoints
wpt.pos.x = [0 1 1 4 4]*50;
wpt.pos.y = [0 1 4 4 1]*50;
wpt.mode =  [0 0 0 1 0];
% MODE 0: straight path, MODE 1: Semicircle path clock wise
% MODE 2: semicircle path counterclockwise

wpt.numWpt = length(wpt.pos.x);
paths.simple.pos.x = wpt.pos.x;
paths.simple.pos.y = wpt.pos.y;
paths.simple.numWpt = wpt.numWpt;
paths.simple.wptMode = wpt.mode;
numWpt = paths.simple.numWpt;

wptList = zeros(paths.simple.numWpt,2);
for i = 1 : length(wptList(:,1))
    wptList(i,1) = paths.simple.pos.x(i);
    wptList(i,2) = paths.simple.pos.y(i);
    wptList(i,3) = paths.simple.wptMode(i);
end;
currentWpt = 1;
nextWpt = 2;