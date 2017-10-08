%% Main script

% Massimiliano Curzi s120949@student.dtu.dk
% 24-11-2014: File created

clear all;
close all;
clc;
%
initParameters;


% Define other variables


% Define Radii for Dubins paths
numCirc = wpt.numWpt - 2;
radii = 10*ones(numCirc,1);

% Initial position of the craft
craft.pos.x = 0;
craft.pos.y = 0;
craft.heading = 0;
x0 = craft.pos.x;
y0 = craft.pos.y;
Psi0 = craft.heading;
% Plot Waypoints
%h1 = plot(wpt.pos.x,wpt.pos.y,'ro');
%f1 = gcf;
%p = get(h);
labels = cell(wpt.numWpt,1);
for i = 1 : wpt.numWpt
    labels{i} = cellstr(strcat('P_',num2str(i) ));
end;
% %labels = cellstr( strcat('(',num2str(wpt.x),',',num2str(wpt.y),')' ));
% text(wpt.pos.x, wpt.pos.y, labels, 'VerticalAlignment','bottom', ...
%     'HorizontalAlignment','right')
% minX = min(wpt.pos.x);
% maxX = max(wpt.pos.x);
% minY = min(wpt.pos.y);
% maxY = max(wpt.pos.y);
% axis ([0.9*minX - 0.1, 1.1*maxX + 0.1,0.9*minY - 0.1,1.1*maxY]);
% clear minX maxX minY maxY
% axis equal
% xlabel('x [m]');
% ylabel('y [m]');
% grid on;
% title('Waypoints');
% hold on;
% 
% % %f2=copyobj(f1,0);
% %plot(wpt.pos.x,wpt.pos.y);
% title('Waypoints and straight lines path');

%legend('Waypoints','Trajectory by order of Waypoints','location','best');

% Dubins path generation ( Fossen 255 - 257 )
dubCirc = struct('radius',radii,...
    'xC',zeros(numCirc,1),...
    'yC', zeros(numCirc,1),...
    'alpha',zeros(numCirc,1),...
    'numCirc',numCirc);
newWpt = wpt;
newWpt.numWpt = newWpt.numWpt+numCirc;
k = 2;
for i = 1 : 1 : wpt.numWpt - 2                               % Start and Ending wpt have no angles
    vect1 = [wpt.pos.x(i+1),wpt.pos.y(i+1)]-[wpt.pos.x(i),wpt.pos.y(i)];     % from 1 to 2
    vect2 = [wpt.pos.x(i+2),wpt.pos.y(i+2)]-[wpt.pos.x(i+1),wpt.pos.y(i+1)]; % from 2 to 3
    vect3 = [wpt.pos.x(i+2),wpt.pos.y(i+2)] -[wpt.pos.x(i),wpt.pos.y(i)];    % from 1 to 3
    b = norm(vect1);                                         % distance
    c = norm(vect2);                                         % distance
    a = norm(vect3);                                         % distance
    dubCirc.alpha(i) = 0.5*acos((-a^2+b^2+c^2)/(2*b*c));
    dubinsTurningDistance = dubCirc.radius(i)/tan(dubCirc.alpha(i));    % Distance from waypoint where start turning
    
    intersection1 = [wpt.pos.x(i+1) wpt.pos.y(i+1)]-vect1/norm(vect1)*dubinsTurningDistance; % it's the turning point
    intersection2 = [wpt.pos.x(i+1) wpt.pos.y(i+1)]+vect2/norm(vect2)*dubinsTurningDistance; % it's from circle to line
    
    % Save the intersections as new waypoints
    newWpt.pos.x(k) = intersection1(1);
    newWpt.pos.y(k) = intersection1(2);
    k = k+1;
    newWpt.pos.x(k) = intersection2(1);
    newWpt.pos.y(k) = intersection2(2);
    k = k+1;
    
    %plot(intersection1(1),intersection1(2),'rx')
    %plot(intersection2(1),intersection2(2),'rx')
    
    % To obtain the center of the radius get the perpendicular from
    % turning point, and it will be distant as the radius.
    
    perpV1 = vect1*[0 -1 ; 1 0]/norm(vect1);
    if dot(vect2,perpV1)<0 % If perpendicular segment is pointed outwards
        perpV1 = -perpV1;  % take the segment on the other side
    end
    segHyp = perpV1*dubCirc.radius(i);
    centerR = intersection1+segHyp; % Center coordinates
    dubCirc.xC(i) = centerR(1);
    dubCirc.yC(i) = centerR(2);
    %plot( dubCirc.xC(i),dubCirc.yC(i),'mx');
    %circles(dubCirc.xC(i),dubCirc.yC(i),dubCirc.radius(i),'facecolor','none','edgecolor','cyan');
end

% And add the very last way point
newWpt.pos.x(k) = paths.simple.pos.x(end);
newWpt.pos.y(k) = paths.simple.pos.y(end);

clear vect1 vect2 vect3 a b c dubinsTurningDistance perpV1 intersection1 intersection2 centerR segHyp i k

% f3=copyobj(f2,0);
% plot( dubCirc.xC,dubCirc.yC,'mx');
% circles(dubCirc.xC,dubCirc.yC,dubCirc.radius,'facecolor','none','edgecolor','cyan');
% plot(newWpt.pos.x,newWpt.pos.y,'rx');
% 
% labels = cell(dubCirc.numCirc,1);
% for i = 1 : dubCirc.numCirc
%     labels{i} = cellstr(strcat('C_',num2str(i)));
% end;
% text(dubCirc.xC, dubCirc.yC, labels, 'VerticalAlignment','top', ...
%     'HorizontalAlignment','left');
% title('Straight lines and inscribed circles - Dubins Path');


paths.dubins.pos.x = newWpt.pos.x;
paths.dubins.pos.y = newWpt.pos.y;
paths.dubins.numWpt = newWpt.numWpt;
clear newWpt

wptListD = zeros(paths.simple.numWpt,2);
for i = 1 : length(wptListD(:,1))
    wptListD(i,1) = paths.simple.pos.x(i);
    wptListD(i,2) = paths.simple.pos.y(i);
    wptListD(i,3) = paths.simple.wptMode(i);
end;
currentWpt = 1;
nextWpt = 2;


%% Cubic and Hermite interpolations  ( Fossen 267 - 272 )
wpt.omega = [0 10 30 50 70];  % Omega is the path variable

w = 0:0.1:max(wpt.omega);
x_s = spline(wpt.omega,wpt.pos.x,w);
y_s = spline(wpt.omega,wpt.pos.y,w);

w = 0:0.1:max(wpt.omega);
x_p = pchip(wpt.omega,wpt.pos.x,w);
y_p = pchip(wpt.omega,wpt.pos.y,w);

fint1 = figure;
s = subplot(311); h = plot(wpt.omega,wpt.pos.x,'bo',w,x_s,'r',w,x_p,'b'); xlabel('\omega'); ylabel('x(\omega)');
title('Interpolated trajectories');
legend('Waypoints','Spline','Hermite','Location','best');
s = subplot(312); h = plot(wpt.omega,wpt.pos.y,'bo',w,y_s,'r',w,y_p,'b'); xlabel('\omega'); ylabel('y(\omega)');
s = subplot(313); h = plot(wpt.pos.x,wpt.pos.y,'bo',x_s,y_s,'r',x_p,y_p,'b'); xlabel('x(\omega)'); ylabel('y(\omega)');

% figure(f);
% plot(wpt.pos.x,wpt.pos.y,'o',x_s,y_s); xlabel('x(\omega)'); ylabel('y(\omega)');

paths.spline.pos.x = x_s;
paths.spline.pos.y = y_s;
paths.spline.omega = w;
dx = diff(paths.spline.pos.x);
d2x = diff(dx);
dy = diff(paths.spline.pos.y);
d2y = diff(dy);
paths.spline.curvature = abs((dx(2:end).*d2y-dy(2:end).*d2x)./((dx(2:end).^2+dy(2:end).^2).^(1.5)));


paths.pchip.pos.x = x_p;
paths.pchip.pos.y = y_p;
paths.pchip.omega = w;
dx = diff(paths.pchip.pos.x);
d2x = diff(dx);
dy = diff(paths.pchip.pos.y);
d2y = diff(dy);
paths.pchip.curvature = abs((dx(2:end).*d2y-dy(2:end).*d2x)./((dx(2:end).^2+dy(2:end).^2).^(1.5)));

fint2 = figure;
s = subplot(211); h = plot(paths.spline.omega(3:end),paths.spline.curvature); xlabel('\omega'); ylabel('\kappa(\omega) spline ');
title('Curvature'); hold on;
s = subplot(212); h = plot(paths.pchip.omega(3:end),paths.pchip.curvature); xlabel('\omega'); ylabel('\kappa(\omega) Hermit');



%% Path following

% Generate a waypoint matrix (easier to pass to the function)
% List For Simple path
wptListSimple = zeros(paths.simple.numWpt,2);
for i = 1 : length(wptListSimple(:,1))
    wptListSimple(i,1) = paths.simple.pos.x(i);
    wptListSimple(i,2) = paths.simple.pos.y(i);
end;
%numWptSimple = paths.simple.numWpt;
[xOut,success] = lookAheadPathFollower(craft.pos.x,craft.pos.y,wptListSimple,acceptanceR,lookAheadDistance);
figure(f2);
plot(xOut(:,1),xOut(:,2),'--r','linewidth',2); axis auto
if (success)
    fprintf('Simple path following: ok\n');
end
f4 = figure;
plot(xOut(:,3)); grid on;
title('Cross Track Error - Simple path following'); xlabel('Time [s]'); ylabel('e(t) [m]');
% ListFor dubins
wptListDubins = zeros(paths.dubins.numWpt,2);
for i = 1 : length(wptListDubins(:,1))
    wptListDubins(i,1) = paths.dubins.pos.x(i);
    wptListDubins(i,2) = paths.dubins.pos.y(i);
end;
[xOut,success] = lookAheadPathFollower(craft.pos.x,craft.pos.y,wptListDubins,acceptanceR,lookAheadDistance);
if (success)
    fprintf('Dubins path following: ok\n');
end
figure(f3);
plot(xOut(:,1),xOut(:,2),'--r','linewidth',2); axis auto

f5 = figure;
plot(xOut(:,3)); grid on;
title('Cross Track Error - Dubins path following'); xlabel('Time [s]'); ylabel('e(t) [m]');
%numWptDubins = paths.dubins.numWpt;


%% Path following with dynamics Simple


% Generate a waypoint matrix (easier to pass to the function)
% List For Simple path
wptListSimple = zeros(paths.simple.numWpt,2);
for i = 1 : length(wptListSimple(:,1))
    wptListSimple(i,1) = paths.simple.pos.x(i);
    wptListSimple(i,2) = paths.simple.pos.y(i);
end;
%numWptSimple = paths.simple.numWpt;WithDynamics
[xOut,success] = lookAheadPathFollowerD(craft.pos.x,craft.pos.y,craft.heading,wptListSimple,acceptanceR,lookAheadDistance);
figure(f2);

plot(xOut(:,1),xOut(:,2),'-.m','linewidth',2); axis auto ;
pause(0.01);

if (success)
    fprintf('Simple path following Dyn: ok\n');
end
f6 = figure;
plot(xOut(:,7)); grid on;
title('Cross Track Error - Simple path following'); xlabel('Time [s]'); ylabel('e(t) [m]');
f7 = figure;
subplot(221); plot(xOut(:,3)); grid on;
title('Craft Heading'); xlabel('Time [s]'); ylabel('\Psi(t) [rad]'); xlim([0, length(xOut)]);
subplot(222); plot(xOut(:,4)); grid on;
title('Turning rate'); xlabel('Time [s]'); ylabel('r(t) [rad\\s]');xlim([0, length(xOut)]);
subplot(223); plot(xOut(:,5)); grid on;
title('Nozzle Angle'); xlabel('Time [s]'); ylabel('\delta(t) [rad]');xlim([0, length(xOut)]);
subplot(224); plot(xOut(:,7)); grid on;
title('Cross Track Error'); xlabel('Time [s]'); ylabel('e(t) [m]');xlim([0, length(xOut)]);


%% Dubin
% ListFor dubins
wptListDubins = zeros(paths.dubins.numWpt,2);
for i = 1 : length(wptListDubins(:,1))
    wptListDubins(i,1) = paths.dubins.pos.x(i);
    wptListDubins(i,2) = paths.dubins.pos.y(i);
end;
[xOut,success] = lookAheadPathFollowerD(craft.pos.x,craft.pos.y,craft.heading,wptListDubins,acceptanceR,lookAheadDistance);
if (success)
    fprintf('Dubins path following Dyn: ok\n');
end
figure(f3);
plot(xOut(:,1),xOut(:,2),'--b','linewidth',2); axis auto
title('Path Following Controller w/ Nomoto model and PI + PD controller');
f5 = figure;
plot(xOut(:,7)); grid on;
title('Cross Track Error - Dubins path following'); xlabel('Time [s]'); ylabel('e(t) [m]');
%numWptDubins = paths.dubins.numWpt;


f8 = figure;

%%
% f8 = figure;
% subplot(221); plot(xOut(:,3)); grid on;
% title('Craft Heading'); xlabel('Time [s]'); ylabel('\Psi(t) [rad]'); xlim([0, length(xOut)]);
% subplot(222); plot(xOut(:,4)); grid on;
% title('Turning rate'); xlabel('Time [s]'); ylabel('r(t) [rad\\s]');xlim([0, length(xOut)]);
% subplot(223); plot(xOut(:,5)); grid on;
% title('Nozzle Angle'); xlabel('Time [s]'); ylabel('\delta(t) [rad]');xlim([0, length(xOut)]);
% subplot(224); plot(xOut(:,7)); grid on;
% title('Cross Track Error'); xlabel('Time [s]'); ylabel('e(t) [m]');xlim([0, length(xOut)]);

%% Plot Simulation Results

k = 1;
for i = 1:length(nextWaypointTime.time) - 1
    diff = (nextWaypointTime.signals.values(i) - nextWaypointTime.signals.values(i+1));
    if diff
        transitionTime(k) = nextWaypointTime.time(i);
        k = k+1;
    end
end
f9 = figure;

subplot(221); plot(xOut.time,xOut.signals(3).values); grid on; hold on
ax = gca;
line([transitionTime;transitionTime],ax.YLim,'Color','r');
title('Craft Heading'); xlabel('Time [s]'); ylabel('\Psi(t) [rad]'); xlim([0, max(xOut.time)]);

subplot(222); plot(xOut.time,xOut.signals(4).values); grid on; hold on;
ax = gca;
line([transitionTime;transitionTime],ax.YLim,'Color','r');
title('Turning rate'); xlabel('Time [s]'); ylabel('r(t) [rad\\s]');xlim([0, max(xOut.time)]);

subplot(223); plot(xOut.time,xOut.signals(5).values); grid on; hold on;
ax = gca; line([transitionTime;transitionTime],ax.YLim,'Color','r');
title('Nozzle Angle'); xlabel('Time [s]'); ylabel('\delta(t) [rad]');xlim([0, max(xOut.time)]);

subplot(224); plot(CTE.time,CTE.signals.values); grid on; hold on;
ax = gca; line([transitionTime;transitionTime],ax.YLim,'Color','r');
title('Cross Track Error'); xlabel('Time [s]'); ylabel('e(t) [m]');xlim([0, max(xOut.time)]);
%%
for i = 1:20

end
%%

%Better waypoint plot
f10 = figure;
plot(wptList(wptList(:,3)==0,1),wptList(wptList(:,3)==0,2),'bo'); hold on; % Normal Waypoints
plot(wptList(wptList(:,3)==1,1),wptList(wptList(:,3)==1,2),'bo');          % Mode 1 Waypoints
%plot(xOut.signals(1).values,xOut.signals(2).values,'b','linewidth',2); grid on;
circles(wptList(:,1),wptList(:,2),acceptanceR,'facecolor','none','edgecolor','cyan');


k = 1;
for i = 1:length(wptList)
    if wptList(i,3) > 0
        xCs(k) = (wptList(i,1) + wptList(i-1,1))/2;
        yCs(k) = (wptList(i,2) + wptList(i-1,2))/2;
        radCs(k) = sqrt((wptList(i,1) - wptList(i-1,1))^2+(wptList(i,2) - wptList(i-1,2))^2)/2;
        k = k+1;
    end
end;
plot(xCs, yCs,'r*');          % Mode 1 Waypoints

k = 1;
for i = 1:length(wptList) - 1
    if ((wptList(i,3) == 0) && (wptList(i+1,3) == 0))||((wptList(i,3) > 0) && (wptList(i+1,3) == 0))
        plot([wptList(i,1) wptList(i+1,1)],[wptList(i,2) wptList(i+1,2)],'--r','linewidth',1);
    end
end;

circles(xCs,yCs,radCs,'facecolor','none','edgecolor','red','linewidth',1,'linestyle','--');
grid on;
xlabel('X [m]');
ylabel('Y [m]');

% Vector Field path generation
x_k = wptList(1,1);
x_k_1 = wptList(2,1);
y_k = wptList(1,2);
y_k_1 = wptList(2,2);
% Simple case, straight line
alphak = atan2( y_k_1 - y_k , x_k_1 - x_k );
xmax = max(xOut.signals(1).values)*1.2;
ymax = max(xOut.signals(2).values)*1.2;
xmin = min(xOut.signals(1).values)-50;
ymin = min(xOut.signals(2).values)-50;
[xM,yM] = meshgrid(-50+wptList(1,1):20:xmax,-50+wptList(1,2):20:ymax);

%figure; hold on;

%%
axis([xmin,xmax,ymin,ymax]);
RealCTerr = CTE.signals(1).values;
%Psi = xOut.signals(1).values;
%dx = u*cos(Psi);
flagWpt = 1;
nextWpt_1 = 1;
q = [];
head = [];
tail = [];
for i = 2:length(xOut.time)
    nextWpt = nextWaypointTime.signals(1).values(i);
    % Plot trajectory of craft
    if i>2
        tail = plot([xOut.signals(1).values(i-2),xOut.signals(1).values(i-1)],[xOut.signals(2).values(i-2),xOut.signals(2).values(i-1)],'k','linewidth',2);
        delete(head);
    end
    head = plot([xOut.signals(1).values(i)],[xOut.signals(2).values(i)],'.r','linewidth',3,'markersize',10);
    
    hold on;
    if (nextWpt~=nextWpt_1) % New waypoint reached! Replot vector field for new waypoint
        for k = 1:3
            delete(head);
            %pause (0.2);
            head = plot([xOut.signals(1).values(i-1),xOut.signals(1).values(i)],[xOut.signals(2).values(i-1),xOut.signals(2).values(i)],'.g','linewidth',4,'markersize',20);
            %pause (0.2);
        end

        if nextWpt < 1
            break
        end;
        wptmode = wptList(nextWpt,3);
        x_k = wptList(nextWpt-1,1);
        x_k_1 = wptList(nextWpt,1);
        y_k = wptList(nextWpt-1,2);
        y_k_1 = wptList(nextWpt,2);
        switch wptmode
            
            case 1 % Constant turning mode
                xC = (x_k+x_k_1)/2; % Center of a circle radius
                yC = (y_k+y_k_1)/2; % Center of a circle radius
                CTerr = sqrt((xM - xC).^2+(yM - yC).^2) - sqrt((x_k-xC).^2+(y_k-yC).^2);
                alphak = atan2(yC - yM, xC -xM) + pi/2;
                %alphak = alphak + 2*pi*turns;
                
            case 2 % Constant turning mode
                xC = (x_k+x_k_1)/2; % Center of a circle radius
                yC = (y_k+y_k_1)/2; % Center of a circle radius
                CTerr = -sqrt((xM - xC).^2+(yM - yC).^2) + sqrt((x_k-xC).^2+(y_k-yC).^2);
                alphak = atan2(yC - yM, xC -xM) - pi/2;
                %alphak = alphak + 2*pi*turns;
                
            otherwise % Straight line mode
                
                % Calculate track angle
                alphak = atan2( y_k_1 - y_k , x_k_1 - x_k );
                %alphak = alphak + 2*pi*turns;
                CTerr = -(xM - x_k).*sin(alphak) + (yM - y_k).*cos(alphak);
                
        end
        psi = alphak + atan(-CTerr/lookAheadDistance);
        dx = u_craft*cos(psi);
        dy = u_craft*sin(psi);
        %delete(q);
        %subplot(2,2,nextWpt-1);
        %q = quiver(xM,yM,dx./sqrt(dx.^2+dy.^2),dy./sqrt(dx.^2+dy.^2),.5,'b','linewidth',1,'MarkerSize',10);
        axis([xmin,xmax,ymin,ymax]);
        nextWpt_1 = nextWpt;
       % pause;
       hold on;
        plot(wptList(:,1),wptList(:,2),'ro'); hold on; % Normal Waypoints
        plot(wptList(nextWpt,1),wptList(nextWpt,2),'rx'); hold on; % Normal Waypoints

        
    end;
  %  pause(0.01);
    %f=getframe;
    %im(:,:,1,i) = rgb2ind(f.cdata,50,'nodither');
end

%imwrite(im,map,'Test2.gif','DelayTime',0,'LoopCount',inf)