function [xM,yM,u_f,v_f,un,vn] = generateVectorField(xmin,xmax,ymin,ymax,refineF,xTarget,yTarget,obstacleList)

d = obstacleList;
coder.varsize('xM', [3001 3001], [1 1]);
coder.varsize('yM', [3001 3001], [1 1]);
coder.varsize('u_f', [3001 3001], [1 1]);
coder.varsize('v_f', [3001 3001], [1 1]);
coder.varsize('un', [3001 3001], [1 1]);
coder.varsize('vn', [3001 3001], [1 1]);
% xC, yC, strength, angle, a_ellipse, b_ellipse
%d(:,3) = 1;
%obstacleList = d;

%     for i  = 1:length(d(:,1));
%         muFromSinkToDoublet(i,1) = 1./hypot(d(i,1)-s(1,1),d(i,2)-s(1,2));
%     end

% Check that obstacles are not too close, in case create a new bigger obstacle
%initParameters;
%d = obstacleList;
% j = 1;
% coder.varsize('newObst', [9 6]);
% coder.varsize('d', [9 6]);
% 
% newObst = zeros(1,6);
% for i = 1:length(d(:,1))
%     for k = 2:length(d(:,1))
%         distObst = sqrt((d(i,1)-d(k,1))^2+(d(i,2)-d(k,2))^2);
%         if (distObst < 15) && (i~=k)
%             newObst = [newObst;(d(i,1)+d(k,1))/2,(d(i,2)+d(k,2))/2,1,pi,15,15];
%             obstacleList(k,:)=[0,0,0,0,0,0];
%             d(k,:)=[0,0,0,0,0,0];
%             obstacleList(i,:)=[0,0,0,0,0,0];
%             j = j+1;
%         end
%     end
% end
% d=[d;newObst];
% d(all(d==0,2),:)=[];

[xM, yM] = meshgrid(xmin:refineF:xmax,ymin:refineF:ymax);
u_f = zeros(length(xM(:,1)),length(xM(1,:)));
v_f = zeros(length(yM(:,1)),length(yM(1,:)));

for i = 1:length(d(:,1))
    u = 0;
    v = 0;
    xr = xM - d(i,1);
    yr = yM - d(i,2);
    mu =  d(i,3);%1;%250./hypot(d(i,1)-s(1,1),d(i,2)-s(1,2));
    phi = d(i,4);
    a =  d(i,5);
    b =  d(i,6);
    
    % THIS IS GOOD in practice, althought the analytical equation is not
    % correct. It works very well with mu = 1 and, a and b give the shape
    % of the ellipse
      u = u -  mu.*((yr./b).^2-(xr./a).^2)./(((yr./b).^2+(xr./a).^2).^2);
      v = v + mu.*2.*(xr./a^2).*yr./(((yr./b).^2+(xr./a).^2).^2);

    normalizer = sqrt(u.^2+v.^2);
    
    un = u./normalizer;
    vn = v./normalizer;
    ur = un*cos(phi) - vn*sin(phi);
    vr = un*sin(phi) + vn*cos(phi);
    u = ur.*normalizer;
    v = vr.*normalizer;
    
    u_f = u_f+u;
    v_f = v_f+v;
    
end;

%figure;
%normalizer = sqrt(u_f.^2+v_f.^2);
%quiver(xM,yM,u_f./normalizer,v_f./normalizer);



    xC = xTarget;
    yC = yTarget;
    mu = -1;
    r = sqrt((xM-xC).^2+(yM-yC).^2);
    Vr = mu;%mu./(2*pi.*r);
    u_f = u_f + Vr.*(xM-xC)./r;
    v_f = v_f + Vr.*(yM-yC)./r;


normalizer = sqrt(u_f.^2+v_f.^2);
un = u_f./normalizer;
vn = v_f./normalizer;

%alpha = pi/4;
% Add additional flow
%CTerr = -(xM - 0)*sin(alpha) + (yM - 0)*cos(alpha);
%surf(u,v,'linestyle','none');

% % u = u + sin(alpha).*CTerr.*sign(CTerr);
% % v = v - cos(alpha).*CTerr.*sign(CTerr);
% surf(-1./sqrt(u.^2+v.^2))