function [trajectoryX,trajectoryY,xM,yM,un,vn] = getTrajectory(x0,y0,xTarget,yTarget, obstacles)

   xmin = 0; %min(x0,xTarget);
   ymin = -200; %min(y0,yTarget);
   xmax = 1000; %max(x0,xTarget);
   ymax = 200; %max(y0,yTarget);
   refineFactor = 2; % Grid of 2m x 2m
  
   [xM,yM,u_f,v_f,un,vn] = generateVectorField(xmin,xmax,ymin,ymax,refineFactor,xTarget,yTarget,obstacles);
    trajectory = mmstream2arr(xmin:refineFactor:xmax,ymin:refineFactor:ymax,u_f,v_f,x0,y0);
    trajectory( ~any(trajectory,2), : ) = [];  %rows
    
    trajectoryX = trajectory(:,1);
    trajectoryY = trajectory(:,2);
    

