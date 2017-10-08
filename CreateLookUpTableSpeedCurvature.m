% Creation of lookup table for speed/curvature limitation
% It is based on Nomoto model simulations using the controller 
% CTRL = tf(num,den);
% num = [0.8 1.6 0.8]; den = [0.04 0.5 1];

speedRangeFullRud = [1:1:10];
speedRangeMidRud = 12.5;
speedRangeLimRud = [15:5:30];
curvatureCapabilityLowSpeed = 0.21./speedRangeFullRud;
curvatureCapabilityMidSpeed = 0.013;
curvatureCapabilityHighSpeed = 0.09./speedRangeLimRud;
craftSpeedCurvatureLookup = [0,1000;
                             speedRangeFullRud', curvatureCapabilityLowSpeed'; ...
                             speedRangeMidRud' , curvatureCapabilityMidSpeed'; ...
                             speedRangeLimRud' , curvatureCapabilityHighSpeed';
                             30, 0];
craftSpeedCurvatureLookup(:,2) =  craftSpeedCurvatureLookup(:,2)/3*1;
                         
%%
% plot(craftSpeedCurvatureLookup(2:end,2),craftSpeedCurvatureLookup(2:end,1),'linewidth',1.5);
% hold on; scatter(craftSpeedCurvatureLookup(2:end,2),craftSpeedCurvatureLookup(2:end,1),'linewidth',1.5);
% grid on;
% xlabel('Steady state curvature \kappa [m^{-1}]'); ylabel('Max admissible surge velocity U_{max} [m/s]');
% title('Maximum surge velocity Versus Max curvature (90 degrees turn)');