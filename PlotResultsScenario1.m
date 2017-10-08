clear all;
close all;
clc;
ih = 1;
h(ih) = figure(ih); ih = ih+1;
s1range = [0.1000 0.5000 0.8 0.4250];
s2range = [0.1000 0.1200 0.8 0.2500];

load scenario1_uF5ms_DeltaF25.mat;
plotPathAndRudder(xMeas,yMeas,deltaDeg);
load scenario1_uF5ms_DeltaF10.mat;
plotPathAndRudder(xMeas,yMeas,deltaDeg);
load scenario1_uF5ms_DeltaV25_CONS.mat;
plotPathAndRudder(xMeas,yMeas,deltaDeg);

subplot(211); plot(trajectory(:,1),trajectory(:,2),':k','linewidth',2);
legend('u = 5m/s, \Delta = 25m','u = 5m/s, \Delta = 10m','u = 5m/s, \Delta = 10~25m','path','location','southwest');
title('Fixed \Delta Vs Variable \Delta (conservative)');
%h(ih) = figure(ih); ih = ih+1;
% plot(Delta.time, Delta.signals.values);
% ylabel('\Delta');
% xlabel('time [s]');
%
set(subplot(211),'Position',s1range);
set(subplot(212),'Position',s2range);

h(ih) = figure(ih); ih = ih+1;
load scenario1_uF5ms_DeltaV25_CONS.mat;
plotPathAndRudder(xMeas,yMeas,deltaDeg);
load scenario1_uV5ms_DeltaV25_CONS.mat;
plotPathAndRudder(xMeas,yMeas,deltaDeg)
subplot(211); plot(trajectory(:,1),trajectory(:,2),':k','linewidth',2);
legend('u = 5m/s, \Delta = 10~25m','u = 2~10m/s, \Delta = 10~25m','path','location','southwest');

title('Fixed speed Variable \Delta (conservative) Vs Variable Speed Variable \Delta (conservative)');
set(subplot(211),'Position',s1range);
set(subplot(212),'Position',s2range);

h(ih) = figure(ih); ih = ih+1;
load scenario1_uF5ms_DeltaV25_CONS.mat;
plotPathAndRudder(xMeas,yMeas,deltaDeg);
h(ih) = figure(ih); ih = ih+1;
plot(Delta.time, Delta.signals.values,'linewidth',2);
ylabel('\Delta [m]');
xlabel('time [s]');
hold on;
figure(ih-2);
load scenario1_uF5ms_DeltaV25.mat;
plotPathAndRudder(xMeas,yMeas,deltaDeg);
subplot(211); plot(trajectory(:,1),trajectory(:,2),':k','linewidth',2);
legend('u = 5m/s, \Delta = 5~25m - CON','u = 5m/s, \Delta = 10~25m - REL','path','location','southwest');
title('Fixed speed Variable \Delta (conservative) Vs Fixed Speed Variable \Delta (relaxed)');
set(subplot(211),'Position',s1range);
set(subplot(212),'Position',s2range);

figure(ih-1);
plot(Delta.time, Delta.signals.values,'linewidth',2);
ylabel('\Delta [m]');
xlabel('time [s]');
hold on; grid on;
legend('\Delta (conservative)', '\Delta (relaxed)','location','southwest');
title('Look ahead distance comparison Conservative Vs Relaxed');

h(ih) = figure(ih); ih = ih+1;
load scenario1_uV5ms_DeltaV25_CONS.mat;
plotPathAndRudder(xMeas,yMeas,deltaDeg)
h(ih) = figure(ih); ih = ih+1;
plot(uMeas.time, uMeas.signals.values,'linewidth',2); hold on; grid on;
load scenario1_uV5ms_DeltaV25.mat;
figure(ih-2);
plotPathAndRudder(xMeas,yMeas,deltaDeg)
subplot(211); plot(trajectory(:,1),trajectory(:,2),':k','linewidth',2);
legend('u = 2-10m/s, \Delta = 5~25m - CON','u = 2-10m/s, \Delta = 10~25m - REL','path','location','southwest');
title('Variable speed Variable \Delta (conservative) Vs Variable Speed Variable \Delta (relaxed)');
set(subplot(211),'Position',s1range);
set(subplot(212),'Position',s2range);

figure(ih-1);
plot(uMeas.time, uMeas.signals.values,'linewidth',2);
xlabel('time [s]'); ylabel('craft speed [m/s]');
legend('u: CON','u: REL', 'location','southwest');
title('Craft speed, Conservative mode Vs Relaxed');


h(ih) = figure(ih); ih = ih+1;load scenario1_uF5ms_DeltaF25_LOS.mat;
plotPathAndRudder(xMeas,yMeas,deltaDeg);
load scenario1_uF5ms_DeltaF10_LOS.mat;
plotPathAndRudder(xMeas,yMeas,deltaDeg)
sbpl=subplot(211); plot(trajectory(:,1),trajectory(:,2),':k','linewidth',2);
legend('u = 5m/s, \Delta = 25m','u = 5m/s, \Delta = 10m','path','location','southwest');
title('Traditional LOS guidance law performance');
set(subplot(211),'Position',s1range);
set(subplot(212),'Position',s2range);
%%
for i = 1:length(h)
    h(i).Position = [50*i 500 640 480];
end
