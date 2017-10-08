function [] = plotPathAndRudder(xMeas,yMeas,deltaDeg)

subplot(211)
plot(xMeas.signals.values(xMeas.signals.values<100), yMeas.signals.values(xMeas.signals.values<100), 'linewidth',2);
xlabel('x [m]'); ylabel('y [m]'); grid on; hold on;
axis([0 100 -20 25]);

subplot(212);
plot(deltaDeg.time(deltaDeg.time<max(xMeas.time(xMeas.signals.values<100))), deltaDeg.signals.values(deltaDeg.time<max(xMeas.time(xMeas.signals.values<100))), 'linewidth',2);
xlabel('time [s]'); ylabel('\delta_r [deg]'); grid on; hold on;
axis([0 max(xMeas.time) -15 15]);
