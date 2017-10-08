% My comet generation
figure();
filename = 'pathEx.gif';
plot(trajX,trajY,':k','linewidth',2); hold on;  axis([0 100 -42 42]); grid on;
xlabel('x [m]'); ylabel('y [m]'); title('Craft trajectory along the path with look-ahead particle');
plot(50,-5,'*r', 'linewidth',20);
ellipse(15,15,0,50,-5,'r');
%%
for i=1:1:length(xMeas.signals.values)-1
    f = plot([xMeas.signals.values(i) xMeas.signals.values(i+1)],...
        [yMeas.signals.values(i) yMeas.signals.values(i+1)],'b','linewidth',2); grid on;
    h = plot([xMeas.signals.values(i) xVTP.signals.values(i)], ...
        [yMeas.signals.values(i) yVTP.signals.values(i)],'-dr','linewidth',2);
          drawnow
%       frame = getframe(1);
%       im = frame2im(frame);
%       [imind,cm] = rgb2ind(im,256);
%       if i == 1;
%           imwrite(imind,cm,filename,'gif', 'Loopcount',inf,'DelayTime',0.05);
%       else
%           imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',0.05);
%       end
%     %pause(0.05);
    delete(h);
end