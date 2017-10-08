function k = getCurvature(myPathX, myPathY)

dx = diff(myPathX);
d2x = diff(dx);
dy = diff(myPathY);
d2y = diff(dy);
k = abs((dx(2:end).*d2y-dy(2:end).*d2x)./((dx(2:end).^2+dy(2:end).^2).^(1.5)));
