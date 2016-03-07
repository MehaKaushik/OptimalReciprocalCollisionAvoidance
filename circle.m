function  circle(x,y,r)
%function for drawing circle

th = 0:pi/50:2*pi;
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;
plot(xunit, yunit);
axis([-10,10,-10,10]);

