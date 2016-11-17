function [c_x, c_y] = circle(in)

[x,y,r] = deal(in(1),in(2),in(3));

w = linspace(0,2*pi);
c_x = x + r*cos(w);
c_y = y + r*sin(w);