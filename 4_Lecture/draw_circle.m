function h = draw_circle(r,c,colour)
    th = linspace(0,2*pi);
    x = r*sin(th)+c(1);
    y = r*cos(th)+c(2);
    h=fill(x,y,colour);
end