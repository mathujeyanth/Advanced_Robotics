function draw_manipulator(q)
    fig(1)=figure;
    hold on
    pbaspect([3 1.5 1])
    plot([-2 2],[1 1],['--k'])
    p=directKin(q);
    for i = 1:length(p)
        draw_circle(0.01,p(i,:),'k');
    end
    for i=1:length(p)-1
        plot([p(i,1) p(i+1,1)],[p(i,2) p(i+1,2)],'-ok')
    end
    axis([-2 2 0 1.5]);
    box on
    hold off
end