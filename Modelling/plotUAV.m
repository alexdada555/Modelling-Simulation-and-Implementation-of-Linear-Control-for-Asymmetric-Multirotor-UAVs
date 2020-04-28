theta = linspace(0,2*pi);
% xc = cos(theta);
% yc = -sin(theta);
% plot(xc,yc);
figure(1)
axis equal
xt = [-1 -1 1 1];
yt = [0 -0.2 -0.2 0];
hold on
t = area(xt,yt); % initial flat triangle
hold on
p = plot(0,0,'o','MarkerFaceColor','black');
hold off
for j = 1:length(theta)
    xt(1) = -1*cos(theta(j));
    xt(2) = -1*cos(theta(j))-(0.2*sin(theta(j)));
    xt(3) = 1*cos(theta(j))-(0.2*sin(theta(j))); % determine new vertex value
    xt(4) = 1*cos(theta(j));
    
    yt(1) = 1*sin(theta(j));
    yt(2) = 1*sin(theta(j))-(0.2*cos(theta(j)));
    yt(3) = -1*sin(theta(j))-(0.2*cos(theta(j))) ;
    yt(4) = -1*sin(theta(j));
    t.XData = xt; % update data properties
    t.YData = yt;
    t.BASEVALUE = yt(4);
    drawnow limitrate % display updates
    pause(0.1);
end