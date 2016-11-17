% Path generation
WP = importdata('WP.mat');

N = size(WP,2);
wpt.pos.y = WP(1,:);
wpt.pos.x = WP(2,:);

wpt.time = [0 200 400 600 800];
t = 0:800;

% Path generation using hermitian interpolation
yp = pchip(wpt.time,wpt.pos.y,t);
xp = pchip(wpt.time,wpt.pos.x,t);


% Path generation using straight lines and circles
dev_max = [Inf 200 500 200 Inf];                                     % Maximum allowed deviation from point
lp_max_dev = @(dev,alpha) dev./(sqrt(1+tan(alpha).^2)-tan(alpha));     % 
c_p = zeros(3,N-2);

for k = 2:N-1,
    p1 = [wpt.pos.x(k-1)-wpt.pos.x(k) wpt.pos.y(k-1)-wpt.pos.y(k)];
    p2 = [wpt.pos.x(k+1)-wpt.pos.x(k) wpt.pos.y(k+1)-wpt.pos.y(k)];
    
    lp1 = ((wpt.pos.x(k-1)-wpt.pos.x(k)).^2+(wpt.pos.y(k-1)-wpt.pos.y(k)).^2).^0.5;
    lp2 = ((wpt.pos.x(k+1)-wpt.pos.x(k)).^2+(wpt.pos.y(k+1)-wpt.pos.y(k)).^2).^0.5;
    
    lp_max = 0.5*min(lp1,lp2);
    alpha = 0.5*angle([wpt.pos.x(k-1)-wpt.pos.x(k) wpt.pos.y(k-1)-wpt.pos.y(k)],[wpt.pos.x(k+1)-wpt.pos.x(k) wpt.pos.y(k+1)-wpt.pos.y(k)]);
    
    R = min(lp_max,lp_max_dev(dev_max(1,k),alpha));
    
    R_bar = R*tan(alpha);
    
    x = R*(sqrt(1+tan(alpha).^2)-tan(alpha));
    
    c_p(:,k-1) = [[wpt.pos.x(k) wpt.pos.y(k)]+((x+R_bar)*(1/norm(p1/lp1+p2/lp2))*(p1/lp1+p2/lp2)) R_bar]';
end


figure('Name','Spline interpolation');
plot(wpt.pos.x,wpt.pos.y,'-o',xp,yp); grid on; hold on;
for k = 2:N-1;
    [c1,c2] = circle(c_p(:,k-1));
    plot(c1,c2);
end
ylabel('N');xlabel('E');
legend('Waypoints','Hermitian interpolation','location','northeastoutside');
axis equal;