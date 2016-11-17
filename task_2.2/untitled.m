

hold on
plot(t, rad2deg(psi_tilde))
plot(t, rad2deg(psi_r));
plot(t, rad2deg(psi_d));
plot(t, rad2deg(psi));
plot(t, rad
legend('\psi_t','\psi_r', '\psi_d', 'psi');

hold off



% % Keep track of current waypoint
% 
% % Acceptance circle radius
% Lpp = 304.8;
% R = 3*Lpp;
% 
%     k = 1; % Set next waypoint to first one. Current is init pos
% 
%    
% % Extract current waypoint and next.
% xn = WP(1,k+1); % next WP x-pos
% yn = WP(2,k+1); % next WP y-post
% 
% xk = WP(1,k); % Current WP x-pos
% yk = WP(2,k); % Current WP y-pos
% 
% % Extract current position.
% x = 1500;
% y = 500;
% 
% % Check if we can switch to next waypoint
% if ( ((xn - x)^2 + (yn - y)^2 <= R^2) && k < length(WP))
%     k = k + 1;
%     % Update waypoints
%     xk = xn;
%     yk = yn;
%     
%     xn = WP(1,k+1);
%     yn = WP(2,k+1);
% end
% 
% % Compute the path tangential angle
% alpha = atan2( yn - yk, xn - xk );
% 
% Rp = [cos(alpha) -sin(alpha);
%       sin(alpha)  cos(alpha)];
%   
% % Distance from craft to current waypoint
% p = [x y]' - WP(:,k);
% 
% % Position of craft in path-fixed ref frame
% eps = Rp'*p;
% 
% s = eps(1);
% e = eps(2);
% 
% % Compute the inverse error gain
% delta = sqrt(max(0, R^2 -e^2));
% 
% % Compute the desired heading
% psi_d = alpha + atan2(-e, delta); % Will be in range  -pi, pi
% 
% % Scale psi_d based on current heading, heading error always in(-pi,pi)
% % psi_d = psi_d - sign(psi_d -psi)*2*pi*floor(abs(psi)/(2*pi));
% 
% NED_psi = psi - sign(psi)*pi*floor(abs(psi)/pi); % in range -pi, pi
% 
% % Find the most correctedness angular difference between current heading, and desired.
% psi_e = psi_d - NED_psi;
% if psi_e > pi 
%     psi_e = psi_e - 2*pi;
% elseif psi_e < -pi
%     psi_e = psi_e + 2*pi;
% end
% 
% % Find correct way to turn.
% psi_d = psi + psi_e;
%     
% 
% 
% 