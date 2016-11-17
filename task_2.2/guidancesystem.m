function [ psi_d ] = guidancesystem(wp, k, p )
%GUIDANCESYSTEM A LOS based guidance system

% Keep track of current waypoint

persistent k;

if isempty(k)
    k = 1; % Set active waypoint to first one.
end


% Acceptance circle radius
Lpp = 304.8;
R = 2*Lpp;

% Extract current waypoint and next.
xkn = WP(1,k+1); % next WP x-pos
ykn = WP(2,k+1); % next WP y-post

xk = WP(1,k); % Current WP x-pos
yk = WP(2, k); % Current WP y-pos

% Extract current position.
x = p(1);
y = p(2);

% Check if we can switch to next waypoint
if ( (xkn - x)^2 + (ykn - y)^2 <= R^2)
    k = k + 1;
    % Update waypoints
    xk = xkn;
    yk = ykn;
    
    xkn = WP(1,k+1);
    ykn = WP(2,k+1);
end

% Compute the path tangential angle
alpha_k = atan2( ykn - yk, xkn - xk );

% Compute the cross-track error
e = -(x - xk)*sin(alpha_k) + (y - yk)*cos(alpha_k);

% Compute the error gain
delta = sqrt( R^2 - e^2);
Kpe = 1/delta;

% Compute the desired heading
psi_d = alpha_k + atan(-K_pe*e);


end

