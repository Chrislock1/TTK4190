function desired_heading = guidance_heading(x,waypoints) 
% Calculates the desired heading for the vessel. x is the state vector

% Extraction
vessel.x = x(1);
vessel.y = x(2);
vessel.yaw = x(3);
vessel.heading = x(4);
vessel.surge = x(5);
vessel.sway = x(6);
vessel.yaw_rate = x(7);

% Parameters
R_accept = 2*L_pp;              % Radius for circle of acceptance
R_LOS = 1000;                   % Radius for Line Of Sight
persistent k;                   % Current waypoint index
persistent wpt_k;               % Current waypoint
persistent alpha_k;               % Angle between north and path vector

% Initiate (Only runs until vessel reach circle of acceptance of first waypoint)
if (R_accept < path_length([vessel.x;vessel.y],waypoints(:,1)) && k == 1)
    k = 1;
    wpt_k = waypoints(:,k);
    desired_heading = atan((wpt_k(1)-vessel.x)/(wpt_k(2)-vessel.y));
end

% Determine active path segment
if (R_accept > path_length([vessel.x;vessel.y],wpt_k)),
    k = k + 1;
    wpt_k = waypoints(:,k);
    path_vec = wpt_k - waypoints(:,k-1);
    alpha_k = angle(path_vec,[0; 1]);
end

e = (wpt_k(1)-vessel.x)*sin(alpha_k)+(vessel.y-wpt_k(2))*cos(alpha_k);
delta = sqrt(R_LOS.^2-e.^2);
heading_r = atan(-e/delta);

desired_heading = alpha_k + heading_r;

end


