%% Simulation parameters.
tstart=0;           % Sim start time
tstop=3100;         % Sim stop time
tsamp=10;           % Sampling time for how often states are stored. (NOT ODE solver time step)
                
p0=[1500, 500]; % Initial position (NED)
v0=[6.63 0]';  % Initial velocity (body)
psi0=deg2rad(150);        % Inital yaw angle
r0=0;          % Inital yaw rate
c=1;           % Current on (1)/off (0)

%% Load the waypoints
load('WP.mat');


%% 1st order Nomoto model
K = 0.066928;
T = 118.426906;

%% Heading Controller gains
w_n = 0.1;
zeta = 1;

K_p = w_n^2*T/K;
K_d = 2.2*(2*zeta*w_n*T - 1)/K;
K_i = K_p*w_n/10;

%% Surge model constants
d1 = 1.0163;
d2 = 2.4413e-05;
m = 365.2560;

%% Surge controller gains
lambda = 0.5;
K_ps = 2*lambda;
K_is = lambda^2;

%% Surge reference model constants
zeta_s = 1; % Critical damping
w_ns = 0.01;


%% Simulation
sim MSFartoystyring
pathplotter(p(:,1), p(:,2), psi, tsamp, 7, tstart, tstop, 0, WP);

figure(4)
plot(t, rad2deg(psi_r))
title('Desired heading from guidance system')


