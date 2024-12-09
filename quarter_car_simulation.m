clear all
clc
close all
% Main script for simulating the quarter car model
%% Vehicle Parameters
params.kt = 3092200; % N/m tire stiffness
params.ks = 3159.177; % N/m shock stiffness (wheel rate)
params.cs = 237.5; % kg/s shock damping
params.mu = 25/2.2; % kg (single wheel weight)
params.ms = 181/2.2/2; %kg sprung mass weight (front) divided by number of wheels
params.rRollingRadius = 0.2794; % m wheel center height
params.v = 11; %m/s

% Initial conditions [z_s, z_s_dot, z_u, z_u_dot]

x0 = [0; 0; 0; 0];

% Time span
tspan = [0 15]; % Simulate for 5 seconds

params.road_func_t = tspan(1):0.001:tspan(2);
params.road_func = road_profiles(params.road_func_t, 'multiple_bumps',params.v); % Road profile (sinusoidal)
params.zCOM_initial = 0.51 + params.road_func(1); % m ride height above initial ground
params.zWheel_initial = params.rRollingRadius + params.road_func(1);

% Solve ODE
[t, x] = ode45(@(t, x) quarter_car_model_ss(t, x, params), tspan, x0);

% Plot results
figure;
zCOM = x(:, 1) + params.zCOM_initial;
zWheel = x(:,3) + params.zWheel_initial;
zTire = x(:,3);
zr = interp1(params.road_func_t,params.road_func,t);

subplot(2,1,1)
plot(t, zCOM, t, zWheel, t, zTire, t, zr);
legend('Sprung Mass (z_s)', 'Unsprung Mass (z_u)', 'tire contact', 'Ground (z_r)');
xlabel('Time (s)');
ylabel('Displacement (m)');
title('Quarter Car Model: Displacements');


subplot(2,1,2)
sLap = params.v*t;
plot(sLap, zCOM, sLap, zWheel, sLap, zTire, sLap, zr);
legend('Sprung Mass (z_s)', 'Unsprung Mass (z_u)', 'tire contact', 'Ground (z_r)');
xlabel('sLap (m)');
ylabel('Displacement (m)');
title('Quarter Car Model: Displacements');

%plotting PSDs
% determining power spectral density
[Pxx1, F1] = PSD(t, zCOM);

plot(F1, Pxx1, 'g')
xlabel('Frequency (Hz)')
title('Power Spectral Density of COM height')
grid on
xlim([0, 10])
