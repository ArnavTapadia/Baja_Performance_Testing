clear all
close all
clc

load_system('Half_Car_Model.slx');

%% getting basic parameters
mdlWks = get_param('Half_Car_Model','ModelWorkspace');
rRollingRadius = 0.2794; %meters
zCOMRideHeight = mdlWks.getVariable('h').Value; % meters
zAxleRideHeight = zCOMRideHeight - rRollingRadius; %meters
L_F = mdlWks.getVariable('L_F').Value; % meters - front shock pickup point distance from COM; paralle
L_R  = mdlWks.getVariable('L_R').Value; % meters - rear shock pickup point distance from COM; parallel to floor
vCar = mdlWks.getVariable('v').Value; % car longitudinal velocity;
gLong = mdlWks.getVariable('gLong').Value; % car longitudinal acceleration;

%% Setting road input
% TODO: call function to set custom waveform (need to make)
z_Fg.time = [0:0.001:4.5]';
z_Fg.signals.values = zeros(size(z_Fg.time));

%% running sim
simout = sim('Half_Car_Model.slx');
% TODO: Fix so its all measured relative to ground not equilibrium ride heights

%% gettign sim output
data.time = simout.tout;
data.sLapFWheel = vCar.*data.time; % measured from front wheel
data.phi_s = simout.phi_s.Data; % sprung mass pitch angle clockwise convention (into the page)
data.phi_s = -data.phi_s; % anticlockwise since convention is ****into the page****
data.sLap = data.sLapFWheel - L_F*cos(data.phi_s); % COM x Position
data.z_g_COM = simout.z_COMg.Data; % ground height relative to 0 vertically below COM;
data.z_s = simout.z_s.Data + zCOMRideHeight - data.z_g_COM; % sprung mass height above ground
data.z_Fg = simout.z_Fg.Data; % front ground
data.z_Rg = simout.z_Rg.Data; % rear ground
data.z_FW = data.z_s + sin(data.phi_s)*L_F - zAxleRideHeight - (simout.z_Fs.Data - data.z_Fg); % front wheel center z height from ground below Front wheel
data.z_RW = data.z_s - sin(data.phi_s)*L_R - zAxleRideHeight - (simout.z_Rs.Data - data.z_Rg); % rear wheel center z height from ground below rear wheel
% data.z_FW = simout.z_Fs.Data + rRollingRadius not relative to ground
% data.z_RW = simout.z_Rs.Data + rRollingRadius

% Plots of Variables
figure
subplot(4,1,1)
plot(data.sLapFWheel, data.z_s);
title('Sprung mass COM height')
legend('z_s')
xlabel('sLap FWheel (m)')
ylabel('z (m)')
grid minor

subplot(4,1,2)
plot(data.sLapFWheel, data.phi_s.*180/pi);
title('Sprung mass pitch angle')
legend('phi_s')
xlabel('sLap FWheel (m)')
ylabel('pitch angle (deg)')
grid minor

subplot(4,1,3)
%fronts
plot(data.sLapFWheel, data.z_FW, 'r')
hold on
% rear
plot(data.sLapFWheel, data.z_RW, 'b')
%rears
title('Wheel Centers z Height')
legend('z_Fs', 'z_Rs');
xlabel('sLap FWheel (m)')
ylabel('z (m)')
grid minor

subplot(4,1,4)
plot(data.sLapFWheel, data.z_Fg, '-r');
hold on
plot(data.sLapFWheel, data.z_Rg, '-b');
title('ground height')
legend('Ground (z_Fg)','Ground (z_Rg)');
xlabel('sLap FWheel (m)')
ylabel('z (m)')
grid minor

% Animation

figure(2)

subplot(2,1,2)
plot(data.sLapFWheel, data.z_Fg, '-r');
title('Road Surface')
legend('Ground (z_Fg)');
xlabel('sLap FWheel (m)')
ylabel('z (m)')
grid minor
ylim([min(data.z_Fg)-0.1,max(data.z_s)+0.1])

subplot(2,1,1)
hold on
ylim([min(data.z_FW)-0.1,max(data.z_s)+0.1])
xlim([-1,1])

for i = 1:length(data.z_s)
    %% front wheel location
    subplot(2,1,2)
    hold on
    carLocation = plot(data.sLapFWheel(i), data.z_Fg(i), '.k', 'MarkerSize', 5, 'HandleVisibility','off');
    
    %% animation
    subplot(2,1,1)
    hold on
    %sprung mass plot
    COM = plot(0,data.z_s(i),'.k', 'MarkerSize', 10);
    sprungMass = line([-L_R*cos(data.phi_s(i)) L_F*cos(data.phi_s(i))], ...
        [data.z_s(i)-L_R*sin(data.phi_s(i)) data.z_s(i)+L_F*sin(data.phi_s(i))]);

    %Front wheel center
    WheelCenterF = plot(L_F*cos(data.phi_s(i)), data.z_FW(i), 'or', 'MarkerSize', 5);
    WheelRimF =  plot(L_F*cos(data.phi_s(i)), data.z_FW(i), 'ok', 'MarkerSize', 25);
    WheelCenterR = plot(-L_R*cos(data.phi_s(i)), data.z_RW(i), 'ob', 'MarkerSize', 5);
    WheelRimR = plot(-L_R*cos(data.phi_s(i)), data.z_RW(i), 'ok', 'MarkerSize', 25);
    
    % shock
    shockF = line([L_F*cos(data.phi_s(i)) L_F*cos(data.phi_s(i))], [data.z_s(i)+L_F*sin(data.phi_s(i)) data.z_FW(i)]);
    shockR = line([-L_R*cos(data.phi_s(i)) -L_R*cos(data.phi_s(i))], [data.z_s(i)-L_R*sin(data.phi_s(i)) data.z_RW(i)]);

    % deleting previous step
    pause(0.1)
    if(i == length(data.z_s))
        break;
    end
    delete(COM)
    delete([sprungMass shockF shockR])
    delete(carLocation)
    delete([WheelCenterF WheelCenterR WheelRimF WheelRimR])
end
