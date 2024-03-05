clear all
close all

%unsprung_weight = 25; %lbm
%This script assumes rigid shocks
%this script does not consider de-accel, steering, slip angle, etc.
%this script does not consider oversteering and lateral slip

%{
Things to add: This script is as simple as possible and only looks at
weight transfer, with a lot of noted assumptions. It also hasn't been built
out to look at understeering moment generated due to wheel slip before the
torsen starts differentiating, nor does it properly capture limit straps +
tipping the car that well. This script does not design anything, it merely
just puts some more numbers behind what Keaton and I have been playing
around with in looking at yaw and ways to AWD with fucking corners, and it
points towards actually helping in corners thus far.
%}

%Vehicle Inputs
mass = 450; %lbm
track_width = 53; %in
wheel_base = 61; %in
weight_distribution = 42; %Percent front
cg_z = 22; %in
wc_z = 10; %effective tire radius

%Need to plug in actual RC logic along with deflection
rc_z = 10; %rollcenter height

%Inputs
mu = 0.7; %per Keaton saying we can pull 0.7Gs

len = 100;

v_band = linspace(1,20,len); %speed in mph
r_bend = linspace(400,200,len); %cornering radius, inches

%America
mass = 0.453592*mass; %kg
track_width = 0.0254*track_width; %m
wheel_base = 0.0254*wheel_base; %m
cg_z = 0.0254*cg_z; %m
rc_z = 0.0254*rc_z; %m
wc_z = 0.0254*wc_z; %m
v_band = v_band.*0.44704; %m/s
r_bend = r_bend.*0.0254; %m
a_vertical = 9.81; %m/s^2


tractive_effort = mass * a_vertical * mu;

weight_outside = zeros(len, len);
weight_inside = zeros(len, len);
a_lateral = zeros(len, len);

for i = 1:1:len
    for j = 1:1:len
        
        a_lateral(i,j) = (v_band(i)^2) / r_bend(j); %m/s^2

        f_lateral = a_lateral(i,j)*mass;
        lateral_thrust = mass * a_lateral(i,j);
        
        rz_adj = a_lateral(i,j) / a_vertical / mu; %travel
        rz_actual = rc_z - (rc_z*rz_adj*3); 
        %RC fake numbers, main region of investigation
        %Playing with this adjusts when you slide vs. tip
        %Using just the normal numbers results in never tipping just
        %sliding, points towards missing something crucial. tread friction?

        A = [1 1;
            0  track_width];
        B = [mass*a_vertical; (track_width/2)*mass*a_vertical - f_lateral*(cg_z-rz_actual)];
        
        wheel_force = linsolve(A,B);
        
        weight_outside(i,j) = wheel_force(1) / sum(wheel_force(:));
        weight_inside(i,j) = 1 - weight_outside(i,j);

        %tipover condition
        if weight_inside(i,j) < 0
            weight_outside(i,j) = 1;
            weight_inside(i,j) = 0;
            a_lateral(i,j) = 0;
        end
        
        %slip condition
        if lateral_thrust >= tractive_effort && a_lateral(i,j) > 0
            weight_outside(i,j)  = 0;
            weight_inside(i,j)  = 0;
            a_lateral(i,j) = 0;
        end
        
    end
end

thrust_outside = weight_outside.*tractive_effort*0.5;
thrust_inside = weight_inside.*tractive_effort*0.5;


%Needs Engine Dyno Data
power = 10; %hP
engine_rpm = 3000; %rpm

%Torsen
len2 = 100;

TBR = linspace(1,3,len2);
yaw_set = zeros(len,len,len2);
tor_inside_torque = zeros(len,len,len2);
tor_outside_torque = zeros(len,len,len2);

for k = 1:1:len2
    for i=1:1:len
        for j = 1:1:len
        
            wheel_rpm = (v_band(i)*1056) / (2*pi*wc_z);
            drive_ratio = engine_rpm /wheel_rpm; 
            torque_engine = power / (engine_rpm*5252);
            torque_front = 0.5*drive_ratio*torque_engine;
        
            max_torque_inside = thrust_inside(i,j) * wc_z;
            max_torque_outside = thrust_outside(i,j) * wc_z;
            wheel_torque = torque_front / 2;

            torque_ratio = max_torque_outside / max_torque_inside;
            
            %refine to capture wheel slip scenarios / RPM
            if TBR(k) > torque_ratio %locked diff
                if max_torque_inside > wheel_torque
                    tor_inside_torque(i,j,k) = wheel_torque;
                    tor_outside_torque(i,j,k) = wheel_torque;
                    yaw_set(i,j,k) = 0;
                %{
                else %inside wheel slip scenario
                    tor_inside_torque(i,j,k) = max_torque_inside;
                    tor_outside_torque(i,j,k) = torque_front - max_torque_inside;
                    if tor_outside_torque(i,j,k) > max_torque_outside
                        tor_outside_torque(i,j,k) = max_torque_outside;
                    end
                    yaw_set(i,j,k) = tor_outside_torque - tor_inside_torque;
                %}

                end
            else
                    tor_inside_torque(i,j,k) = max_torque_inside;
                    tor_outside_torque(i,j,k) = TBR(k)*max_torque_inside;

                    if tor_outside_torque(i,j,k) > max_torque_outside
                        tor_outside_torque(i,j,k) = max_torque_outside;
                    end
                    yaw_set(i,j,k) = tor_outside_torque(i,j,k) - tor_inside_torque(i,j,k);
            end
            

        end
    end
end

yaw_mean = 0; tbr_select = 0; yaw_mean_set = zeros(1,len2);
for i=1:1:len2
    yaw_check = mean(yaw_set(:,:,i),"All");
    yaw_mean_set(1,i) = yaw_check;

    if yaw_check > yaw_mean
        tbr_select = i;
        yaw_mean = yaw_check;
    end

end

TBR_CHOSEN = TBR(tbr_select);

figure
plot(TBR, yaw_mean_set);