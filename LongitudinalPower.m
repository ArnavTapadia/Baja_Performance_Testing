close all

%{
front_motor, center_motor, and rear_motor output the torque, rpm, and power
that that motor is using at a certain grade in rows 1, 2, and 3
respectively. Seems like everything you need to pull the effeciency.
gearbox ratio subject to change.
%}

wheelbase = 132/12;
sprung_weight = 10000;
unsprung_weight = 3000;
mu = 0.65;
wheel_r = 20/12;
cg_x = 63/12; cg_z = 50/12;
l1 = cg_x; l2 = wheelbase/2 - l1; l3 = wheelbase - l1;
k = 4800;

power = 1.34102*400;

air_density = 0.002377;
area = 108;
cd = 1.28;

grade_plot = 1:1:60;
grade = atan(grade_plot.*0.01);
torque_front = zeros(1,length(grade));
torque_center = zeros(1,length(grade));
torque_rear = zeros(1,length(grade));

torque_front2 = zeros(1,length(grade));
torque_center2 = zeros(1,length(grade));
torque_rear2 = zeros(1,length(grade));

gear_ratio = 5.5;
rpm = zeros(1,length(grade));

for idx = 1:1:length(grade)

    fx_cg = -sin(grade(idx))*sprung_weight;
    fz_cg = -cos(grade(idx))*sprung_weight;
    m_cg = -fx_cg*cg_z;
    cg_del = abs(fz_cg/(4*k));

    A = [-l1*k, l2*2*k, l3*k;
         0, 1, -l2/l3;
         k, 2*k, k];
    B = [m_cg - k*cg_del*l1 + 2*k*cg_del*l2 + k*cg_del*l3; 0; 0];

    X = linsolve(A,B);

    tract_r = mu*( k*(cg_del-X(1)) + cos(grade(idx))*unsprung_weight/3);
    tract_c = mu*( 2*k*(cg_del-X(2)) + cos(grade(idx))*unsprung_weight/3);
    tract_f = mu*( k*(cg_del-X(3)) + cos(grade(idx))*unsprung_weight/3);
    
    axial_force_needed = sin(grade(idx))*(sprung_weight+unsprung_weight);

    if (3*tract_f > axial_force_needed)
        torque_needed = axial_force_needed/6*wheel_r;
        torque_front(idx) = torque_needed;
        torque_center(idx) = torque_needed;
        torque_rear(idx) = torque_needed;
    elseif (2*min(tract_c,tract_r) > axial_force_needed-tract_f)
        torque_front(idx) = tract_f*wheel_r*0.5;
        torque_needed = (axial_force_needed-tract_f)/4*wheel_r;
        torque_center(idx) = torque_needed;
        torque_rear(idx) = torque_needed;
    else
        if (tract_r > tract_c)
            torque_front(idx) = tract_f*wheel_r*0.5;
            torque_center(idx) = tract_c*wheel_r*0.5;
            torque_needed = (axial_force_needed - tract_f - tract_c)/2*wheel_r;
            torque_rear(idx) = torque_needed;
        else

            torque_front(idx) = tract_f*wheel_r*0.5;
            torque_rear(idx) = tract_r*wheel_r*0.5;
            torque_needed = (axial_force_needed - tract_f - tract_r)/2*wheel_r;
            torque_center(idx) = torque_needed;
        end
    end
    
    if (axial_force_needed > tract_f + tract_c + tract_r)
        disp('pain')
    end

    power_solve = -inf; rpm(idx) = 0.01;
    while power_solve < power
        
        rpm(idx) = rpm(idx) + 0.01;

        velocity = (2*pi*wheel_r)*(rpm(idx)/60/gear_ratio);
        drag = 0.5*air_density*cd*area*velocity^2;
        axial_force_needed2 = axial_force_needed + drag;
        

        if (3*tract_f > axial_force_needed2)
            torque_needed2 = axial_force_needed2/6*wheel_r;
            torque_front2(idx) = torque_needed2;
            torque_center2(idx) = torque_needed2;
            torque_rear2(idx) = torque_needed2;
        elseif (2*min(tract_c,tract_r) > axial_force_needed2-tract_f)
            torque_front2(idx) = tract_f*wheel_r*0.5;
            torque_needed2 = (axial_force_needed2-tract_f)/4*wheel_r;
            torque_center2(idx) = torque_needed2;
            torque_rear2(idx) = torque_needed2;
        else
            if (tract_r > tract_c)
                torque_front2(idx) = tract_f*wheel_r*0.5;
                torque_center2(idx) = tract_c*wheel_r*0.5;
                torque_needed2 = (axial_force_needed2 - tract_f - tract_c)/2*wheel_r;
                torque_rear2(idx) = torque_needed2;
            else
                torque_front2(idx) = tract_f*wheel_r*0.5;
                torque_rear2(idx) = tract_r*wheel_r*0.5;
                torque_needed2 = (axial_force_needed2 - tract_f - tract_r)/2*wheel_r;
                torque_center2(idx) = torque_needed2;
            end
        end

        if (axial_force_needed2 > tract_f + tract_c + tract_r)
            torque_front2(idx) = tract_f*wheel_r*0.5;
            torque_center2(idx) = tract_c*wheel_r*0.5;
            torque_rear2(idx) = tract_r*wheel_r*0.5;

            total_torque = torque_front2(idx) + torque_center2(idx) + torque_rear2(idx);
            total_torque = total_torque*2;

            power_solve = (total_torque/gear_ratio * rpm(idx))/5252;
            
            disp('panic');
            break
        end
        
        total_torque = torque_front2(idx) + torque_center2(idx) + torque_rear2(idx);
        total_torque = total_torque*2;

        power_solve = (total_torque/gear_ratio * rpm(idx))/5252;
        
    end


end

front_motor = torque_front2./gear_ratio;
center_motor = torque_center2./gear_ratio;
rear_motor = torque_rear2./gear_ratio;

front_motor(2,:) = rpm(1,:);
center_motor(2,:) = rpm(1,:);
rear_motor(2,:) = rpm(1,:);

max_power = zeros(1,length(rpm));
for k=1:1:length(rpm)
    front_motor(3,k) = (front_motor(1,k)*front_motor(2,k))/5252;
    center_motor(3,k) = (center_motor(1,k)*center_motor(2,k))/5252;
    rear_motor(3,k) = (rear_motor(1,k)*rear_motor(2,k))/5252;
    
    max_power(1,k) = 2*(front_motor(3,k) + center_motor(3,k) + rear_motor(3,k));
end

speed_set = rpm*(2*pi*wheel_r);
speed_set = speed_set.*(60/5280/gear_ratio);


figure
plot(rpm, grade_plot);
title('Motor RPM vs. Grade')
xlabel('RPM'); ylabel('Grade');

figure
plot(speed_set, grade_plot);
title('Vehicle Speed vs. Grade')
xlabel('MPH'); ylabel('Grade');


figure
hold on
plot(grade_plot, torque_front);
plot(grade_plot, torque_center);
plot(grade_plot, torque_rear);
legend('front', 'center', 'rear','Location','northwest');
title('Required Wheel Torque vs. Grade');
ylabel('Torque (ft-lb)'); xlabel('Grade');

figure 
hold on
plot(grade_plot, front_motor(1,:));
plot(grade_plot, center_motor(1,:));
plot(grade_plot, rear_motor(1,:));
legend('front', 'center', 'rear','Location','northwest');
title('Required Motor Torque vs. Grade at Power Limit');
ylabel('Torque (ft-lb)'); xlabel('Grade');

figure
hold on
plot(grade_plot, front_motor(3,:));
plot(grade_plot, center_motor(3,:));
plot(grade_plot, rear_motor(3,:));
legend('front', 'center', 'rear','Location','northwest');
title('Required Motor Power vs. Grade at Power Limit');
ylabel('Power (hP)'); xlabel('Grade');

figure
hold on
plot(grade_plot, max_power(1,:));
title('Max Power Usage vs. Grade');
ylabel('Power (hP)'); xlabel('Grade');
ylim([530 540]);
