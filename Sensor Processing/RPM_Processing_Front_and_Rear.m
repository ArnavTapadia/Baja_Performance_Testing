clear
close all

fileName = ['straight 4wd small wheel_0.dat'];

RearTeethPerRev=4; %Spokes per rotation for rear wheel
FrontTeethPerRev=28; %Spokes per revolution on the front wheel
%gboxRatio=9.75;
maxVoltage=.3; %Max hall effect output voltage
tire_diameter = 22/12; %ft
%fsamp = 1000; %sampling frequency
%smoothConstant = 5;%fsamp/200;

disp(['Importing ' fileName]);
data=importdata(fileName);
time=data.data(:,1);
rear_rpm_data=data.data(:,2); %Rear wheel hall effect data column
front_rpm_data=data.data(:,4); %Front wheel hall effect data column

figure(1)
 plot(rear_rpm_data);
 title('Rear Hall Effect');
 ylim([-0.4 1])
 %xlim([1.4 1.41])

figure(4)
plot(front_rpm_data);
title('Front Hall Effect');
ylim([-0.4 1])

%smoothRPM=smooth(rpm_data,smoothConstant); %preprocessing
%rpm = calcRpm([time smoothRPM], teethPerRev, maxVoltage);
rear_rpm = calcRpm([time rear_rpm_data], RearTeethPerRev, maxVoltage);

%rpm(rpm>4500) = 0;
Front_rpm = calcRpm([time front_rpm_data], FrontTeethPerRev, maxVoltage);
%secondary_rpm = gbox_rpm.*gboxRatio;
%ratio = rpm(:,2)./secondary_rpm(:,2);
%ratio(ratio>2.5) = nan;
%wheel_speed = gbox_rpm.*tire_diameter*pi*60/5280;
%wheel_speed = filloutliers(wheel_speed,'linear');

%ratio = filloutliers(ratio,'linear');
%rpm(:,2) = filloutliers(rpm(:,2),'linear');
%rpm(:,2) = smooth(rpm(:,2),1000);

figure(3)
hold on
yyaxis left;
plot(time, rear_rpm(:,2), "r");
plot(time, Front_rpm(:,2));
ylim([0 550])
legend(["Rear", 'Front', 'Wheel Slip'])
xlabel('time [sec]');
ylabel('RPM');
title('RPM vs. time');
yyaxis right;
plot(time, 100.*(rear_rpm(:,2) - Front_rpm(:,2))./Front_rpm(:,2),"g");
ylabel('% Slip')
ylim([0 200])
%yyaxis right;
%plot(time, ratio);
%ylabel('ratio');
grid on
grid minor

% figure(4)
% plot(time, ratio);
% xlabel('time [sec]');
% ylabel('ratio');

%figure(5)
%plot(time, wheel_speed(:,2));
%xlabel('time');
%ylabel('speed [mph]');

%spreadfigures;

function rpm=calcRpm(data, teethPerRev, maxVoltage)
%Calculates RPM values given the raw hall effect data:

[nr,~]=size(data);
rpm(:,1)=data(:,1);

signal=zeros(nr,1, 'logical');
data(:,2)=round(data(:,2),2);

for idx=1:nr
    
    if data(idx,2)>maxVoltage/2 %Coverting hall effect voltages to binary
        signal(idx)=1;
    end
    
end

% figure(2)
% plot(signal);

toothCounter=0;
timeFrameStart=1;
for idx=2:nr
    if signal(idx-1)~=signal(idx)%Checking for changes in voltage
        toothCounter=toothCounter+0.5; %two changes in voltage to every passing tooth
    end
    if toothCounter==1
        dt=rpm(idx,1)-rpm(timeFrameStart,1); %Time taken for one revolution's worth of teeth to pass
        rpm(timeFrameStart:idx,2)=1/dt*60*(1/teethPerRev);%Calculating rpm
        timeFrameStart=idx+1;
        toothCounter=0;
    end
end

dt=rpm(nr,1)-rpm(timeFrameStart,1);
rpm(timeFrameStart:nr,2)=toothCounter/teethPerRev/dt*60;

end