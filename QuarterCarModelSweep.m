clear all
close all
clc

load_system('Quarter_Car_model');

springStiffnessSweep = [3159.177 5000];

simout = cell(1,length(springStiffnessSweep));

mdlWks = get_param('Quarter_Car_model','ModelWorkspace');
% whos(mdlWks) % lists all variables in the workspace
tic
for i = 1:length(springStiffnessSweep) %front spring stiffness sweep
    assignin(mdlWks, 'k_s', springStiffnessSweep(i))
    % getVariable(mdlWks,'k_s')
    simout{i} = sim('Quarter_Car_model');
end

run1 = simout{1};
run2 = simout{2};

figure
subplot(2,1,1)
plot(run1.yout{1}.Values.Time, run1.yout{1}.Values.Data, 'g')
hold on
% plot(run1.yout{2}.Values.Time, run1.yout{2}.Values.Data, 'b')

plot(run2.yout{1}.Values.Time, run2.yout{1}.Values.Data, 'r')
% plot(run2.yout{2}.Values.Time, run2.yout{2}.Values.Data)

plot(run1.yout{3}.Values.Time, run1.yout{3}.Values.Data, 'k')
legend('Sprung Mass Height k = 3159.177', 'Sprung Mass Height k = 1000', 'x\_ground')
title('run 1 & 2 test comparison')
xlim([3,8])
xlabel('time (s)')
ylabel('position (m)')


% determining power spectral density
[Pxx1, F1] = PSD(run1.yout{1}.Values.Time, run1.yout{1}.Values.Data);

[Pxx2, F2] = PSD(run2.yout{1}.Values.Time, run2.yout{1}.Values.Data);

subplot(2,1,2)
plot(F1, Pxx1, 'g')
xlabel('Frequency (Hz)')
title('Power Spectral Density')
grid on
hold on
plot(F2, Pxx2, 'r')
xlim([0, 10])

% figure
% plot(run2.yout{1}.Values.Time, run2.yout{1}.Values.Data, 'g')
% hold on
% plot(run2.yout{2}.Values.Time, run2.yout{2}.Values.Data, 'b')
% plot(run2.yout{3}.Values.Time, run2.yout{3}.Values.Data, 'r')
% legend('x_s', 'x_w', 'x_ground')
% title('run 2')



