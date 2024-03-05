clear all
close all
clc

load_system('Half_Car_Model');

frontSpringStiffnessSweep = [5 2 5 1 5];
rearSpringStiffnessSweep = [5 2 15 6 1];

simout = cell(length(frontSpringStiffnessSweep),length(frontSpringStiffnessSweep));

mdlWks = get_param('Half_Car_Model','ModelWorkspace');
% whos(mdlWks) % lists all variables in the workspace
tic
for i = 1:length(frontSpringStiffnessSweep) %front spring stiffness sweep
    assignin(mdlWks, 'k_Fs', frontSpringStiffnessSweep(i))
    for j = 1:length(rearSpringStiffnessSweep)
        assignin(mdlWks, 'k_Rs', rearSpringStiffnessSweep(j));
        simout{i,j} = sim('Half_Car_Model');
    end
end