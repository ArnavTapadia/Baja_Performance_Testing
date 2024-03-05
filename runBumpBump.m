% Run learner solution.
clear
clc

g = 9.81;

% INITIAL CONDITIONS:
% CORNER MASS:
    m = 105;            % mass car (corner) lb
% MOTION RATIO:
    mR = .52;             % motion ratio (shock/wheel)
% SHOCK:
  % SPRING:
    k = 60;               % lb/in
    % PRELOAD
      pL = .75;            % in
  % DAMPING:
    comp = (150/(35));    % lb/(in/s)
    reb  = (200/(40));    % lb/(in/s)
  % TRAVEL:
    travel = 13;          % in
% INITIAL HEIGHT:
    Y0 = 20;              % in (0 is bottomed out)
% INITIAL VELOCITY:
    dY0 = -2;              % m/s (postive up)
% MAX TIME:
    tEnd = 4;             % s

% CONVERT VALUES TO SI UNITS
m = m*.4535;                    % lb*(kg/lb)= kg
k = k*(.4535)*(1/.0254)*g;      % (lb/in)*(kg/lb)*(in/m)*(N/kg) = N/m
pL = pL*.0254;                  % in*(m/in)= m
comp = comp*(.4535)*(1/.0254)*g;% (lb/(in/s))*(kg/lb)*(in/m)*(N/kg) = N*s/m
reb  = reb*(.4535)*(1/.0254)*g; % (lb/(in/s))*(kg/lb)*(in/m)*(N/kg) = N*s/m
travel = travel*.0254;          % in*(m/in)= m
Y0 = Y0*.0254;                  % in*(m/in)= m
    
[ie,coeffComp,coeffReb,yend]=BumpBump(m,mR,k,pL,comp,reb,travel,Y0,dY0,tEnd);
disp(yend)
