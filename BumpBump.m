function [ie,coeffComp,coeffReb, yend]=BumpBump(m,mR,k,pL,comp,reb,travel,Y0,dY0,tEnd)
% ie is a flag to indicate which state the car is in
% coeffComp is the compression coefficient relative to critical damping
% coeffReb is the rebound coefficient relative to critical damping

%%
% GENERAL NUMBERS:
g = 9.81;
coeffComp = .5;
coeffReb  = .7;
maxCycles =  100;   % stops it from breaking
coeffBump = .01;   % coeff of restitution for bump stop
amp = 10;
freq = 20; 
% MAKE SHOCK IN TERMS OF WHEEL:
k = k*mR;
pL = pL/mR;
comp = comp*(mR^2);
reb = reb*(mR^2);
%% Prepare numerical integration
% define the RHS function for while car isn't touching the ground
    function hg=ground(t)
        hg=(amp/2)*(1-sin(freq*t))*.0254;
    end

    function v=der(t)
        v= -(amp/2)*freq*cos(freq*t)*.0254;
    end

    function dz=freefallRHS(t,z)
        % z(1) = position, z(2) = velocity
        dz = [z(2); -g];
    end

    function dz=compressionRHS(t,z)
        y=z(1); dy=z(2);
        dz = [dy; -g-(((dy-der(t))*comp)/m)+(k*(travel-y+pL+ground(t))/m)];
    end

    function dz=reboundRHS(t,z)
        y=z(1); dy=z(2);
        dz = [dy; -g-(((dy-der(t))*reb)/m)+(k*(travel-y+pL+ground(t))/m)];
    end

% EVENT FUNCTIONS:
    function [value, isTerminal, direction] = springEngages(t,z)
        % There are three conditions when we want to stop the integration:
        % 1) the car engages the shock
        value(1) = z(1)-travel-ground(t);
        isTerminal = 1;  %stops integrating when any condition is met
        direction = -1; %sign of derivative to trigger event
    end
    function [value, isTerminal, direction] = somethingChanges(t,z)
        % There are three conditions when we want to stop the integration:
        % make a vector called "value" that has three entries corresponding to
        % those three conditions. Make sure that when the condition is hit,
        % your "value"=0.
        value(1) = z(1)-travel-ground(t); % HITS SHOCK
        value(2) = z(1)-ground(t);        % BOTTOMS OUT
        value(3) = z(2);                  % INITIATES REBOUND
        value(4) = z(2);                  % INITIATES COMPRESSION
        value(5) = z(1)-travel-ground(t); % LEAVES GROUND
        isTerminal = [ 1  1  1  1  1];  %stops integrating when any condition is met
        direction  = [-1 -1  1 -1  1]; %sign of derivative to trigger event
    end

% initialize integration
ICs=[Y0;dY0];
tfinal=tEnd; % final time.
opts=odeset('RelTol',1e-10,'AbsTol',1e-10,'Events',@springEngages);


%% Perform integration
% Run the loop-the-loop integration until shock engages, 
% car bottoms out, or switches directions. check to see which event happened.
[tstart,zstart,te,ze,ie]=ode89(@freefallRHS,[0 tfinal], ICs, opts);
        ttotal=tstart;
        ytot=zstart(:,1); dytot=zstart(:,2);

disp('freefall complete')
i = 0;
while ttotal(end) < tEnd && i < maxCycles
    
    if ie == 1 || ie == 4 % Spring compression event
        disp('compression initiated')
        % find the ICs to start the spring integration.
        yend=ze(1);
        ydotend=ze(2);
        ICs2 = [yend, ydotend];
        % integrate for freefall
        opts2=odeset('RelTol',1e-11,'AbsTol',1e-11,'Events',@somethingChanges);
        [tcomp,zcomp,te,ze,ie]=ode89(@compressionRHS,[te tfinal], ICs2, opts2);
        % concatenate results
        ttotal=[ttotal;tcomp];
        ytot=[ytot;zcomp(:,1)];
        dytot=[dytot;zcomp(:,2)];
        
    elseif ie == 2
        disp('shock bottomed out. rebound initiated')
        % find the ICs to start the spring integration.
        yend=ze(1);
        ydotend=-ze(2)*coeffBump;
        ICs2 = [yend, ydotend];
        % integrate for freefall
        opts2=odeset('RelTol',1e-11,'AbsTol',1e-11,'Events',@somethingChanges);
        [tbump,zbump,te,ze,ie]=ode89(@reboundRHS,[te tfinal], ICs2, opts2);
        % concatenate results
        ttotal=[ttotal;tbump];
        ytot=[ytot;zbump(:,1)];
        dytot=[dytot;zbump(:,2)];
        
    elseif ie == 3
        disp('rebound initiated')
        % find the ICs to start the spring integration.
        yend=ze(1);
        ydotend=ze(2);
        ICs2 = [yend, ydotend];
        % integrate for freefall
        opts2=odeset('RelTol',1e-11,'AbsTol',1e-11,'Events',@somethingChanges);
        [treb,zreb,te,ze,ie]=ode89(@reboundRHS,[te tfinal], ICs2, opts2);
        % concatenate results
        ttotal=[ttotal;treb];
        ytot=[ytot;zreb(:,1)];
        dytot=[dytot;zreb(:,2)];
        
    elseif ie == 5
        disp('freefall initiated')
        % find the ICs to start the spring integration.
        yend=ze(1);
        ydotend=ze(2);
        ICs2 = [yend, ydotend];
        % integrate for freefall
        opts2=odeset('RelTol',1e-11,'AbsTol',1e-11,'Events',@somethingChanges);
        [tfall,zfall,te,ze,ie]=ode89(@freefallRHS,[te tfinal], ICs2, opts2);
        % concatenate results
        ttotal=[ttotal;tfall];
        ytot=[ytot;zfall(:,1)];
        dytot=[dytot;zfall(:,2)];
        wheel=[ytot-travel;zfall(:,1)];
    end
    i = i+1;
    disp(ytot(end)/.0254)
end
%% Plot results

ytot = ytot/.0254;              % convert back to inches for plotting
dytot = dytot/.0254*mR;
figure; 
hold on
plot(ttotal,ytot,'b-')
plot(tstart,zstart(:,1)/.0254,'r-')
%plot(tstart,wheel/.0254,'r-')
fplot(@(t) ground(t)/.0254,[0, tEnd],'r-')
%plot(ttotal,dytot,'k-')
title('Position vs Time'); 
xlabel('Time'); ylabel('Vertical Position');
hold off

yend=ytot(end);

end