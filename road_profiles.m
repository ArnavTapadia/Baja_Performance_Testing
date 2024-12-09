function zr = road_profiles(t, type,vCar)
    switch type
        case 'sinusoidal'
            zr = 0.2 * (sin(2 * pi * 0.8 * (t - pi/2)) + 1); % Sinusoidal profile
        case 'step'
            zr = (t > 1) * 0.05; % Step input at t = 1 second
        case 'jump'
            % Define bump parameters
            jump_start = 1;       % Start time of bump (s)
            jump_length = 0.5;    % Length of bump (m)
            jump_duration = jump_length / vCar; % Duration of bump (s)
            jump_end = jump_start + vCar;
            jump_amplitude = 0.5; % Bump height (m)
            
            % Logical masks for each region
            flat_before = t < jump_start;
            jump_region = (t >= jump_start) & (t <= jump_end);
            flat_after = t > jump_end;

            % Create the bump
            zr(jump_region) = jump_amplitude / jump_duration * (t(jump_region) - jump_start);
            zr(flat_before) = 0; % Flat before the bump
            zr(flat_after) = 0;  % Flat after the bump

        case 'bump'
            % Define bump parameters
            bump_start = 0.5;           % Start time of bump (s)
            bump_length = 5;            % Length of bump (m)
            bump_duration = bump_length / vCar; % Duration of bump (s)
            bump_end = bump_start + bump_duration;
            bump_amplitude = 0.5;       % Bump height (m)
            
            % Logical masks for each region
            flat_before = t < bump_start;
            bump_region = (t >= bump_start) & (t <= bump_end);
            flat_after = t > bump_end;

            % Create the bump (cosine shape)
            zr(bump_region) = bump_amplitude / 2 * (-1 * cos(2 * pi * (t(bump_region) - bump_start) / bump_duration) + 1);
            zr(flat_before) = 0; % Flat before the bump
            zr(flat_after) = 0;  % Flat after the bump

        case 'multiple_bumps'
            % Define bump parameters
            bump_length = 5;            % Length of bump (m)
            bump_duration = bump_length / vCar; % Duration of bump (s)
            bump_amplitude = 0.3;       % Bump height (m)
            period = 1.5;               % Time interval between bumps (s)

            % Initialize the road profile
            zr = zeros(size(t));
            
            % Create periodic bumps
            for k = 0:floor(max(t) / period)
                bump_start = k * period+2;            % Start time of current bump
                bump_end = bump_start + bump_duration; % End time of current bump
                
                % Logical mask for the current bump region
                bump_region = (t >= bump_start) & (t <= bump_end);
                
                % Add the bump to the road profile
                zr(bump_region) = bump_amplitude / 2 * (-1 * cos(2 * pi * (t(bump_region) - bump_start) / bump_duration) + 1);
            end
            
        otherwise
            zr = 0; % Flat road
    end

    % Optionally, apply a smoothing pulse (uncomment if needed)
    % smoothing_pulse = smoothed_pulse(t, 0.02, 30); % Hardcoded - need to change
    % zr = zr .* smoothing_pulse;
end


function S = smoothed_pulse(t, t_start, k)
    % Generates a rectangular pulse with smoothed edges
    % t: Time vector
    % t_start: Pulse start time
    % t_end: Pulse end time
    % k: Smoothing factor
    
    % Logistic functions for smooth transitions
    start_transition = 1 ./ (1 + exp(-k * (t - t_start)));
    S = start_transition;
end