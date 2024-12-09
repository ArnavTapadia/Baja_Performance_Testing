function dxdt = quarter_car_model_ss(t, x, params)
    % Extract parameters
    ms = params.ms; % Sprung mass
    mu = params.mu; % Unsprung mass
    ks = params.ks; % Suspension stiffness
    cs = params.cs; % Suspension damping
    kt = params.kt; % Tire stiffness
    zr = interp1(params.road_func_t, params.road_func,t); % Road displacement as a function of time
    g = 9.81; % Gravitational acceleration
    delta_z_t = mu * g / kt; % Static displacement threshold for jumping

    % State variables
    z_s = x(1); % Sprung mass position
    z_s_dot = x(2); % Sprung mass velocity
    z_u = x(3); % Unsprung mass position
    z_u_dot = x(4); % Unsprung mass velocity

    % Check contact condition
    in_contact = (z_u - zr) <= 0*delta_z_t;

    % Differential equations
    z_s_ddot = (-ks * (z_s - z_u) - cs * (z_s_dot - z_u_dot)-g) / ms-g;

    if in_contact
        % Tire in contact
        z_u_ddot = (ks * (z_s - z_u) + cs * (z_s_dot - z_u_dot) - kt * (z_u - zr)) / mu-g;
    else
        % Tire loses contact (free motion with gravity)
        z_u_ddot = (ks * (z_s - z_u) + cs * (z_s_dot - z_u_dot)) / mu-g;
    end

    % State derivatives
    dxdt = [z_s_dot; z_s_ddot; z_u_dot; z_u_ddot];
end
