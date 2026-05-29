function [y, y_meas_hist] = simulate_closedloop_2dof_euler_attacked(plant_ss, sensor_ss, C_r, C_y, t, r, attack_cfg)
% Closed-loop 2DoF simulation with attack injected on the measured signal.
plant_ss = ss(plant_ss);
sensor_ss = ss(sensor_ss);
A = plant_ss.A; B = plant_ss.B; C = plant_ss.C; D = plant_ss.D;
As = sensor_ss.A; Bs = sensor_ss.B; Cs = sensor_ss.C; Ds = sensor_ss.D;

Cr_ss = safe_controller_ss(C_r, plant_ss);
Cy_ss = safe_controller_ss(C_y, plant_ss);
Ar = Cr_ss.A; Br = Cr_ss.B; Crm = Cr_ss.C; Dr = Cr_ss.D;
Ay = Cy_ss.A; By = Cy_ss.B; Cym = Cy_ss.C; Dy = Cy_ss.D;

nx_p = size(A,1);
nx_s = size(As,1);
nx_r = size(Ar,1);
nx_y = size(Ay,1);

xp = zeros(nx_p,1);
xs = zeros(nx_s,1);
xr = zeros(nx_r,1);
xy = zeros(nx_y,1);

N = length(t);
y = zeros(N,1);
y_meas_hist = zeros(N,1);
u_prev = 0;

for k = 1:N
    if k == 1
        dt = t(1);
    else
        dt = t(k) - t(k-1);
    end

    yk = C * xp + D * u_prev;
    if nx_s > 0
        y_s = Cs * xs + Ds * yk;
        xs = xs + (As * xs + Bs * yk) * dt;
    else
        y_s = yk;
    end
    y_meas = apply_attack_scalar(y_s, t(k), attack_cfg);

    ur = Crm * xr + Dr * r(k);
    uy = Cym * xy + Dy * y_meas;
    uk = ur - uy;

    xr = xr + (Ar * xr + Br * r(k)) * dt;
    xy = xy + (Ay * xy + By * y_meas) * dt;
    xp = xp + (A * xp + B * uk) * dt;

    y(k) = C * xp + D * uk;
    y_meas_hist(k) = y_meas;
    u_prev = uk;
end
end

function y_attack = apply_attack_scalar(y, t, attack_cfg)
y_attack = y;
if nargin < 3 || isempty(attack_cfg) || ~isfield(attack_cfg, 'enabled') || ~attack_cfg.enabled
    return;
end
if ~isfield(attack_cfg, 'start_time')
    attack_cfg.start_time = 0;
end
if t < attack_cfg.start_time
    return;
end
attack_start_time = attack_cfg.start_time;
active = true;
if isfield(attack_cfg,'burst_on_time') && isfield(attack_cfg,'burst_off_time') && attack_cfg.burst_on_time > 0 && attack_cfg.burst_off_time >= 0
    period = attack_cfg.burst_on_time + attack_cfg.burst_off_time;
    if period > 0
        cycle_idx = floor((t - attack_cfg.start_time) / period);
        if isfield(attack_cfg,'burst_cycles') && ~isempty(attack_cfg.burst_cycles) && cycle_idx >= attack_cfg.burst_cycles
            active = false;
        else
            cycle_t = t - attack_cfg.start_time - cycle_idx * period;
            active = cycle_t <= attack_cfg.burst_on_time;
            attack_start_time = attack_cfg.start_time + cycle_idx * period;
        end
    end
end
if ~active
    return;
end
switch lower(string(attack_cfg.type))
    case "bias"
        if isfield(attack_cfg, 'magnitude')
            y_attack = y + attack_cfg.magnitude;
        end
    case "ramp"
        if isfield(attack_cfg, 'slope')
            y_attack = y + attack_cfg.slope * (t - attack_start_time);
        end
    case "sine"
        amp = 0;
        freq = 1;
        if isfield(attack_cfg, 'magnitude'), amp = attack_cfg.magnitude; end
        if isfield(attack_cfg, 'frequency'), freq = attack_cfg.frequency; end
        y_attack = y + amp * sin(2*pi*freq*(t - attack_start_time));
    otherwise
        y_attack = y;
end
end

function ss_sys = safe_controller_ss(C, plant_ss)
try
    if isa(C,'tf') || isa(C,'zpk') || isa(C,'pid') || isa(C,'pidstd')
        tfC = tf(C);
        try
            [num, den] = tfdata(tfC, 'v');
            if ~isempty(num) && ~isempty(den)
                isProper = true;
                try
                    isProper = isproper(tfC);
                catch
                    isProper = numel(num) <= numel(den);
                end
                if ~isProper
                    k = real(evalfr(tfC, 0));
                    if ~isfinite(k), k = 1; end
                    ss_sys = ss(k);
                    return;
                end
            end
        catch
        end
    end
    ss_sys = ss(C);
catch
    try
        pid_fb = pidtune(plant_ss, 'PID');
        ss_sys = ss(pid_fb);
    catch
        try
            k = evalfr(C, 0);
            if ~isfinite(k)
                k = 1;
            end
            ss_sys = ss(real(k));
        catch
            ss_sys = ss(1);
        end
    end
end
end
