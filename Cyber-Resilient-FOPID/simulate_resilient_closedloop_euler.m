function [u, mode_history, switch_times, y, diag] = simulate_resilient_closedloop_euler(plant_ss, sensor_ss, C_r, C_y, C_pid, t, r, attack_cfg, attack_flag, detection_time, switcher_cfg)
% Self-consistent resilient closed-loop simulation.
% Mode 1: 2DoF control u = C_r*r - C_y*y_meas
% Mode 3: PID control on isolated measurement after detection

plant_ss = ss(plant_ss);
sensor_ss = ss(sensor_ss);
A = plant_ss.A; B = plant_ss.B; C = plant_ss.C; D = plant_ss.D;
As = sensor_ss.A; Bs = sensor_ss.B; Cs = sensor_ss.C; Ds = sensor_ss.D;

Cr_ss = safe_controller_ss(C_r, plant_ss);
Cy_ss = safe_controller_ss(C_y, plant_ss);
Cpid_ss = safe_controller_ss(C_pid, plant_ss);
Ar = Cr_ss.A; Br = Cr_ss.B; Crm = Cr_ss.C; Dr = Cr_ss.D;
Ay = Cy_ss.A; By = Cy_ss.B; Cym = Cy_ss.C; Dy = Cy_ss.D;
Ap = Cpid_ss.A; Bp = Cpid_ss.B; Cpm = Cpid_ss.C; Dp = Cpid_ss.D;

nx_p = size(A,1);
nx_s = size(As,1);
nx_r = size(Ar,1);
nx_y = size(Ay,1);
nx_pidx = size(Ap,1);

xp = zeros(nx_p,1);
xs = zeros(nx_s,1);
xr = zeros(nx_r,1);
xy = zeros(nx_y,1);
xpid = zeros(nx_pidx,1);

N = length(t);
y = zeros(N,1);
u = zeros(N,1);
mode_history = ones(N,1);
switch_times = [];

mode = 1;
if attack_flag && isfinite(detection_time)
    switch_index = find(t >= detection_time, 1, 'first');
    if isempty(switch_index)
        switch_index = N + 1;
    end
else
    switch_index = N + 1;
end

if ~exist('switcher_cfg','var') || isempty(switcher_cfg) || ~isfield(switcher_cfg,'blend_time')
    blend_time = 0.5;
else
    blend_time = switcher_cfg.blend_time;
end
if isfinite(detection_time)
    blend_end_time = detection_time + max(0, blend_time);
    blend_end_index = find(t >= blend_end_time, 1, 'first');
    if isempty(blend_end_index), blend_end_index = switch_index; end
else
    blend_end_time = NaN; blend_end_index = N + 1;
end
if ~exist('switcher_cfg','var') || isempty(switcher_cfg) || ~isfield(switcher_cfg,'recovery_time')
    recovery_time = 1.0;
else
    recovery_time = switcher_cfg.recovery_time;
end
if ~exist('switcher_cfg','var') || isempty(switcher_cfg) || ~isfield(switcher_cfg,'isolation_tau')
    isolation_tau = max(0.25, 0.5 * recovery_time);
else
    isolation_tau = max(eps, switcher_cfg.isolation_tau);
end
if ~exist('switcher_cfg','var') || isempty(switcher_cfg) || ~isfield(switcher_cfg,'observer_recovery_time')
    observer_recovery_time = max(1.0, recovery_time);
else
    observer_recovery_time = max(eps, switcher_cfg.observer_recovery_time);
end
if ~exist('switcher_cfg','var') || isempty(switcher_cfg) || ~isfield(switcher_cfg,'observer_innovation_limit')
    observer_innovation_limit = 0.05;
else
    observer_innovation_limit = max(eps, switcher_cfg.observer_innovation_limit);
end
if ~exist('switcher_cfg','var') || isempty(switcher_cfg) || ~isfield(switcher_cfg,'observer_min_gain')
    observer_min_gain = 0.02;
else
    observer_min_gain = min(max(switcher_cfg.observer_min_gain, 0), 1);
end
if ~exist('switcher_cfg','var') || isempty(switcher_cfg) || ~isfield(switcher_cfg,'actuator_limits')
    umax = 10; umin = -10;
else
    lim = switcher_cfg.actuator_limits;
    if numel(lim) == 2
        umin = lim(1); umax = lim(2);
    else
        umax = 10; umin = -10;
    end
end

[Aobs, Bobs, Cobs, Dobs, Lobs, observer_ok] = build_recovery_observer(plant_ss, sensor_ss);
if observer_ok
    zhat = zeros(size(Aobs,1),1);
else
    zhat = [];
end
attack_est = 0;
attack_est_hist = zeros(N,1);
y_hat_hist = zeros(N,1);
obs_gain_hist = zeros(N,1);
y_iso_hist = zeros(N,1);
u_comp_hist = zeros(N,1);
isolation_conf_hist = zeros(N,1);
recovery_initialized = false;
switch_recorded = false;

for k = 1:N
    if k == 1
        dt = t(1);
    else
        dt = t(k) - t(k-1);
    end

    u_prev = 0;
    if k > 1
        u_prev = u(k-1);
    end

    yk = C * xp + D * u_prev;
    if nx_s > 0
        y_s = Cs * xs + Ds * yk;
        xs = xs + (As * xs + Bs * yk) * dt;
    else
        y_s = yk;
    end
    y_meas = apply_attack_scalar(y_s, t(k), attack_cfg);

    if observer_ok
        y_hat = Cobs * zhat + Dobs * u_prev;
        innovation = y_meas - y_hat;
        innovation = max(min(innovation, 1e6), -1e6);
        if isfinite(detection_time) && t(k) >= detection_time
            iso_gain = min(1, dt / max(eps, isolation_tau));
            attack_est = (1 - iso_gain) * attack_est + iso_gain * innovation;
            y_iso = y_meas - attack_est;
            isolation_conf = min(1, abs(attack_est) / max(eps, abs(innovation) + observer_innovation_limit));
            y_ctrl = y_iso;
            obs_gain = max(observer_min_gain, 1 - max(0, t(k) - detection_time) / observer_recovery_time);
            mode = 3;
        else
            attack_est = 0;
            y_iso = y_meas;
            y_ctrl = y_meas;
            isolation_conf = 0;
            obs_gain = 1;
            mode = 1;
        end
        innovation_gain = min(1, observer_innovation_limit / max(observer_innovation_limit, abs(innovation)));
        obs_gain = max(observer_min_gain, obs_gain * innovation_gain);
        zhat = zhat + (Aobs * zhat + Bobs * u_prev + obs_gain * (Lobs * innovation)) * dt;
        attack_est_hist(k) = attack_est;
        y_hat_hist(k) = y_hat;
        obs_gain_hist(k) = obs_gain;
        y_iso_hist(k) = y_iso;
        isolation_conf_hist(k) = isolation_conf;
        u_comp_hist(k) = 0;
        if mode == 3 && ~recovery_initialized
            recovery_initialized = true;
        end
        if ~switch_recorded && isfinite(detection_time) && t(k) >= detection_time
            switch_times = [t(k), 1, 3];
            switch_recorded = true;
        end
    else
        if k < switch_index || ~isfinite(detection_time)
            y_ctrl = y_meas;
            mode = 1;
        elseif t(k) <= detection_time + recovery_time
            beta = (t(k) - detection_time) / max(eps, recovery_time);
            beta = min(max(beta, 0), 1);
            y_ctrl = (1 - beta) * y_meas + beta * yk;
            mode = 3;
        else
            y_ctrl = yk;
            mode = 3;
        end
        y_iso = y_ctrl;
        isolation_conf = 0;
        attack_est = y_meas - y_ctrl;
        u_comp_hist(k) = 0;
        y_iso_hist(k) = y_iso;
        isolation_conf_hist(k) = isolation_conf;
        if ~switch_recorded && isfinite(detection_time) && t(k) >= detection_time
            switch_times = [t(k), 1, 3];
            switch_recorded = true;
        end
    end

    mode_history(k) = mode;

    ur = Crm * xr + Dr * r(k);
    uy = Cym * xy + Dy * y_ctrl;
    epid = r(k) - y_ctrl;

    if mode == 3 && ~recovery_initialized
        xpid = align_controller_state(Cpid_ss, epid, u_prev, xpid, get_bumpless_reg(switcher_cfg));
        recovery_initialized = true;
    end

    pid_out = Cpm * xpid + Dp * epid;

    if mode == 3
        uk_unclamped = pid_out;
    else
        uk_unclamped = ur - uy;
    end

    uk = min(max(uk_unclamped, umin), umax);

    if ~isempty(Ar)
        xr = xr + (Ar * xr + Br * r(k)) * dt;
    end
    if ~isempty(Ay)
        xy = xy + (Ay * xy + By * y_ctrl) * dt;
    end
    if ~isempty(Ap)
        if abs(uk - uk_unclamped) < 1e-9
            xpid = xpid + (Ap * xpid + Bp * epid) * dt;
        else
            xpid = xpid + 0.1 * (Ap * xpid + Bp * epid) * dt;
        end
    end

    xp = xp + (A * xp + B * uk) * dt;
    y(k) = C * xp + D * uk;
    u(k) = uk;
end

if isempty(switch_times)
    switch_times = zeros(0,3);
end

diag = struct();
diag.attack_est_hist = attack_est_hist;
diag.y_hat_hist = y_hat_hist;
diag.obs_gain_hist = obs_gain_hist;
diag.y_iso_hist = y_iso_hist;
diag.u_comp_hist = u_comp_hist;
diag.isolation_conf_hist = isolation_conf_hist;
end

function reg = get_bumpless_reg(switcher_cfg)
reg = 1e-4;
if nargin >= 1 && isstruct(switcher_cfg) && isfield(switcher_cfg,'bumpless_reg') && ~isempty(switcher_cfg.bumpless_reg)
    reg = switcher_cfg.bumpless_reg;
end
end

function x = align_controller_state(ctrl_ss, input_value, desired_output, fallback_state, reg)
x = fallback_state;
try
    Cc = ctrl_ss.C;
    Dc = ctrl_ss.D;
    if isempty(Cc)
        return;
    end
    target = desired_output - Dc * input_value;
    if isempty(ctrl_ss.A)
        x = zeros(size(fallback_state));
        return;
    end
    if nargin < 5 || isempty(reg)
        reg = 1e-6;
    end
    if size(Cc,1) == 1
        denom = Cc * Cc.' + reg;
        x = (Cc.' / denom) * target;
    else
        Regl = reg * eye(size(Cc,2));
        x = (Cc.' * Cc + Regl) \ (Cc.' * target);
    end
    if any(~isfinite(x))
        x = fallback_state;
    end
catch
    x = fallback_state;
end
end

function [Aobs, Bobs, Cobs, Dobs, Lobs, ok] = build_recovery_observer(plant_ss, sensor_ss)
ok = false;
Aobs = []; Bobs = []; Cobs = []; Dobs = []; Lobs = [];
try
    plant_ss = ss(plant_ss);
    sensor_ss = ss(sensor_ss);

    A = plant_ss.A; B = plant_ss.B; C = plant_ss.C; D = plant_ss.D;
    As = sensor_ss.A; Bs = sensor_ss.B; Cs = sensor_ss.C; Ds = sensor_ss.D;

    nplant = size(A,1);
    nsensor = size(As,1);
    Aobs = [A, zeros(nplant, nsensor); Bs * C, As];
    Bobs = [B; Bs * D];
    Cobs = [Ds * C, Cs];
    Dobs = Ds * D;

    nobs = size(Aobs,1);
    if nobs == 0
        return;
    end

    pole_base = max(4, 2 * nobs);
    desired_poles = -pole_base - (0:nobs-1);
    try
        Lobs = place(Aobs', Cobs', desired_poles)';
    catch
        desired_poles = -max(2, nobs) - (0:nobs-1);
        Lobs = place(Aobs', Cobs', desired_poles)';
    end
    if any(~isfinite(Lobs(:)))
        return;
    end
    ok = true;
catch
    ok = false;
    Aobs = []; Bobs = []; Cobs = []; Dobs = []; Lobs = [];
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
switch lower(string(attack_cfg.type))
    case "bias"
        if isfield(attack_cfg, 'magnitude')
            y_attack = y + attack_cfg.magnitude;
        end
    case "ramp"
        if isfield(attack_cfg, 'slope')
            y_attack = y + attack_cfg.slope * (t - attack_cfg.start_time);
        end
    case "sine"
        amp = 0;
        freq = 1;
        if isfield(attack_cfg, 'magnitude'), amp = attack_cfg.magnitude; end
        if isfield(attack_cfg, 'frequency'), freq = attack_cfg.frequency; end
        y_attack = y + amp * sin(2*pi*freq*(t - attack_cfg.start_time));
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
