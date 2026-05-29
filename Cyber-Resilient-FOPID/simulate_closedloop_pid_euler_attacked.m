function y = simulate_closedloop_pid_euler_attacked(plant_ss, sensor_ss, C_pid, t, r, attack_cfg)
% Closed-loop PID simulation with attack injected on the measured signal.
plant_ss = ss(plant_ss);
sensor_ss = ss(sensor_ss);
A = plant_ss.A; B = plant_ss.B; C = plant_ss.C; D = plant_ss.D;
As = sensor_ss.A; Bs = sensor_ss.B; Cs = sensor_ss.C; Ds = sensor_ss.D;

Cc_ss = safe_controller_ss(C_pid, plant_ss);
Ac = Cc_ss.A; Bc = Cc_ss.B; Cc = Cc_ss.C; Dc = Cc_ss.D;

nx_p = size(A,1);
nx_s = size(As,1);
nx_c = size(Ac,1);

xp = zeros(nx_p,1);
xs = zeros(nx_s,1);
xc = zeros(nx_c,1);

N = length(t);
y = zeros(N,1);
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
    e = r(k) - y_meas;
    u_unclamped = Cc * xc + Dc * e;
    umax = 10; umin = -10;
    uk = min(max(u_unclamped, umin), umax);
    if abs(uk - u_unclamped) < 1e-9
        xc = xc + (Ac * xc + Bc * e) * dt;
    else
        xc = xc + 0.1 * (Ac * xc + Bc * e) * dt;
    end
    xp = xp + (A * xp + B * uk) * dt;

    y(k) = C * xp + D * uk;
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
