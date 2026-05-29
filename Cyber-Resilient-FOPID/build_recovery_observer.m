function [Aobs, Bobs, Cobs, Dobs, Lobs, ok] = build_recovery_observer(plant_ss, sensor_ss, cfg)
% BUILD_RECOVERY_OBSERVER Construct observer for plant+sensor cascade.
% Tries an LQR-dual (LQE-like) design using Q/R scales from cfg, falls
% back to pole placement if LQR fails.
ok = false;
Aobs = []; Bobs = []; Cobs = []; Dobs = []; Lobs = [];
try
    if nargin < 3, cfg = struct(); end
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

    % Get Q/R scales from cfg if present
    qscale = 1.0; rscale = 1.0;
    if isstruct(cfg)
        if isfield(cfg,'Q_scale'), qscale = cfg.Q_scale; end
        if isfield(cfg,'R_scale'), rscale = cfg.R_scale; end
    end
    Qn = max(eps, qscale) * eye(nobs);
    Rn = max(eps, rscale) * eye(size(Cobs,1));

    % Try LQR dual: L = lqr(A', C', Qn, Rn)'
    try
        Lobs = lqr(Aobs', Cobs', Qn, Rn)';
    catch
        % fallback: pole placement
        pole_base = max(4, 2 * nobs);
        desired_poles = -pole_base - (0:nobs-1);
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
