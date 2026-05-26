function [best_params, best_ITAE, history] = pso_tuner(G_plant, G_sensor, options)
%PSO_TUNER  Particle Swarm Optimisation for 2DoF FOPID parameter tuning
%
%   [best_params, best_ITAE, history] = pso_tuner(G_plant, G_sensor, options)
%
%   Inputs:
%     G_plant   : forward path plant tf (amp * exciter * generator)
%     G_sensor  : sensor tf
%     options   : struct with fields (all optional):
%                   .n_particles  (default 30)
%                   .max_iter     (default 100)
%                   .w            (default 0.7)   inertia weight
%                   .c1           (default 1.5)   cognitive coefficient
%                   .c2           (default 1.5)   social coefficient
%                   .Tfinal       (default 10)    simulation horizon (s)
%                   .bounds       struct with fields:
%                                  .lb, .ub (overrides default bounds)
%                   .seed         initial parameter guess (overrides first particle)
%                   .frac         struct with fields:
%                                  .wb, .wh, .N (Oustaloup settings)
%                   .fixed_bc     (default false) fix b=c=1.0 (1DoF tuning)
%                   .eval         struct with fields:
%                                  .settle_threshold (default 0.02)
%                                  .max_settle       (default 10)
%                                  .max_rise         (default 2)
%                                  .settle_weight    (default 50)
%                                  .rise_weight      (default 20)
%                                  .ss_weight        (default 200)
%
%   Returns:
%     best_params : [Kp Ki Kd lambda mu b c]
%     best_ITAE   : final ITAE value
%     history     : ITAE per iteration (for convergence plot)

    % --- Default options ---
    if nargin < 3, options = struct(); end
    n  = getopt(options, 'n_particles', 30);
    MI = getopt(options, 'max_iter',   100);
    w  = getopt(options, 'w',          0.72);
    c1 = getopt(options, 'c1',         1.49);
    c2 = getopt(options, 'c2',         1.49);
    Tf = getopt(options, 'Tfinal',      10);
    fixed_bc = getopt(options, 'fixed_bc', false);
    eval_cfg = getopt(options, 'eval', struct());
    eval_cfg = normalize_eval_cfg(eval_cfg);
    frac_cfg = getopt(options, 'frac', struct());
    frac_cfg = normalize_frac_cfg(frac_cfg);
    seed = getopt(options, 'seed', []);

    % --- Parameter bounds [Kp, Ki, Kd, lambda, mu, b, c] ---
    lb = [0.01, 0.01, 0.01, 0.01, 0.01, 0.0, 0.0];
    ub = [5.00, 5.00, 5.00, 2.00, 2.00, 1.0, 1.0];
    if isfield(options, 'bounds')
        if isfield(options.bounds, 'lb'), lb = options.bounds.lb; end
        if isfield(options.bounds, 'ub'), ub = options.bounds.ub; end
    end

    if fixed_bc
        n_params = 5;
        lb = lb(1:5);
        ub = ub(1:5);
    else
        n_params = 7;
    end

    % --- Initialise swarm ---
    pos = lb + rand(n, n_params) .* (ub - lb);   % positions
        if ~isempty(seed)
            if fixed_bc
                seed = seed(1:5);
            end
            if numel(seed) == n_params
                pos(1,:) = max(lb, min(ub, seed));
            end
        end
    vel = zeros(n, n_params);                      % velocities
    pbest     = pos;                               % personal bests
    pbest_val = inf(n, 1);
    gbest     = pos(1,:);
    gbest_val = inf;

    history = zeros(MI, 1);
    t_eval  = (0 : 0.001 : Tf)';
    vmax = 0.15 * (ub - lb);

    fprintf('Starting PSO: %d particles, %d iterations\n', n, MI);

    for iter = 1 : MI
        for i = 1 : n
            p = pos(i,:);
            itae = evaluate_2dof_fopid(p, G_plant, G_sensor, t_eval, fixed_bc, eval_cfg, frac_cfg);

            if itae < pbest_val(i)
                pbest_val(i) = itae;
                pbest(i,:)   = p;
            end
            if itae < gbest_val
                gbest_val = itae;
                gbest     = p;
            end
        end

        % --- Update velocities and positions ---
        r1 = rand(n, n_params);
        r2 = rand(n, n_params);
        vel = w*vel ...
            + c1*r1.*(pbest - pos) ...
            + c2*r2.*(gbest  - pos);

        vel = max(-vmax, min(vmax, vel));

        pos = pos + vel;
        pos = max(lb, min(ub, pos));   % clamp to bounds

        history(iter) = gbest_val;

        if mod(iter, 10) == 0
            fprintf('  Iter %3d/%d  |  best ITAE = %.5f\n', ...
                iter, MI, gbest_val);
        end
    end

    if fixed_bc
        best_params = [gbest, 1.0, 1.0];
    else
        best_params = gbest;
    end
    best_ITAE   = gbest_val;
    fprintf('\nPSO complete. Best ITAE = %.5f\n', best_ITAE);
    if fixed_bc
        fprintf('Kp=%.4f Ki=%.4f Kd=%.4f lambda=%.4f mu=%.4f b=%.4f c=%.4f\n', ...
            gbest(1), gbest(2), gbest(3), gbest(4), gbest(5), 1.0, 1.0);
    else
        fprintf('Kp=%.4f Ki=%.4f Kd=%.4f lambda=%.4f mu=%.4f b=%.4f c=%.4f\n', ...
            gbest(1), gbest(2), gbest(3), gbest(4), gbest(5), gbest(6), gbest(7));
    end
end

% -----------------------------------------------------------------------
function cost = evaluate_2dof_fopid(params, G_plant, G_sensor, t, fixed_bc, eval_cfg, frac_cfg)
%EVALUATE_2DOF_FOPID  Compute cost for one particle's parameter vector

    if nargin < 5
        fixed_bc = false;
    end
    if nargin < 6
        eval_cfg = normalize_eval_cfg(struct());
    else
        eval_cfg = normalize_eval_cfg(eval_cfg);
    end
    if nargin < 7
        frac_cfg = normalize_frac_cfg(struct());
    end

    Kp  = params(1);
    Ki  = params(2);
    Kd  = params(3);
    lam = params(4);
    mu  = params(5);

    if fixed_bc
        b = 1.0;
        c = 1.0;
    else
        b = params(6);
        c = params(7);
    end

    LARGE = 1e8;
    persistent debug_hits;
    if isempty(debug_hits)
        debug_hits = 0;
    end

    try

        % ---------------------------------------------------------
        % Build 2DoF FOPID
        % ---------------------------------------------------------
        [C_r, C_y] = fopid_2dof( ...
            Kp, Ki, Kd, lam, mu, b, c, ...
            frac_cfg.wb, frac_cfg.wh, frac_cfg.N);

        % ---------------------------------------------------------
        % 2DoF closed-loop (correct form)
        % T(s) = G*C_r / (1 + G*C_y*H)
        % ---------------------------------------------------------
        den = 1 + G_plant * C_y * G_sensor;
        G_cl = minreal((G_plant * C_r) / den, 1e-3);

        % ---------------------------------------------------------
        % Stability check
        % ---------------------------------------------------------
        p = pole(G_cl);

        if any(real(p) > 0)
            if isfield(eval_cfg, 'debug') && eval_cfg.debug && debug_hits < 3
                debug_hits = debug_hits + 1;
                fprintf('DEBUG: unstable poles, max real = %.4g\n', max(real(p)));
            end
            cost = LARGE;
            return;
        end

        % ---------------------------------------------------------
        % Step response
        % ---------------------------------------------------------
        [y, t_out] = step(G_cl, t);

        if any(isnan(y)) || any(isinf(y))
            if isfield(eval_cfg, 'debug') && eval_cfg.debug && debug_hits < 3
                debug_hits = debug_hits + 1;
                fprintf('DEBUG: step() produced NaN/Inf\n');
            end
            cost = LARGE;
            return;
        end

        % ---------------------------------------------------------
        % Performance metrics
        % ---------------------------------------------------------
        e = 1 - y;

        ITAE = trapz(t_out, t_out .* abs(e));

        info = stepinfo(y, t_out, 'SettlingTimeThreshold', eval_cfg.settle_threshold);

        if ~isfinite(info.SettlingTime) || isempty(info.SettlingTime)
            info.SettlingTime = t_out(end);
        end
        if ~isfinite(info.RiseTime) || isempty(info.RiseTime)
            info.RiseTime = t_out(end);
        end

        overshoot = info.Overshoot;
        if ~isfinite(overshoot)
            overshoot = 0;
        end
        overshoot = max(0, overshoot);

        if max(abs(y)) > 1e3
            if isfield(eval_cfg, 'debug') && eval_cfg.debug && debug_hits < 3
                debug_hits = debug_hits + 1;
                fprintf('DEBUG: response blew up, max |y| = %.4g\n', max(abs(y)));
            end
            cost = LARGE;
            return;
        end

        ss_error = abs(1 - y(end));

        % ---------------------------------------------------------
        % Penalties
        % ---------------------------------------------------------
        settle_penalty = 0;
        rise_penalty   = 0;

        if info.SettlingTime > eval_cfg.max_settle
            settle_penalty = 100 * ...
                (info.SettlingTime - eval_cfg.max_settle)^2;
        end

        if info.RiseTime > eval_cfg.max_rise
            rise_penalty = 50 * ...
                (info.RiseTime - eval_cfg.max_rise)^2;
        end

        if isempty(eval_cfg.target_os)
            overshoot_penalty = eval_cfg.os_weight * overshoot^2;
        else
            overshoot_penalty = eval_cfg.os_weight * (overshoot - eval_cfg.target_os)^2;
        end

        ss_penalty = 1000 * ss_error^2;

        % ---------------------------------------------------------
        % Final objective
        % ---------------------------------------------------------
        cost = ITAE ...
             + settle_penalty ...
             + rise_penalty ...
             + overshoot_penalty ...
             + ss_penalty;

        if ~isfinite(cost)
            cost = LARGE;
        end

    catch ME
        if isfield(eval_cfg, 'debug') && eval_cfg.debug && debug_hits < 3
            debug_hits = debug_hits + 1;
            fprintf('DEBUG: evaluation error: %s\n', ME.message);
        end
        cost = LARGE;
    end
end

% -----------------------------------------------------------------------
function val = getopt(s, field, default)
    if isfield(s, field)
        val = s.(field);
    else
        val = default;
    end
end

% -----------------------------------------------------------------------
function cfg = normalize_eval_cfg(cfg)
    if ~isfield(cfg, 'settle_threshold'), cfg.settle_threshold = 0.02; end
    if ~isfield(cfg, 'max_settle'),       cfg.max_settle = 10; end
    if ~isfield(cfg, 'max_rise'),         cfg.max_rise = 2; end
    if ~isfield(cfg, 'settle_weight'),    cfg.settle_weight = 50; end
    if ~isfield(cfg, 'rise_weight'),      cfg.rise_weight = 20; end
    if ~isfield(cfg, 'ss_weight'),        cfg.ss_weight = 200; end
    if ~isfield(cfg, 'target_os'),        cfg.target_os = []; end
    if ~isfield(cfg, 'os_weight'),        cfg.os_weight = 5; end
    if ~isfield(cfg, 'debug'),            cfg.debug = false; end
end

% -----------------------------------------------------------------------
function cfg = normalize_frac_cfg(cfg)
    if ~isfield(cfg, 'wb'), cfg.wb = 1e-2; end
    if ~isfield(cfg, 'wh'), cfg.wh = 1e2; end
    if ~isfield(cfg, 'N'),  cfg.N  = 3; end
end