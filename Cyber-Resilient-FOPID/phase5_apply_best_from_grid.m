% phase5_apply_best_from_grid.m
% Select best recovery params from phase5_grid_search_results.csv and write phase5_config.mat

paths5 = phase_artifacts('phase5');
csvp = fullfile(paths5.csv, 'phase5_grid_search_results.csv');
outmat = fullfile(paths5.root, 'phase5_config.mat');
if ~exist(csvp,'file')
    error('Grid results not found: %s', csvp);
end

T = readtable(csvp);
required = {'blend_time','recovery_time','bumpless_reg','isolation_tau','actuator_limits'};
required = [required, {'Q_scale','R_scale'}];
missing = required(~ismember(required, T.Properties.VariableNames));
if ~isempty(missing)
    error('Grid results missing required columns: %s', strjoin(missing, ', '));
end

limits_key = cellfun(@(c) sprintf('%g_%g', c(1), c(2)), T.actuator_limits, 'UniformOutput', false);
T.key = strcat(string(T.blend_time), '_', string(T.recovery_time), '_', string(T.bumpless_reg), '_', string(T.isolation_tau), '_', string(limits_key));

[G, key_list] = findgroups(T.key);
mean_itae = splitapply(@mean, T.itae_res, G);
mean_uj = splitapply(@mean, T.u_jump, G);
mean_up = splitapply(@mean, T.u_peak_rate, G);

mean_itae(~isfinite(mean_itae)) = max(1e6, nanmax(mean_itae(isfinite(mean_itae))) * 10);
mean_uj(~isfinite(mean_uj)) = nanmax(mean_uj(isfinite(mean_uj)));
mean_up(~isfinite(mean_up)) = nanmax(mean_up(isfinite(mean_up)));

fnorm = @(x) (x - min(x)) ./ max(eps, (max(x) - min(x)));
score = 0.6 * fnorm(mean_itae) + 0.2 * fnorm(mean_uj) + 0.2 * fnorm(mean_up);
[~, best_idx] = min(score);
best_key = string(key_list(best_idx));
fprintf('Selected best key: %s (score=%g)\n', best_key, score(best_idx));

rowidx = find(T.key == best_key, 1, 'first');
chosen = T(rowidx,:);
best_cfg.blend_time = chosen.blend_time;
best_cfg.recovery_time = chosen.recovery_time;
best_cfg.bumpless_reg = chosen.bumpless_reg;
best_cfg.isolation_tau = chosen.isolation_tau;
best_cfg.actuator_limits = chosen.actuator_limits{1};
if ismember('Q_scale', T.Properties.VariableNames)
    best_cfg.Q_scale = chosen.Q_scale;
end
if ismember('R_scale', T.Properties.VariableNames)
    best_cfg.R_scale = chosen.R_scale;
end

% Write both a user-editable config and a locked config that Phase5 will
% prioritize. The locked config ensures the selected settings are applied
% consistently across runs until manually changed.
save(outmat, '-struct', 'best_cfg');
lockedmat = fullfile(paths5.root, 'phase5_locked.mat');
save(lockedmat, '-struct', 'best_cfg');
fprintf('Wrote Phase5 config to %s and locked config to %s\n', outmat, lockedmat);
