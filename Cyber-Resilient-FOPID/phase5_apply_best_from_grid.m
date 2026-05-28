% phase5_apply_best_from_grid.m
% Select best recovery params from phase5_grid_search_results.csv and write phase5_config.mat

paths5 = phase_artifacts('phase5');
csvp = fullfile(paths5.csv, 'phase5_grid_search_results.csv');
outmat = fullfile(paths5.root, 'phase5_config.mat');
if ~exist(csvp,'file')
    error('Grid results not found: %s', csvp);
end

T = readtable(csvp);
% Aggregate per unique parameter set (blend,recovery,reg,limits) across scenarios
% Build a key
keys = strcat(string(T.blend_time),'_',string(T.recovery_time),'_',string(T.bumpless_reg),'_',string(cellfun(@(c) sprintf('%g_%g',c(1),c(2)), T.actuator_limits,'UniformOutput',false)));
T.key = keys;
G = findgroups(T.key);
S = splitapply(@(itae,yf,uj,up) deal(nanmean(itae), nanmean(yf), nanmean(uj), nanmean(up)), T.itae_res, T.y_res_final, T.u_jump, T.u_peak_rate, G);

agg = table();
uniq = unique(keys);
agg.key = uniq(:);
agg.mean_itae = S(:,1); agg.mean_yfinal = S(:,2); agg.mean_ujump = S(:,3); agg.mean_upeak = S(:,4);

% Normalize metrics (robust to NaNs)
score_table = table(); score_table.key = agg.key;
m_itae = agg.mean_itae; m_uj = agg.mean_ujump; m_up = agg.mean_upeak;
% replace NaN with large numbers for ITAE, and large for u metrics
m_itae(isnan(m_itae)) = max(1e6, nanmax(m_itae(~isnan(m_itae)))*10);
m_uj(isnan(m_uj)) = nanmax(m_uj(~isnan(m_uj))); m_up(isnan(m_up)) = nanmax(m_up(~isnan(m_up)));

% Min-max normalize (lower is better)
fnorm = @(x) (x - nanmin(x)) ./ max(eps, (nanmax(x) - nanmin(x)));
norm_itae = fnorm(m_itae);
norm_uj = fnorm(m_uj);
norm_up = fnorm(m_up);

% Weighted score: prioritize ITAE (0.6) then u_jump (0.2) and u_peak_rate (0.2)
score = 0.6 * norm_itae + 0.2 * norm_uj + 0.2 * norm_up;

[~,best_idx] = min(score);
best_key = string(agg.key(best_idx));
fprintf('Selected best key: %s (score=%g)\n', best_key, score(best_idx));

% Extract parameters from a matching row in original table
rowidx = find(T.key == best_key, 1, 'first');
chosen = T(rowidx,:);
best_cfg.blend_time = chosen.blend_time;
best_cfg.recovery_time = chosen.recovery_time;
best_cfg.bumpless_reg = chosen.bumpless_reg;
best_cfg.actuator_limits = chosen.actuator_limits{1};

% Write both a user-editable config and a locked config that Phase5 will
% prioritize. The locked config ensures the selected settings are applied
% consistently across runs until manually changed.
save(outmat, '-struct', 'best_cfg');
lockedmat = fullfile(paths5.root, 'phase5_locked.mat');
save(lockedmat, '-struct', 'best_cfg');
fprintf('Wrote Phase5 config to %s and locked config to %s\n', outmat, lockedmat);
