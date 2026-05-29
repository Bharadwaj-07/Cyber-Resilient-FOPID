% phase5_apply_best_from_grid.m
% Select best recovery params from phase5_grid_search_results.csv and write phase5_config.mat

paths5 = phase_artifacts('phase5');
csvp = fullfile(paths5.csv, 'phase5_grid_search_results.csv');
outmat = fullfile(paths5.root, 'phase5_config.mat');
lockedmat = fullfile(paths5.root, 'phase5_locked.mat');

% If a locked config already exists, reuse it directly instead of re-selecting.
if exist(lockedmat, 'file')
    try
        locked = load(lockedmat);
        if isfield(locked, 'best_cfg') && isstruct(locked.best_cfg)
            best_cfg = locked.best_cfg;
        else
            best_cfg = locked;
        end
        save(outmat, '-struct', 'best_cfg');
        fprintf('Reused locked Phase5 config from %s and wrote %s\n', lockedmat, outmat);
        return;
    catch lockedErr
        fprintf('Locked Phase5 config could not be reused (%s); falling back to grid selection.\n', lockedErr.message);
    end
end

if ~exist(csvp,'file')
    error('Grid results not found: %s', csvp);
end

T = readtable(csvp);
% If a MAT version of the grid exists, prefer loading it to avoid re-parsing CSV quirks
matp_grid = fullfile(paths5.mat, 'phase5_grid_search_results.mat');
if exist(matp_grid,'file')
    try
        data = load(matp_grid);
        if isfield(data,'T') && istable(data.T)
            T = data.T;
        elseif isfield(data,'results')
            try
                T = struct2table(data.results);
            catch
            end
        elseif isfield(data,'grid_results')
            try
                T = struct2table(data.grid_results);
            catch
            end
        end
    catch
    end
end
required = {'blend_time','recovery_time','bumpless_reg','isolation_tau'};
required = [required, {'Q_scale','R_scale'}];
missing = required(~ismember(required, T.Properties.VariableNames));
if ~isempty(missing)
    error('Grid results missing required columns: %s', strjoin(missing, ', '));
end

% Support two CSV formats: a single cell-array column `actuator_limits`
% or two numeric columns `actuator_limits_1` and `actuator_limits_2`.
if ismember('actuator_limits', T.Properties.VariableNames)
    raw = T.actuator_limits;
    % Normalize to cell array of two-element numeric vectors
    nrows = height(T);
    limits_col = cell(nrows,1);
    if iscell(raw)
        for i=1:nrows
            v = raw{i};
            if isnumeric(v) && numel(v)>=2
                limits_col{i} = double(v(1:2));
            elseif ischar(v) || isstring(v)
                nums = regexp(char(v),'(-?\d+\.?\d*)','match');
                if numel(nums)>=2
                    limits_col{i} = [str2double(nums{1}), str2double(nums{2})];
                elseif numel(nums)==1
                    limits_col{i} = [str2double(nums{1}), str2double(nums{1})];
                else
                    limits_col{i} = [-inf, inf];
                end
            else
                limits_col{i} = [-inf, inf];
            end
        end
    elseif isstring(raw) || ischar(raw)
        for i=1:nrows
            s = char(raw(i));
            nums = regexp(s,'(-?\d+\.?\d*)','match');
            if numel(nums)>=2
                limits_col{i} = [str2double(nums{1}), str2double(nums{2})];
            elseif numel(nums)==1
                limits_col{i} = [str2double(nums{1}), str2double(nums{1})];
            else
                limits_col{i} = [-inf, inf];
            end
        end
    elseif isnumeric(raw)
        % numeric array: if Nx2, split into rows; if Nx1, duplicate
        if size(raw,2) >= 2
            for i=1:nrows, limits_col{i} = double(raw(i,1:2)); end
        else
            for i=1:nrows, limits_col{i} = [double(raw(i)), double(raw(i))]; end
        end
    else
        for i=1:nrows, limits_col{i} = [-inf, inf]; end
    end
elseif ismember('actuator_limits_1', T.Properties.VariableNames) && ismember('actuator_limits_2', T.Properties.VariableNames)
    % combine numeric columns into cell array of [min max]
    nrows = height(T);
    limits_col = cell(nrows,1);
    for i=1:nrows
        a = T.actuator_limits_1(i); b = T.actuator_limits_2(i);
        limits_col{i} = [double(a), double(b)];
    end
else
    error('Grid results missing actuator limits columns (expected actuator_limits or actuator_limits_1/2)');
end


% defensive check: ensure parsed limits match table rows; rebuild if mismatch
if numel(limits_col) ~= height(T)
    if ismember('actuator_limits_1', T.Properties.VariableNames) && ismember('actuator_limits_2', T.Properties.VariableNames)
        nrows = height(T);
        limits_col = cell(nrows,1);
        for i=1:nrows, limits_col{i} = [double(T.actuator_limits_1(i)), double(T.actuator_limits_2(i))]; end
    else
        nrows = height(T);
        limits_col = repmat({[-inf, inf]}, nrows, 1);
    end
end

limits_key = cellfun(@(c) sprintf('%g_%g', c(1), c(2)), limits_col, 'UniformOutput', false);
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
% pick actuator limits from the assembled limits_col
best_cfg.actuator_limits = limits_col{rowidx};
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
save(lockedmat, '-struct', 'best_cfg');
fprintf('Wrote Phase5 config to %s and locked config to %s\n', outmat, lockedmat);
