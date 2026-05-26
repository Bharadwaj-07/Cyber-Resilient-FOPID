function y_meas = avr_attack_injector(y_true, t, attack_config)
% AVR_ATTACK_INJECTOR Add attack signal to true measurement
% y_meas = avr_attack_injector(y_true, t, attack_config)
%
% attack_config fields:
%   .enabled (bool)
%   .type ('bias'|'ramp'|'sine')
%   .magnitude (float)  % bias magnitude or sine amplitude
%   .slope (float)      % for ramp
%   .frequency (float)  % for sine
%   .start_time (float) % attack start time
%
if nargin < 3 || isempty(attack_config)
    attack_config.enabled = false;
end

if ~isfield(attack_config,'enabled')
    attack_config.enabled = false;
end
if ~isfield(attack_config,'type')
    attack_config.type = 'bias';
end
if ~isfield(attack_config,'magnitude')
    attack_config.magnitude = 0.1;
end
if ~isfield(attack_config,'slope')
    attack_config.slope = 0.05;
end
if ~isfield(attack_config,'frequency')
    attack_config.frequency = 1.0;
end
if ~isfield(attack_config,'start_time')
    attack_config.start_time = 5.0;
end

% Ensure column vectors
t = t(:); y_true = y_true(:);
N = length(t);
if length(y_true) ~= N
    error('t and y_true must have same length');
end

a = zeros(N,1);
if ~attack_config.enabled
    y_meas = y_true;
    return;
end

idx = find(t >= attack_config.start_time, 1);
if isempty(idx)
    % attack starts after simulation end
    y_meas = y_true;
    return;
end

switch lower(attack_config.type)
    case 'bias'
        a(idx:end) = attack_config.magnitude;
    case 'ramp'
        a(idx:end) = attack_config.slope * (t(idx:end) - t(idx));
    case 'sine'
        phase = t(idx:end) - t(idx);
        a(idx:end) = attack_config.magnitude * sin(2*pi*attack_config.frequency .* phase);
    otherwise
        error('Unknown attack type: %s', attack_config.type);
end

y_meas = y_true + a;
end
