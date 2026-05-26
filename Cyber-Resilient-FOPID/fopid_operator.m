function G = fopid_operator(alpha, wb, wh, N)
%FOPID_OPERATOR  Oustaloup recursive approximation of s^alpha
%
%   G = fopid_operator(alpha, wb, wh, N)
%
%   alpha : fractional order (positive = differentiator, negative = integrator)
%   wb    : lower fitting frequency (rad/s), typically 1e-4
%   wh    : upper fitting frequency (rad/s), typically 1e4
%   N     : approximation order (integer), typically 5 or 7
%
%   Returns a MATLAB tf object approximating s^alpha over [wb, wh].
%
%   Reference: Oustaloup et al., IEEE T-CST, 2000.

if nargin < 4, N  = 5;    end
if nargin < 3, wh = 1e4;  end
if nargin < 2, wb = 1e-4; end

if wb >= wh
    error('fopid_operator:InvalidBand', 'wb must be less than wh.');
end

% Frequency ratio between successive poles/zeros
mu = (wh / wb)^(1 / (2*N + 1));

% Initialise arrays
zeros_k = zeros(1, N);
poles_k = zeros(1, N);

for k = 0 : N-1
    % Zero frequencies
    zeros_k(k+1) = -wb * mu^(2*k + 1 - alpha);
    % Pole frequencies
    poles_k(k+1) = -wb * mu^(2*k + 1 + alpha);
end

% High-frequency gain correction
K = wh^alpha * prod(-poles_k) / prod(-zeros_k);

% Build transfer function from zeros and poles
G = tf(K * poly(zeros_k), poly(poles_k));
end