function [C_r, C_y] = fopid_2dof(Kp, Ki, Kd, lambda, mu, b, c, wb, wh, N)
%FOPID_2DOF  Two-degree-of-freedom Fractional-Order PID controller
%
%   [C_r, C_y] = fopid_2dof(Kp, Ki, Kd, lambda, mu, b, c, wb, wh, N)
%
%   Parameters:
%     Kp, Ki, Kd  : proportional, integral, derivative gains
%     lambda      : fractional integration order (0 < lambda < 2)
%     mu          : fractional differentiation order (0 < mu < 2)
%     b           : setpoint weight on proportional term (0 <= b <= 1)
%     c           : setpoint weight on derivative term  (0 <= c <= 1)
%     wb, wh, N   : Oustaloup approximation parameters
%
%   Returns:
%     C_r : reference (setpoint) path transfer function
%     C_y : output (feedback) path transfer function
%
%   Controller law in time domain:
%     u(t) = Kp*(b*r - y) + Ki*I^lambda*(r - y) + Kd*(c*r_dot - y_dot)
%
%   In transfer function form:
%     U(s) = C_r(s)*R(s) - C_y(s)*Y(s)

if nargin < 10, N  = 5;    end
if nargin < 9,  wh = 1e4;  end
if nargin < 8,  wb = 1e-4; end

% Fractional integrator: s^(-lambda)
s = tf('s');
if abs(lambda - 1) < 1e-2
	I_lambda = 1 / s;
else
	I_lambda = fopid_operator(-lambda, wb, wh, N);
end

% Fractional differentiator: s^(mu)
if abs(mu - 1) < 1e-2
	D_mu = s;
else
	D_mu = fopid_operator(mu, wb, wh, N);
end

% Derivative filter
tau_f = 0.01;
D = (Kd * D_mu) / (1 + tau_f*s);

% Reference path: C_r(s) = Kp*b + Ki*s^(-lambda) + c*D
C_r = Kp*b + Ki*I_lambda + c*D;

% Output/feedback path: C_y(s) = Kp + Ki*s^(-lambda) + D
% (full controller on the feedback; b,c only modify reference path)
C_y = Kp + Ki*I_lambda + D;

% Reduce order slightly to keep state count manageable
C_r = minreal(C_r, 1e-3);
C_y = minreal(C_y, 1e-3);
end