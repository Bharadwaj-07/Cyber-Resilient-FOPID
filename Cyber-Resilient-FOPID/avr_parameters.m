%% avr_parameters.m
% IEEE Type-1 AVR plant parameters
% All values from standard benchmark literature (Gaing 2004, Pan & Das 2016)

% --- Amplifier ---
Ka = 10;      % gain
Ta = 0.1;     % time constant (s)

% --- Exciter ---
Ke = 1.0;     % gain
Te = 0.4;     % time constant (s)

% --- Generator ---
Kg = 1.0;     % gain
Tg = 1.0;     % time constant (s)

% --- Sensor ---
Ks = 1.0;     % gain (usually 1)
Ts = 0.01;    % time constant (s) — sensor is fast

% --- Simulation settings ---
Tfinal = 10;       % simulation duration (s)
Ts_sim = 0.001;    % sample time for discrete checks

disp('AVR parameters loaded.');