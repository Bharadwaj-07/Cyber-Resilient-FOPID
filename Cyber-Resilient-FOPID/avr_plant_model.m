%% avr_plant_model.m
% Builds AVR transfer functions and runs open-loop validation
% Run avr_parameters.m first

avr_parameters;

% --- Individual block transfer functions ---
G_amp  = tf(Ka,  [Ta  1]);   % amplifier
G_exc  = tf(Ke,  [Te  1]);   % exciter
G_gen  = tf(Kg,  [Tg  1]);   % generator
G_sen  = tf(Ks,  [Ts  1]);   % sensor

% --- Forward path: amplifier * exciter * generator ---
G_fwd  = G_amp * G_exc * G_gen;

% --- Open-loop transfer function (forward * feedback) ---
G_ol   = G_fwd * G_sen;

% --- Closed-loop transfer function (unity negative feedback) ---
% For now sensor is in feedback; use feedback() builtin
G_cl   = feedback(G_fwd, G_sen);

% --- Display poles and zeros ---
disp('=== Open-loop poles ===');
disp(pole(G_ol));

disp('=== Closed-loop poles ===');
disp(pole(G_cl));

% --- Stability check ---
p = pole(G_cl);
if all(real(p) < 0)
    disp('Plant closed-loop is STABLE (all poles in LHP)');
else
    disp('WARNING: unstable poles detected');
end

% --- Bode plot ---
figure('Name','AVR Open-Loop Bode');
margin(G_ol);
grid on;
title('Open-loop Bode — AVR plant (no controller)');

% --- Step response of raw closed-loop ---
figure('Name','AVR Closed-Loop Step (no controller)');
step(G_cl, Tfinal);
grid on;
title('Closed-loop step response — no controller');
ylabel('Terminal voltage Vt (pu)');
xlabel('Time (s)');