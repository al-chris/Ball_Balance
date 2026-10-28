%DESIGN_CONTROLLER  Analyse the LQI + Kalman ball-position controller (bb_controller.m)
% on the linear model, including the servo dynamics it was designed without.
% Prints gains, poles and stability margins, and saves results/controller_design.png.

here = fileparts(mfilename('fullpath'));
addpath(here);
bb = bb_params();
ct = bb.ctrl; Ts = ct.Ts; K = ct.K;

fprintf('Plate tilt per servo angle   : %.3f rad/rad\n', ct.kin);
fprintf('Ball accel per servo angle c : %.3f m/s^2/rad  ->  G(s) = -%.3f / s^2\n', ct.c, ct.c);
fprintf('LQI gains  K = [k_x k_v k_xi]: [%.3f  %.3f  %.3f]\n', K);
fprintf('Kalman innovation gain  M    : [%.4f  %.4f]\n', ct.M);

%% Closed loop of the design model (state feedback, lqi convention check)
Ad = ct.sysd.A; Bd = ct.sysd.B; Cd = ct.sysd.C;
Aaug = [Ad zeros(2,1); -Ts*Cd 1]; Baug = [Bd; 0];
pz = eig(Aaug - Baug*K);
ps = log(pz) / Ts;
fprintf('State-feedback poles (s-equiv): %s\n', mat2str(ps.', 3));
fprintf('Estimator poles (s-equiv)     : %s\n', mat2str((log(eig(ct.est.A))/Ts).', 3));

%% Full linear loop: servo lag + pivot-offset zeros, estimator, integrator, gains
fprintf('Full plant zeros (pivot offset d_eff = %.1f mm): +/-%.2f rad/s\n', 1000*ct.d_eff, sqrt(bb.g/ct.d_eff));
[T, Lu] = bb_linear_loop(ct);              % loop broken at the servo command
mg = allmargin(Lu);
gm = 20*log10(min(mg.GainMargin(mg.GainMargin > 1)));
gml = 20*log10(max(mg.GainMargin(mg.GainMargin < 1)));
[pm, ip] = min(mg.PhaseMargin);
fprintf('Loop at servo input: gain margin +%.1f dB / %.1f dB, phase margin %.1f deg at %.2f rad/s, delay margin %.0f ms\n', ...
    gm, gml, pm, mg.PMFrequency(ip), 1000*min(mg.DelayMargin)*Ts);
fprintf('Closed-loop stable (with servo): %d\n', isstable(T));

% Response to the smooth 50 mm move used in simulation (minimum jerk, 1 s)
tt = (0:Ts:3)';
R6 = interp1(bb.ref.r(:,1), bb.ref.r(:,2:7), 4 + tt);
ry = lsim(T, R6(:, [1 3 5]), tt);          % x axis: [x vx ax]
yy = ry(:,1); uy = ry(:,2);
info = stepinfo(yy, tt, R6(end,1), 'SettlingTimeThreshold', 0.02);
fprintf('Linear smooth 50 mm move: max tracking error %.2f mm, overshoot %.1f %%, settled(2%%) at %.2f s\n', ...
    1000*max(abs(R6(:,1) - yy)), info.Overshoot, info.SettlingTime);
fprintf('Peak servo command for the move: %.1f deg\n', rad2deg(max(abs(uy))));
sr = stepinfo(T(1,1), 'SettlingTimeThreshold', 0.02);
fprintf('Raw 50 mm step (no shaping, for reference): settling %.2f s, overshoot %.1f %%\n', sr.SettlingTime, sr.Overshoot);

%% Plots
f = figure('Color', 'w', 'Position', [100 100 1150 750], 'Visible', 'off');
tl = tiledlayout(f, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, 'Ball-position LQI + Kalman controller (linear analysis)');
nexttile; plot(tt, 1000*R6(:,1), 'k--', tt, 1000*yy, 'LineWidth', 1.6); grid on
xlabel('t (s)'); ylabel('ball position (mm)'); title('Smooth 50 mm move (linear model)'); legend('reference', 'ball', 'Location', 'southeast');
nexttile; stairs(tt, rad2deg(uy), 'LineWidth', 1.6); grid on
xlabel('t (s)'); ylabel('servo command (deg)'); title('Servo command for the move');
nexttile; w = logspace(-1, log10(pi/Ts), 400);
[mag, ph] = bode(Lu, w); mag = squeeze(mag); ph = squeeze(ph);
yyaxis left; semilogx(w, 20*log10(mag), 'LineWidth', 1.4); ylabel('|L| (dB)'); yline(0, ':');
yyaxis right; semilogx(w, ph, 'LineWidth', 1.4); ylabel('\angle L (deg)'); yline(-180, ':');
grid on; xlabel('\omega (rad/s)');
title(sprintf('Loop at servo input: PM %.0f°, GM +%.1f/%.1f dB', pm, gm, gml));
nexttile; pzplot(T(1,1)); title('Closed-loop poles/zeros (r \rightarrow y)'); grid on
resDir = fullfile(here, 'results'); if ~exist(resDir, 'dir'), mkdir(resDir); end
exportgraphics(f, fullfile(resDir, 'controller_design.png'), 'Resolution', 150);
fprintf('Saved %s\n', fullfile(resDir, 'controller_design.png'));
