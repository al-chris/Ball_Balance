%RUN_CLOSED_LOOP  Simulate ball_balance_cl (LQI + Kalman on the Simscape plant).
% Scenario (bb.ref): 0-4 s regulate from the initial offset to the centre,
% 4-8 s step to (50, 30) mm, 8-16 s track a 40 mm circle with a 4 s period.
% Compares the nonlinear step response with the linear-model prediction and
% saves results/closed_loop.png.

here = fileparts(mfilename('fullpath'));
addpath(here);
bb  = bb_params();
mdl = 'ball_balance_cl';
load_system(fullfile(here, [mdl '.slx']));

tic; out = sim(mdl); tsim = toc;
fprintf('Simulated %.1f s in %.1f s wall time (first run includes model compile)\n', bb.ref.T, tsim);

el  = @(n) out.logsout.getElement(n).Values;
cols = @(d, n) reshape(permute(d, [find(size(d) ~= n, 1) find(size(d) == n, 1)]), [], n);
pos = el('ball_pos');  t = pos.Time;  p = cols(squeeze(pos.Data), 3);
ref = el('ref');       r = interp1(ref.Time, cols(squeeze(ref.Data), 6), t, 'previous', 'extrap');
r = r(:, 1:2);                                   % [x y] (columns 3-6 are feedforward derivatives)
cmd = el('servo_cmd'); u = cols(squeeze(cmd.Data), 2);
ang = el('plate_angle'); a = cols(squeeze(ang.Data), 2);
trq = el('servo_torque'); q = cols(squeeze(trq.Data), 2);
xh  = el('x_hat');     xe = cols(squeeze(xh.Data), 4);

%% Metrics
e = r - p(:, 1:2);
reg = t > 3 & t < 4;
stp = t >= 4 & t < 7;
hold_ = t >= 5.5 & t < 7;
cir = t >= 10;
fprintf('Regulation (3-4 s)      : |e| max %.2f mm\n', 1000*max(vecnorm(e(reg,:), 2, 2)));
fprintf('Move to (50,30) (4-5 s) : |e| max %.2f mm during the move\n', 1000*max(vecnorm(e(t >= 4 & t < 5.5,:), 2, 2)));
fprintf('Hold at (50,30) (5.5-7) : |e| RMS %.2f mm, max %.2f mm\n', ...
    1000*rms(vecnorm(e(hold_,:), 2, 2)), 1000*max(vecnorm(e(hold_,:), 2, 2)));
fprintf('Circle tracking (10-16s): RMS error %.2f mm, max %.2f mm\n', ...
    1000*rms(vecnorm(e(cir,:), 2, 2)), 1000*max(vecnorm(e(cir,:), 2, 2)));
fprintf('Peak servo command      : %.1f deg  (limit %.0f)\n', rad2deg(max(abs(u(:)))), rad2deg(bb.ctrl.u_sat));
fprintf('Peak plate tilt         : %.2f deg\n', rad2deg(max(abs(a(:)))));
fprintf('Peak servo torque       : %.3f N*m\n', max(abs(q(:))));
fprintf('Ball stayed on plate    : %d (min height above surface %.4f m)\n', min(p(:,3)) > 0.9*bb.ball.r, min(p(:,3)));

%% Linear prediction of the move (same loop as design_controller.m, with full feedforward)
Ts = bb.ctrl.Ts;
Tlin = bb_linear_loop(bb.ctrl);
Tlin = Tlin(1, :);                               % {r, rd, rdd} -> y
tl = (4:Ts:7)';
R6 = interp1(bb.ref.r(:,1), bb.ref.r(:,2:7), tl);
p4 = interp1(t, p(:,1:2), 4);
lin = zeros(numel(tl), 2);
for k = 1:2                                      % same linear model for both axes
    lin(:,k) = p4(k) + lsim(Tlin, [R6(:,k) - p4(k), R6(:,k+2), R6(:,k+4)], tl - 4);
end

%% Plots
f = figure('Color', 'w', 'Position', [80 80 1250 820], 'Visible', 'off');
tlo = tiledlayout(f, 2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tlo, 'Closed loop on the Simscape Multibody plant (LQI + Kalman, 100 Hz)');
nexttile([2 1]); hold on; axis equal; grid on; box on
hw = 1000*bb.surf.half; rectangle('Position', [-hw -hw 2*hw 2*hw], 'EdgeColor', [0.4 0.4 0.4], 'LineWidth', 2);
plot(1000*r(:,1), 1000*r(:,2), 'k--', 'LineWidth', 1.2);
plot(1000*p(:,1), 1000*p(:,2), 'LineWidth', 1.6, 'Color', [0 0.45 0.74]);
plot(1000*p(1,1), 1000*p(1,2), 'o', 'MarkerFaceColor', [0.85 0.33 0.1], 'MarkerEdgeColor', 'none');
xlabel('x (mm)'); ylabel('y (mm)'); title('Ball path on the plate'); legend('reference', 'ball', 'start', 'Location', 'southoutside', 'Orientation', 'horizontal');
xlim([-hw hw]*1.05); ylim([-hw hw]*1.05);
nexttile; plot(t, 1000*p(:,1:2), 'LineWidth', 1.4); hold on
set(gca, 'ColorOrderIndex', 1); plot(t, 1000*r, '--', 'LineWidth', 1); grid on
xlabel('t (s)'); ylabel('position (mm)'); title('Ball position vs reference'); legend('x', 'y', 'x_{ref}', 'y_{ref}', 'Location', 'best');
nexttile; plot(t(stp) - 4, 1000*p(stp,1:2), 'LineWidth', 1.6); hold on
set(gca, 'ColorOrderIndex', 1); plot(tl - 4, 1000*lin, ':', 'LineWidth', 2); grid on
xlabel('t - 4 s'); ylabel('position (mm)'); title('Move: Simscape (solid) vs linear model (dotted)');
legend('x sim', 'y sim', 'x linear', 'y linear', 'Location', 'southeast');
nexttile; stairs(cmd.Time, rad2deg(u), 'LineWidth', 1.2); grid on
xlabel('t (s)'); ylabel('servo command (deg)'); title('Controller output'); legend('\theta_X', '\theta_Y', 'Location', 'best');
nexttile; plot(t, 1000*e, 'LineWidth', 1.2); grid on
xlabel('t (s)'); ylabel('error (mm)'); title('Tracking error r - y'); legend('e_x', 'e_y', 'Location', 'best');
resDir = fullfile(here, 'results'); if ~exist(resDir, 'dir'), mkdir(resDir); end
exportgraphics(f, fullfile(resDir, 'closed_loop.png'), 'Resolution', 150);
fprintf('Saved %s\n', fullfile(resDir, 'closed_loop.png'));
save(fullfile(resDir, 'closed_loop_run.mat'), 't', 'p', 'r', 'u', 'a', 'q');
