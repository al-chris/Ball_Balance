%RUN_OPEN_LOOP_TEST  Simulate ball_balance_sim with the open-loop servo test
% profile (bb.test.u), plot the response and validate the rolling physics.
%
% Check: with the plate held at a constant tilt q, a solid sphere rolling
% without slip accelerates at a = 5/7 * g * sin(q) along the plate.

here = fileparts(mfilename('fullpath'));
addpath(here);
bb  = bb_params();
mdl = 'ball_balance_sim';
load_system(fullfile(here, [mdl '.slx']));

tic; out = sim(mdl); tsim = toc;
fprintf('Simulated %.1f s of motion in %.1f s wall time (first run includes model compile)\n', bb.Tsim, tsim);

get = @(name) out.logsout.getElement(name).Values;
pos = get('ball_pos');  vel = get('ball_vel');
ang = get('plate_angle'); srv = get('servo_angle'); trq = get('servo_torque');
t = pos.Time; p = squeeze(pos.Data); if size(p,1) == 3, p = p.'; end
a = squeeze(ang.Data); if size(a,1) == 2 && size(a,2) ~= 2, a = a.'; end
s = squeeze(srv.Data); if size(s,1) == 2 && size(s,2) ~= 2, s = s.'; end
q = squeeze(trq.Data); if size(q,1) == 2 && size(q,2) ~= 2, q = q.'; end

%% Rolling validation: constant X tilt window (servo X held at +0.3 rad)
win = t > 0.50 & t < 0.80;
qy  = mean(interp1(ang.Time, a(:,2), t(win)));
cx  = polyfit(t(win), p(win,1), 2);
a_sim    = 2*cx(1);
a_theory = 5/7 * bb.g * sin(qy);
fprintf('Plate tilt about Y in window : %+.3f deg\n', rad2deg(qy));
fprintf('Ball accel along x (sim)     : %+.4f m/s^2\n', a_sim);
fprintf('Rolling theory 5/7 g sin(q)  : %+.4f m/s^2  (error %.1f %%)\n', ...
        a_theory, 100*abs(a_sim - a_theory)/abs(a_theory));
fprintf('Sliding (no-roll) would be   : %+.4f m/s^2\n', bb.g*sin(qy));
fprintf('Ball height above surface    : %.5f m (radius %.3f) -> penetration %.1f um\n', ...
        median(p(t < 0.3,3)), bb.ball.r, 1e6*(bb.ball.r - median(p(t < 0.3,3))));
fprintf('Peak servo torque            : X %.3f N*m, Y %.3f N*m\n', max(abs(q(:,1))), max(abs(q(:,2))));
fprintf('Plate tilt per servo angle   : %.3f deg/deg (small-angle, X axis)\n', ...
        abs(qy) / 0.3);

%% Plots
f = figure('Color', 'w', 'Position', [100 100 1100 750], 'Visible', 'off');
tiledlayout(f, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
nexttile; plot(srv.Time, rad2deg(s), 'LineWidth', 1.4); grid on
ylabel('servo angle (deg)'); title('Servo angles (actual)'); legend('\theta_X', '\theta_Y', 'Location', 'best');
nexttile; plot(ang.Time, rad2deg(a), 'LineWidth', 1.4); grid on
ylabel('plate angle (deg)'); title('Plate angles (U-joint)'); legend('about X', 'about Y', 'Location', 'best');
nexttile; plot(t, 1000*p(:,1:2), 'LineWidth', 1.4); grid on; hold on
yline(1000*(bb.surf.half - bb.ball.r)*[1 -1], 'k--', 'rim');
xlabel('t (s)'); ylabel('ball position (mm)'); title('Ball position on plate'); legend('x', 'y', 'Location', 'best');
nexttile; plot(trq.Time, q, 'LineWidth', 1.4); grid on
xlabel('t (s)'); ylabel('torque (N\cdotm)'); title('Servo torque required'); legend('X', 'Y', 'Location', 'best');
resDir = fullfile(here, 'results'); if ~exist(resDir, 'dir'), mkdir(resDir); end
exportgraphics(f, fullfile(resDir, 'open_loop_test.png'), 'Resolution', 150);
fprintf('Saved %s\n', fullfile(resDir, 'open_loop_test.png'));
