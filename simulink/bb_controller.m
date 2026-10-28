function ctrl = bb_controller(bb, lqrOverride)
%BB_CONTROLLER  Linear ball model, discrete LQI state feedback and Kalman estimator.
%
%   Ball on a plate rotating by beta about a pivot a distance d below the ball
%   centre, rolling without slip (J = 2/5 m R^2), in plate coordinates:
%       (7/5) x_dd = -g sin(beta) + x beta_dot^2 + (d + 2R/5) beta_dd
%   Plate tilt from the push-rod lever: beta = (r_horn / d_attach) * theta.
%   Linearised, per axis, from servo angle theta to ball position x:
%       G(s) = kin * 5/7 * (-g + d_eff s^2) / s^2,   d_eff = d + 2R/5
%   The zeros at +/- sqrt(g/d_eff) (~13.5 rad/s, one non-minimum-phase) raise the
%   loop gain near crossover, so they are kept in the analysis model (Gfull, with
%   servo dynamics). The controller is designed on the double integrator
%   x_dd = -c theta,  c = 5/7 g kin,  and tuned/verified on Gfull.
%   Sign: +theta lifts the +x (+y) edge, so the ball accelerates towards -x (-y).
%   Both axes have the same model, so one design is used for X and Y.
%   Neglected (checked in simulation): centripetal term, cross-coupling, rate limit.
%
%   Optional lqrOverride: struct with any of x_max, v_max, xi_max, u_max, w_sd.
%
%   Controller (sample time Ts, touch-panel rate):
%       u[n]   = -K * [x_hat - r; v_hat - r_dot; xi] - r_ddot / c
%       xi[n+1] = xi[n] + Ts*(r[n] - y[n])                  (integral of error)
%   K comes from lqi; the reference enters as a state target plus acceleration
%   feedforward, so the integrator only removes residual bias (and can be clamped).
%   Estimator (steady-state Kalman, "current" form, position measured only):
%       x_pred[n] = A x_hat[n-1] + B u[n-1]
%       x_hat[n]  = x_pred[n] + M (y[n] - C x_pred[n])

ctrl.Ts = 0.01;                          % 100 Hz touch panel / control loop
Ts = ctrl.Ts;
d = norm(bb.attachX(1:2));               % plate attachment radius
ctrl.kin = bb.geo.horn.length / d;       % plate tilt per servo angle (small angle)
ctrl.c = 5/7 * bb.g * ctrl.kin;          % ball accel per servo angle, m/s^2/rad

A = [0 1; 0 0]; B = [0; -ctrl.c]; C = [1 0];
ctrl.sysc = ss(A, B, C, 0, 'StateName', {'x', 'v'}, 'InputName', 'theta', 'OutputName', 'x');
ctrl.sysd = c2d(ctrl.sysc, Ts, 'zoh');

% full linear analysis model: servo command -> ball position
ctrl.d_eff = bb.surf.offset(3) + bb.ball.r + 0.4*bb.ball.r;
servo = tf(bb.servo.wn^2, [1 2*bb.servo.zeta*bb.servo.wn bb.servo.wn^2]);
ctrl.Gfull = ctrl.kin * 5/7 * tf([ctrl.d_eff 0 -bb.g], [1 0 0]) * servo;

%% LQI design: Bryson's rule, max acceptable deviation of each state / input
% Tuned by scanning on Gfull for PM >= 50 deg, GM >= 10 dB, fastest settling
% (gives PM ~57 deg, GM +23 dB, delay margin ~110 ms).
ctrl.lqr.x_max  = 0.04;     % m
ctrl.lqr.v_max  = 0.04;     % m/s
ctrl.lqr.xi_max = 0.008;    % m*s
ctrl.lqr.u_max  = 0.25;     % rad servo angle
ctrl.lqr.w_sd   = 0.02;     % Kalman: input-equivalent disturbance, rad
if nargin > 1
    for f = fieldnames(lqrOverride)', ctrl.lqr.(f{1}) = lqrOverride.(f{1}); end
end
Q = diag(1 ./ [ctrl.lqr.x_max ctrl.lqr.v_max ctrl.lqr.xi_max].^2);
R = 1 / ctrl.lqr.u_max^2;
ctrl.K = lqi(ctrl.sysd, Q, R);           % 1x3: [k_x k_v k_xi]

%% Kalman estimator (acceleration disturbance enters like the input)
ctrl.noise_sd = 1e-3;                    % touch-panel position noise, m (1 sigma)
sysn = ss(ctrl.sysd.A, [ctrl.sysd.B ctrl.sysd.B], ctrl.sysd.C, 0, Ts);
[~, ~, ~, ctrl.M] = kalman(sysn, ctrl.lqr.w_sd^2, ctrl.noise_sd^2);   % innovation gain

% Estimator as one discrete state-space block, inputs v = [u[n-1]; y[n]],
% state s[n] = x_hat[n-1], output x_hat[n]:  s+ = Ae s + Be v,  x_hat = Ae s + Be v
I2 = eye(2);
Ad = ctrl.sysd.A; Bd = ctrl.sysd.B;
ctrl.est.A = (I2 - ctrl.M*C) * Ad;
ctrl.est.B = [(I2 - ctrl.M*C) * Bd, ctrl.M];

%% Two-axis matrices for the Simulink implementation
% estimator input  [u_x; u_y; y_x; y_y],  output [x_x; v_x; x_y; v_y]
Ae = ctrl.est.A; Bu = ctrl.est.B(:,1); By = ctrl.est.B(:,2); z2 = zeros(2,1);
ctrl.est2.A = blkdiag(Ae, Ae);
ctrl.est2.B = [Bu z2 By z2; z2 Bu z2 By];
ctrl.est2.C = ctrl.est2.A;
ctrl.est2.D = ctrl.est2.B;
ctrl.est2.x0 = [bb.ball.p0(1) 0 bb.ball.p0(2) 0]';    % start at the known ball position
% u = -K2 * [x_x-r_x  v_x-rd_x  x_y-r_y  v_y-rd_y  xi_x  xi_y] + ff * [rdd_x; rdd_y]
k = ctrl.K;
ctrl.K2 = [k(1) k(2) 0 0 k(3) 0
           0 0 k(1) k(2) 0 k(3)];
ctrl.ff = -1 / ctrl.c;                   % acceleration feedforward, rad per m/s^2

%% Limits / test options
ctrl.u_sat    = deg2rad(35);             % command limit (plate ~8.5 deg)
ctrl.xi_max   = deg2rad(10) / abs(k(3)); % anti-windup: integral share <= 10 deg
ctrl.noise_on = 1;                       % 1: add touch-panel noise in simulation
end
