function bb = bb_params()
%BB_PARAMS  Parameters for the ball-on-plate Simscape Multibody model.
%   bb = BB_PARAMS() loads the CAD geometry exported from Blender
%   (cad/bb_geometry.m) and adds contact, servo, solver and test settings.
%   Units: SI (m, kg, s, rad).

root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root, 'cad'), fullfile(root, 'cad', 'stl'));   % File Solid finds STLs on the path
geo = bb_geometry();
bb.geo = geo;
bb.g = 9.80665;

%% Plate rolling surface (frame S, rigidly attached to the plate)
% S sits at the centre of the glass top, axes parallel to the plate frame.
bb.surf.offset = [0 0 geo.plate.surfaceZ - geo.pivot(3)];
bb.surf.half   = geo.plate.surfaceWidth / 2;      % half width inside the rim
bb.surf.thick  = 0.004;                           % contact brick thickness
bb.rim.height  = geo.plate.rimHeight;             % rim height above surface
bb.rim.wall    = 0.010;                           % contact wall thickness

%% Plate-side joint frames (relative to the plate frame at the pivot)
bb.attachX = geo.rodX.top - geo.pivot;
bb.attachY = geo.rodY.top - geo.pivot;

%% Ball
bb.ball.r   = geo.ball.radius;
bb.ball.rho = 7850;                               % chrome steel
bb.ball.m   = 4/3*pi*bb.ball.r^3*bb.ball.rho;
bb.ball.p0  = geo.ball.center0;                   % initial centre (world)

%% Contact (smooth spring-damper normal force + stick-slip friction)
bb.contact.k    = 1e5;      % N/m
bb.contact.c    = 150;      % N/(m/s)
bb.contact.w    = 1e-4;     % transition region width, m
bb.contact.mu_s = 0.40;
bb.contact.mu_k = 0.30;
bb.contact.v_th = 1e-3;     % critical (stick/slip) velocity, m/s

%% Servo model: command -> saturation -> rate limit -> 2nd-order lag
% Typical metal-gear hobby servo (MG996R class): ~0.17 s / 60 deg.
bb.servo.max   = deg2rad(60);          % mechanical travel used (+/-)
bb.servo.rate  = deg2rad(60) / 0.17;   % slew-rate limit, rad/s
bb.servo.wn    = 2*pi*8;               % closed-loop bandwidth of the servo
bb.servo.zeta  = 0.8;
A1 = [0 1; -bb.servo.wn^2 -2*bb.servo.zeta*bb.servo.wn];
B1 = [0; bb.servo.wn^2];
C1 = [1 0];
bb.servo.A = blkdiag(A1, A1);          % two independent channels [X; Y]
bb.servo.B = blkdiag(B1, B1);
bb.servo.C = blkdiag(C1, C1);
bb.servo.D = zeros(2);
bb.servo.ps_tc = 0.002;                % Simulink-PS input filter time constant

%% Simulation / open-loop test input  [t  thetaX_cmd  thetaY_cmd] (rad)
bb.Tsim = 3.0;
bb.test.u = [ 0.00   0.0   0.0
              0.30   0.0   0.0
              0.30   0.3   0.0
              0.90   0.3   0.0
              0.90  -0.3   0.0
              1.50  -0.3   0.0
              1.50   0.0   0.2
              2.20   0.0   0.2
              2.20   0.0   0.0
              3.00   0.0   0.0 ];

%% Closed loop: controller and reference trajectory
% bb.ref.r = [t  x y  vx vy  ax ay]: smooth position plus derivatives for feedforward.
%   0-4 s  regulate from the initial offset to the centre
%   4-5 s  minimum-jerk move to (50, 30) mm, hold
%   7-8 s  minimum-jerk move back to the centre
%   8-16 s circle, 4 s period, radius ramped 0 -> 40 mm over 8-10 s
bb.ctrl = bb_controller(bb);
bb.ref.T = 16;
dt = 0.005;
t = (0:dt:bb.ref.T)';
s = @(t0, T) min(max((t - t0)/T, 0), 1);        % normalised time
mj = @(u) 10*u.^3 - 15*u.^4 + 6*u.^5;           % minimum-jerk profile 0 -> 1
pos = mj(s(4, 1)) * [0.05 0.03] - mj(s(7, 1)) * [0.05 0.03];
w = 2*pi/4; R = 0.04 * mj(s(8, 2));
on = t >= 8;
pos(on, :) = R(on) .* [cos(w*(t(on) - 8)) sin(w*(t(on) - 8))];
vel = gradient(pos', dt)';
acc = gradient(vel', dt)';
bb.ref.r = [t pos vel acc];
end
