function build_bb_model()
%BUILD_BB_MODEL  Generate the Simscape Multibody ball-on-plate models.
%
%   ball_balance_sim.slx  open loop:   test servo command -> Plant -> logged outputs
%   ball_balance_cl.slx   closed loop: reference -> Controller (LQI + Kalman) -> Plant
%   Plant (shared):       servo dynamics + multibody mechanism + ball contact + sensors
%
%   Kinematic structure (world Z up, origin at base top centre):
%     World -> pivot -> Rev(about X) -> Cross -> Rev(about Y) -> Plate
%     World -> shaft X -> Rev(motion input) -> Horn X -> tip -> Universal -> Rod X -> Spherical -> Plate
%     World -> shaft Y -> Rev(motion input) -> Horn Y -> tip -> Universal -> Rod Y -> Spherical -> Plate
%     World -> 6-DOF -> Ball,  Ball <-> Plate surface + 4 rim walls via Spatial Contact Force
%
%   All numeric values are expressions on the workspace struct "bb" (bb_params.m),
%   which the model loads in its InitFcn, so the model stays parametric.

here = fileparts(mfilename('fullpath'));
assignin('base', 'bb', bb_params());
load_system('sm_lib'); load_system('nesl_utility'); load_system('simulink');
buildModel(here, 'ball_balance_sim', 'bb.Tsim',  @openLoopTop);
buildModel(here, 'ball_balance_cl',  'bb.ref.T', @closedLoopTop);
end

function buildModel(here, mdl, stopTime, topFcn)
if bdIsLoaded(mdl), close_system(mdl, 0); end
slx = fullfile(here, [mdl '.slx']);
if exist(slx, 'file'), delete(slx); end
new_system(mdl);
set_param(mdl, ...
    'InitFcn',   'addpath(fileparts(which(bdroot))); bb = bb_params();', ...
    'PreLoadFcn','addpath(fileparts(which(bdroot))); bb = bb_params();', ...
    'Solver', 'daessc', 'StopTime', stopTime, 'MaxStep', '1e-3', ...
    'RelTol', '1e-3', 'SaveOutput', 'on', 'SaveFormat', 'Dataset', ...
    'ReturnWorkspaceOutputs', 'on', 'SignalLogging', 'on');
P = addPlant(mdl);
topFcn(mdl, P);
try
    Simulink.BlockDiagram.arrangeSystem(P);
    Simulink.BlockDiagram.arrangeSystem(mdl);
catch
end
save_system(mdl, slx);
fprintf('Saved %s\n', slx);
end

function P = addPlant(mdl)
% Plant subsystem. Input: servo_cmd [thetaX; thetaY] (rad).
% Outputs: ball_pos, ball_vel (plate-surface frame), plate_angle, servo_angle, servo_torque.
P = [mdl '/Plant'];
add_block('built-in/Subsystem', P, 'Position', [300 80 460 260]);

lib.world = 'sm_lib/Frames and Transforms/World Frame';
lib.mech  = 'sm_lib/Utilities/Mechanism Configuration';
lib.rt    = 'sm_lib/Frames and Transforms/Rigid Transform';
lib.rev   = 'sm_lib/Joints/Revolute Joint';
lib.uj    = 'sm_lib/Joints/Universal Joint';
lib.sph   = 'sm_lib/Joints/Spherical Joint';
lib.six   = 'sm_lib/Joints/6-DOF Joint';
lib.file  = 'sm_lib/Body Elements/File Solid';
lib.brick = 'sm_lib/Body Elements/Brick Solid';
lib.ball  = 'sm_lib/Body Elements/Spherical Solid';
lib.cf    = 'sm_lib/Forces and Torques/Spatial Contact Force';
lib.ts    = 'sm_lib/Frames and Transforms/Transform Sensor';
lib.solv  = 'nesl_utility/Solver Configuration';
lib.s2ps  = 'nesl_utility/Simulink-PS Converter';
lib.ps2s  = 'nesl_utility/PS-Simulink Converter';

% --- environment
W  = blk(P, lib.world, 'World', [40 300]);
MC = blk(P, lib.mech, 'Mechanism Configuration', [40 200]);
SC = blk(P, lib.solv, 'Solver Configuration', [40 400]);
set_param(MC, 'UniformGravity', 'Constant', 'GravityVector', '[0 0 -bb.g]');
link(P, port1(MC), R(W)); link(P, port1(SC), R(W));

% --- ground body (visual only, welded to world)
base = fsolid(P, 'Base', [160 40]);
set_param(base, 'InertiaType', 'PointMass', 'Mass', 'bb.geo.body.Base.mass');
link(P, R(base), R(W));

% --- universal joint at the pivot: Rev about world X, then about world Y
Fpiv = rt(P, 'Pivot', [160 300], 'bb.geo.pivot', '', '');
link(P, L(Fpiv), R(W));
[Fcross, revCx] = axisJoint(P, 'UJ_X', [300 300], R(Fpiv), '+Y', 90, lib.rev);
set_param(revCx, 'SensePosition', 'on');
cross = fsolid(P, 'Cross', [560 220]);
massFromGeometry(cross, 'bb.geo.body.Cross.mass');
link(P, R(cross), Fcross);
[Fplate, revPy] = axisJoint(P, 'UJ_Y', [560 300], Fcross, '+X', -90, lib.rev);
set_param(revPy, 'SensePosition', 'on');
plate = fsolid(P, 'Plate', [820 220]);
massFromGeometry(plate, 'bb.geo.body.Plate.mass');
link(P, R(plate), Fplate);

% --- plate rolling surface frame S and contact geometry
Fs = rt(P, 'Surface Frame', [820 380], 'bb.surf.offset', '', '');
link(P, L(Fs), Fplate);
surfOff = rt(P, 'Surface Brick Offset', [960 380], '[0 0 -bb.surf.thick/2]', '', '');
link(P, L(surfOff), R(Fs));
sb = contactBrick(P, 'Surface Contact', [1100 380], '[2*bb.surf.half 2*bb.surf.half bb.surf.thick]');
link(P, R(sb), R(surfOff));

wallZ = '(bb.rim.height-bb.surf.thick)/2';
wallH = '(bb.rim.height+bb.surf.thick)';
walls = { 'Wall +X', sprintf('[ (bb.surf.half+bb.rim.wall/2) 0 %s]', wallZ), sprintf('[bb.rim.wall 2*bb.surf.half+2*bb.rim.wall %s]', wallH)
          'Wall -X', sprintf('[-(bb.surf.half+bb.rim.wall/2) 0 %s]', wallZ), sprintf('[bb.rim.wall 2*bb.surf.half+2*bb.rim.wall %s]', wallH)
          'Wall +Y', sprintf('[0  (bb.surf.half+bb.rim.wall/2) %s]', wallZ), sprintf('[2*bb.surf.half bb.rim.wall %s]', wallH)
          'Wall -Y', sprintf('[0 -(bb.surf.half+bb.rim.wall/2) %s]', wallZ), sprintf('[2*bb.surf.half bb.rim.wall %s]', wallH) };
wb = zeros(1, 4);
for i = 1:4
    off = rt(P, [walls{i,1} ' Offset'], [960 460 + 80*i], walls{i,2}, '', '');
    link(P, L(off), R(Fs));
    wb(i) = contactBrick(P, walls{i,1}, [1100 460 + 80*i], walls{i,3});
    link(P, R(wb(i)), R(off));
end

% --- ball: free body with contact against surface and rim walls
six = blk(P, lib.six, 'Ball Joint', [1100 120]);
set_param(six, ...
    'PxPositionTargetSpecify', 'on', 'PxPositionTargetValue', 'bb.ball.p0(1)', 'PxPositionTargetValueUnits', 'm', ...
    'PyPositionTargetSpecify', 'on', 'PyPositionTargetValue', 'bb.ball.p0(2)', 'PyPositionTargetValueUnits', 'm', ...
    'PzPositionTargetSpecify', 'on', 'PzPositionTargetValue', 'bb.ball.p0(3)', 'PzPositionTargetValueUnits', 'm');
link(P, L(six), R(W));
ball = blk(P, lib.ball, 'Ball', [1260 120]);
set_param(ball, 'SphereRadius', 'bb.ball.r', 'SphereRadiusUnits', 'm', ...
    'InertiaType', 'CalculateFromGeometry', 'BasedOnType', 'Density', ...
    'Density', 'bb.ball.rho', 'DensityUnits', 'kg/m^3', ...
    'GraphicDiffuseColor', 'bb.geo.body.Ball.color', 'GraphicShininess', '100', ...
    'ExportEntireGeometry', 'on');
link(P, R(ball), R(six));
contacts = [sb wb];
names = {'Contact Surface', 'Contact Wall +X', 'Contact Wall -X', 'Contact Wall +Y', 'Contact Wall -Y'};
for i = 1:numel(contacts)
    cf = blk(P, lib.cf, names{i}, [1260 380 + 80*(i-1)]);
    set_param(cf, 'NormalStiffness', 'bb.contact.k', 'NormalStiffnessUnits', 'N/m', ...
        'NormalDamping', 'bb.contact.c', 'NormalDampingUnits', 'N/(m/s)', ...
        'NormalTransitionRegionWidth', 'bb.contact.w', 'NormalTransitionRegionWidthUnits', 'm', ...
        'FrictionType', 'SmoothStickSlip', ...
        'CoefficientOfStaticFriction', 'bb.contact.mu_s', ...
        'CoefficientOfDynamicFriction', 'bb.contact.mu_k', ...
        'FrictionalCriticalVelocity', 'bb.contact.v_th', 'FrictionalCriticalVelocityUnits', 'm/s');
    link(P, L(cf), L(contacts(i)));   % B: plate geometry
    link(P, R(cf), L(ball));          % F: ball geometry
end

% --- ball sensing relative to the plate surface (= touch-panel measurement)
tsP = blk(P, lib.ts, 'Ball Position Sensor', [1420 200]);
set_param(tsP, 'SenseXYZ', 'on', 'ObservationFrame', 'Base', 'ExpressedInFrame', 'Base');
tsV = blk(P, lib.ts, 'Ball Velocity Sensor', [1420 280]);
set_param(tsV, 'SenseXYZDot', 'on', 'ObservationFrame', 'Base', 'ExpressedInFrame', 'Base');
for t = [tsP tsV]
    link(P, L(t), R(Fs));
    link(P, R(t), R(ball));
end

% --- servo units
[qX, tX] = servoUnit(P, lib, 'X', [160 620], W, Fplate);
[qY, tY] = servoUnit(P, lib, 'Y', [160 900], W, Fplate);

%% Simulink-side signals inside the Plant
in  = blk(P, 'simulink/Sources/In1', 'servo_cmd', [-420 760]);
set_param(in, 'PortDimensions', '2');
sat = blk(P, 'simulink/Discontinuities/Saturation', 'Servo Travel', [-320 760]);
set_param(sat, 'UpperLimit', 'bb.servo.max', 'LowerLimit', '-bb.servo.max');
rl  = blk(P, 'simulink/Discontinuities/Rate Limiter', 'Servo Slew', [-220 760]);
set_param(rl, 'RisingSlewLimit', 'bb.servo.rate', 'FallingSlewLimit', '-bb.servo.rate');
ss  = blk(P, 'simulink/Continuous/State-Space', 'Servo Lag', [-120 760]);
set_param(ss, 'A', 'bb.servo.A', 'B', 'bb.servo.B', 'C', 'bb.servo.C', 'D', 'bb.servo.D');
dmx = blk(P, 'simulink/Signal Routing/Demux', 'Demux', [-20 760]);
set_param(dmx, 'Outputs', '2');
sig(P, in, 1, sat, 1); sig(P, sat, 1, rl, 1); sig(P, rl, 1, ss, 1); sig(P, ss, 1, dmx, 1);
sig(P, dmx, 1, qX, 1); sig(P, dmx, 2, qY, 1);

outs = {'ball_pos', 'ball_vel', 'plate_angle', 'servo_angle', 'servo_torque'};
op = zeros(1, 5);
for i = 1:5
    op(i) = blk(P, 'simulink/Sinks/Out1', outs{i}, [1700 120 + 90*i]);
end
pPos = ps2s(P, lib, 'PS ball_pos', [1560 200], 'm');   link(P, L(pPos), R(tsP, 1)); sig(P, pPos, 1, op(1), 1);
pVel = ps2s(P, lib, 'PS ball_vel', [1560 280], 'm/s'); link(P, L(pVel), R(tsV, 1)); sig(P, pVel, 1, op(2), 1);
pAx  = ps2s(P, lib, 'PS plate X', [1560 360], 'rad');  link(P, L(pAx), R(revCx, 1));
pAy  = ps2s(P, lib, 'PS plate Y', [1560 420], 'rad');  link(P, L(pAy), R(revPy, 1));
mxA  = blk(P, 'simulink/Signal Routing/Mux', 'Mux plate', [1640 380]); set_param(mxA, 'Inputs', '2');
sig(P, pAx, 1, mxA, 1); sig(P, pAy, 1, mxA, 2); sig(P, mxA, 1, op(3), 1);
sig(P, ss, 1, op(4), 1);
pTx  = ps2s(P, lib, 'PS torque X', [1560 520], 'N*m'); link(P, L(pTx), tX);
pTy  = ps2s(P, lib, 'PS torque Y', [1560 580], 'N*m'); link(P, L(pTy), tY);
mxT  = blk(P, 'simulink/Signal Routing/Mux', 'Mux torque', [1640 550]); set_param(mxT, 'Inputs', '2');
sig(P, pTx, 1, mxT, 1); sig(P, pTy, 1, mxT, 2); sig(P, mxT, 1, op(5), 1);
end

%% ===================================================================== top levels
function logPlantOutputs(mdl, P)
outs = {'ball_pos', 'ball_vel', 'plate_angle', 'servo_angle', 'servo_torque'};
ph = get_param(P, 'PortHandles');
for i = 1:5
    o = blk(mdl, 'simulink/Sinks/Out1', outs{i}, [820 60 + 60*i]);
    sig(mdl, P, i, o, 1);
    logSig(ph.Outport(i), outs{i});
end
end

function openLoopTop(mdl, P)
cmd = blk(mdl, 'simulink/Sources/From Workspace', 'Servo Command (test)', [60 150]);
set_param(cmd, 'VariableName', 'bb.test.u', 'Interpolate', 'off', ...
    'OutputAfterFinalValue', 'Holding final value');
sig(mdl, cmd, 1, P, 1);
logPlantOutputs(mdl, P);
sc = blk(mdl, 'simulink/Sinks/Scope', 'Ball Position', [820 420]);
sig(mdl, P, 1, sc, 1);
end

function closedLoopTop(mdl, P)
% Discrete controller at bb.ctrl.Ts: touch-panel sampling (+ noise), Kalman estimator
% with delayed command input, integral of error with clamping, LQI gain, saturation.
ref = blk(mdl, 'simulink/Sources/From Workspace', 'Reference', [60 150]);
set_param(ref, 'VariableName', 'bb.ref.r', 'Interpolate', 'on', ...
    'OutputAfterFinalValue', 'Holding final value');

C = [mdl '/Controller'];
add_block('built-in/Subsystem', C, 'Position', [260 130 420 230]);
rIn = blk(C, 'simulink/Sources/In1', 'ref', [20 40]);
yIn = blk(C, 'simulink/Sources/In1', 'ball_pos', [20 160]);
uOut = blk(C, 'simulink/Sinks/Out1', 'servo_cmd', [1000 100]);
xOut = blk(C, 'simulink/Sinks/Out1', 'x_hat', [1000 220]);
yOut = blk(C, 'simulink/Sinks/Out1', 'y_meas', [1000 300]);

% ref = [x y vx vy ax ay]
rS  = blk(C, 'simulink/Discrete/Zero-Order Hold', 'Sample ref', [120 40]);
set_param(rS, 'SampleTime', 'bb.ctrl.Ts');
rP  = blk(C, 'simulink/Signal Routing/Selector', 'ref pos', [220 40]);
set_param(rP, 'InputPortWidth', '6', 'IndexParamArray', {'[1 2]'});
rX  = blk(C, 'simulink/Signal Routing/Selector', 'ref state', [220 -40]);
set_param(rX, 'InputPortWidth', '6', 'IndexParamArray', {'[1 3 2 4]'});   % [x vx y vy]
rA  = blk(C, 'simulink/Signal Routing/Selector', 'ref accel', [220 -120]);
set_param(rA, 'InputPortWidth', '6', 'IndexParamArray', {'[5 6]'});
ff  = blk(C, 'simulink/Math Operations/Gain', 'Accel Feedforward', [700 -120]);
set_param(ff, 'Gain', 'bb.ctrl.ff');
dX  = blk(C, 'simulink/Math Operations/Sum', 'State Error', [760 40]);
set_param(dX, 'Inputs', '+-');
uS  = blk(C, 'simulink/Math Operations/Sum', 'Add Feedforward', [900 60]);
set_param(uS, 'Inputs', '++');
sel = blk(C, 'simulink/Signal Routing/Selector', 'Select xy', [120 160]);
set_param(sel, 'InputPortWidth', '3', 'IndexParamArray', {'[1 2]'});
yS  = blk(C, 'simulink/Discrete/Zero-Order Hold', 'Touch Panel Sample', [220 160]);
set_param(yS, 'SampleTime', 'bb.ctrl.Ts');
nz  = blk(C, 'simulink/Sources/Random Number', 'Touch Panel Noise', [220 230]);
set_param(nz, 'Mean', '[0 0]', 'Variance', '(bb.ctrl.noise_on*bb.ctrl.noise_sd)^2*[1 1]', ...
    'Seed', '[11 23]', 'SampleTime', 'bb.ctrl.Ts');
yN  = blk(C, 'simulink/Math Operations/Sum', 'Add Noise', [320 170]);
set_param(yN, 'Inputs', '++');
err = blk(C, 'simulink/Math Operations/Sum', 'Error', [420 60]);
set_param(err, 'Inputs', '+-');
xi  = blk(C, 'simulink/Discrete/Discrete-Time Integrator', 'Integral of Error', [520 60]);
set_param(xi, 'IntegratorMethod', 'Integration: Forward Euler', 'gainval', '1', ...
    'SampleTime', 'bb.ctrl.Ts', 'LimitOutput', 'on', ...
    'UpperSaturationLimit', 'bb.ctrl.xi_max', 'LowerSaturationLimit', '-bb.ctrl.xi_max');
ud  = blk(C, 'simulink/Discrete/Unit Delay', 'u[n-1]', [520 260]);
set_param(ud, 'SampleTime', 'bb.ctrl.Ts', 'InitialCondition', '[0;0]');
mxE = blk(C, 'simulink/Signal Routing/Mux', 'Mux est', [620 200]);
set_param(mxE, 'Inputs', '2');
est = blk(C, 'simulink/Discrete/Discrete State-Space', 'Kalman Estimator', [700 200]);
set_param(est, 'A', 'bb.ctrl.est2.A', 'B', 'bb.ctrl.est2.B', 'C', 'bb.ctrl.est2.C', ...
    'D', 'bb.ctrl.est2.D', 'InitialCondition', 'bb.ctrl.est2.x0', 'SampleTime', 'bb.ctrl.Ts');
mxS = blk(C, 'simulink/Signal Routing/Mux', 'Mux state', [800 100]);
set_param(mxS, 'Inputs', '2');
kg  = blk(C, 'simulink/Math Operations/Gain', 'LQI Gain', [860 100]);
set_param(kg, 'Gain', '-bb.ctrl.K2', 'Multiplication', 'Matrix(K*u)');
sat = blk(C, 'simulink/Discontinuities/Saturation', 'Command Limit', [940 100]);
set_param(sat, 'UpperLimit', 'bb.ctrl.u_sat', 'LowerLimit', '-bb.ctrl.u_sat');

sig(C, rIn, 1, rS, 1);  sig(C, yIn, 1, sel, 1); sig(C, sel, 1, yS, 1);
sig(C, rS, 1, rP, 1);   sig(C, rS, 1, rX, 1);  sig(C, rS, 1, rA, 1);
sig(C, yS, 1, yN, 1);   sig(C, nz, 1, yN, 2);
sig(C, rP, 1, err, 1);  sig(C, yN, 1, err, 2); sig(C, err, 1, xi, 1);
sig(C, ud, 1, mxE, 1);  sig(C, yN, 1, mxE, 2); sig(C, mxE, 1, est, 1);
sig(C, est, 1, dX, 1);  sig(C, rX, 1, dX, 2);
sig(C, dX, 1, mxS, 1);  sig(C, xi, 1, mxS, 2); sig(C, mxS, 1, kg, 1);
sig(C, rA, 1, ff, 1);   sig(C, kg, 1, uS, 1);  sig(C, ff, 1, uS, 2);
sig(C, uS, 1, sat, 1);  sig(C, sat, 1, uOut, 1); sig(C, sat, 1, ud, 1);
sig(C, est, 1, xOut, 1); sig(C, yN, 1, yOut, 1);
try, Simulink.BlockDiagram.arrangeSystem(C); catch, end

sig(mdl, ref, 1, C, 1);
sig(mdl, C, 1, P, 1);
pp = get_param(P, 'PortHandles');
add_line(mdl, pp.Outport(1), get_param(C, 'PortHandles').Inport(2), 'autorouting', 'on');
logPlantOutputs(mdl, P);
cp = get_param(C, 'PortHandles');
names = {'servo_cmd', 'x_hat', 'y_meas'};
for i = 1:3
    o = blk(mdl, 'simulink/Sinks/Out1', names{i}, [820 400 + 60*i]);
    sig(mdl, C, i, o, 1);
    logSig(cp.Outport(i), names{i});
end
logSig(get_param(ref, 'PortHandles').Outport(1), 'ref');
sc = blk(mdl, 'simulink/Sinks/XY Graph', 'Ball XY', [820 650]);
dm = blk(mdl, 'simulink/Signal Routing/Demux', 'Demux xy', [700 650]);
set_param(dm, 'Outputs', '3');
add_line(mdl, pp.Outport(1), get_param(dm, 'PortHandles').Inport(1), 'autorouting', 'on');
sig(mdl, dm, 1, sc, 1); sig(mdl, dm, 2, sc, 2);
term = blk(mdl, 'simulink/Sinks/Terminator', 'Unused z', [820 720]);
sig(mdl, dm, 3, term, 1);
try, set_param(sc, 'xmin', '-0.12', 'xmax', '0.12', 'ymin', '-0.12', 'ymax', '0.12'); catch, end
end

function logSig(port, name)
set_param(port, 'DataLogging', 'on', 'DataLoggingNameMode', 'Custom', 'DataLoggingName', name);
end

%% ===================================================================== helpers
function h = blk(sys, src, name, xy)
h = add_block(src, [sys '/' name], 'MakeNameUnique', 'on');
pos = get_param(h, 'Position');
w = max(pos(3) - pos(1), 30); hgt = max(pos(4) - pos(2), 30);
set_param(h, 'Position', [xy(1) xy(2) xy(1) + w xy(2) + hgt]);
end

function p = L(b, i)
if nargin < 2, i = 1; end
ph = get_param(b, 'PortHandles'); p = ph.LConn(i);
end

function p = R(b, i)
% R(b) is the block's first right-side frame port; R(b, k) is its k-th sensor output.
if nargin < 2, i = 0; end
ph = get_param(b, 'PortHandles'); p = ph.RConn(i + 1);
end

function p = port1(b)
ph = get_param(b, 'PortHandles'); p = [ph.LConn ph.RConn]; p = p(1);
end

function link(sys, a, b)
add_line(sys, a, b, 'autorouting', 'on');
end

function sig(sys, a, ai, b, bi)
pa = get_param(a, 'PortHandles'); pb = get_param(b, 'PortHandles');
add_line(sys, pa.Outport(ai), pb.Inport(bi), 'autorouting', 'on');
end

function h = rt(sys, name, xy, trans, axis, deg)
h = blk(sys, 'sm_lib/Frames and Transforms/Rigid Transform', name, xy);
if ~isempty(trans)
    set_param(h, 'TranslationMethod', 'Cartesian', 'TranslationCartesianOffset', trans, ...
        'TranslationCartesianOffsetUnits', 'm');
end
if ~isempty(axis)
    set_param(h, 'RotationMethod', 'StandardAxis', 'RotationStandardAxis', axis, ...
        'RotationAngle', num2str(deg), 'RotationAngleUnits', 'deg');
end
end

function [Fout, j] = axisJoint(sys, name, xy, Fin, rotAxis, deg, jointLib)
% Revolute joint whose rotation axis is the world-aligned input frame's axis
% obtained by rotating Z onto it; the output frame is restored to world alignment.
a = rt(sys, [name ' In'], xy, '', rotAxis, deg);
j = blk(sys, jointLib, name, xy + [100 0]);
b = rt(sys, [name ' Out'], xy + [200 0], '', rotAxis, -deg);
link(sys, L(a), Fin); link(sys, R(a), L(j)); link(sys, R(j), L(b));
Fout = R(b);
end

function h = fsolid(sys, bodyName, xy)
% File Solid takes a literal file name (not an expression); bb_params puts cad/stl on the path.
h = blk(sys, 'sm_lib/Body Elements/File Solid', [bodyName ' Solid'], xy);
[~, stlName, ext] = fileparts(evalin('base', sprintf('bb.geo.body.%s.stl', bodyName)));
set_param(h, 'ExtGeomFileName', [stlName ext], ...
    'UnitType', 'Custom', 'ExtGeomFileUnits', 'm', ...
    'GraphicType', 'FromGeometry', 'GraphicVisPropType', 'SimpleVisualProperties', ...
    'GraphicDiffuseColor', sprintf('bb.geo.body.%s.color', bodyName));
end

function massFromGeometry(h, massExpr)
set_param(h, 'InertiaType', 'CalculateFromGeometry', 'BasedOnType', 'Mass', ...
    'Mass', massExpr, 'MassUnits', 'kg');
end

function h = contactBrick(sys, name, xy, dims)
% Invisible, massless-in-practice brick that only supplies contact geometry.
h = blk(sys, 'sm_lib/Body Elements/Brick Solid', name, xy);
set_param(h, 'BrickDimensions', dims, 'BrickDimensionsUnits', 'm', ...
    'InertiaType', 'PointMass', 'Mass', '1e-6', 'MassUnits', 'kg', ...
    'GraphicType', 'None', 'ExportEntireGeometry', 'on');
end

function h = ps2s(sys, lib, name, xy, unit)
h = blk(sys, lib.ps2s, name, xy);
set_param(h, 'Unit', unit);
end

function [qIn, tOut] = servoUnit(P, lib, tag, xy, W, Fplate)
% Servo shaft frame: Z along the shaft axis (X unit: world -Y, Y unit: world +X).
if tag == 'X'
    shaft = 'bb.geo.servoX.shaft'; tip = '[bb.geo.horn.length 0 0]'; att = 'bb.attachX';
    rotAx = '+X'; deg = 90;     % Z -> -Y
else
    shaft = 'bb.geo.servoY.shaft'; tip = '[0 bb.geo.horn.length 0]'; att = 'bb.attachY';
    rotAx = '+Y'; deg = 90;     % Z -> +X
end
Fsh = rt(P, ['Shaft ' tag], xy, shaft, '', '');
link(P, L(Fsh), R(W));
[Fh, rev] = axisJoint(P, ['Servo ' tag], xy + [140 0], R(Fsh), rotAx, deg, lib.rev);
set_param(rev, 'MotionActuationMode', 'InputMotion', 'TorqueActuationMode', 'ComputedTorque', ...
    'SenseTorqueForce', 'on');
horn = fsolid(P, ['Horn_' tag], xy + [460 -80]);
massFromGeometry(horn, sprintf('bb.geo.body.Horn_%s.mass', tag));
link(P, R(horn), Fh);
Ft = rt(P, ['Horn Tip ' tag], xy + [480 0], tip, '', '');
link(P, L(Ft), Fh);
uj = blk(P, lib.uj, ['Rod End Lower ' tag], xy + [600 0]);
link(P, L(uj), R(Ft));
rod = fsolid(P, ['Rod_' tag], xy + [740 -80]);
massFromGeometry(rod, sprintf('bb.geo.body.Rod_%s.mass', tag));
link(P, R(rod), R(uj));
Frt = rt(P, ['Rod Top ' tag], xy + [740 0], '[0 0 bb.geo.rod.length]', '', '');
link(P, L(Frt), R(uj));
Fa = rt(P, ['Plate Clevis ' tag], xy + [740 120], att, '', '');
link(P, L(Fa), Fplate);
sj = blk(P, lib.sph, ['Rod End Upper ' tag], xy + [880 60]);
link(P, L(sj), R(Fa)); link(P, R(sj), R(Frt));

qIn = blk(P, lib.s2ps, ['S-PS servo ' tag], xy + [20 120]);
set_param(qIn, 'Unit', 'rad', 'FilteringAndDerivatives', 'filter', ...
    'SimscapeFilterOrder', '2', 'InputFilterTimeConstant', 'bb.servo.ps_tc');
link(P, R(qIn), L(rev, 2));
tOut = R(rev, 1);
end
