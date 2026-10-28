function [T, Lu] = bb_linear_loop(ctrl)
%BB_LINEAR_LOOP  Discrete linear closed loop of one axis for analysis.
%   [T, Lu] = BB_LINEAR_LOOP(ctrl) uses the full plant ctrl.Gfull (servo lag,
%   pivot-offset zeros) with the implemented controller: delayed-command Kalman
%   estimator, integral of error, LQI gains and reference feedforward.
%   T  : inputs {r, rd, rdd} (position ref and derivatives), outputs {y, u}
%   Lu : loop transfer at the servo command (negative-feedback convention)

Ts = ctrl.Ts; K = ctrl.K;
Gp  = c2d(ss(ctrl.Gfull), Ts, 'zoh');       Gp.InputName = 'u';  Gp.OutputName = 'y';
Dly = ss(0, 1, 1, 0, Ts);                   Dly.InputName = 'u'; Dly.OutputName = 'ud';
Est = ss(ctrl.est.A, ctrl.est.B, ctrl.est.A, ctrl.est.B, Ts);
Est.InputName = {'ud', 'y'};                Est.OutputName = {'xh', 'vh'};
Int = ss(1, [Ts -Ts], 1, [0 0], Ts);        Int.InputName = {'r', 'y'}; Int.OutputName = 'xi';
% u = -K [xh - r; vh - rd; xi] + ff * rdd
Kg = ss([-K K(1) K(2) ctrl.ff]); Kg.Ts = Ts;
Kg.InputName = {'xh', 'vh', 'xi', 'r', 'rd', 'rdd'}; Kg.OutputName = 'u';
T = connect(Gp, Dly, Est, Int, Kg, {'r', 'rd', 'rdd'}, {'y', 'u'}, 'u');
Lu = getLoopTransfer(T, 'u', -1);
end
