function model = export_fixed_leg_model(leg_lengths, sample_time_s, source_path)
%EXPORT_FIXED_LEG_MODEL Export an isolated, fixed-leg experimental model.
% Run from this directory:
%   model = export_fixed_leg_model();                 % 0.24 m, 4 ms
%   model = export_fixed_leg_model([0.24 0.24], 0.004);
% Requires Symbolic Math Toolbox and Control System Toolbox.
% Reads the original model; never edits it or runs its K fitting/export loop.
% Writes only tools/chassis/model/generated/fixed_leg_*.mat and *.json.
% This tool does not change the firmware, its K table or its control period.
%
% Ac/Bc: continuous model; Ad/Bd: exact zero-order-hold discretization.
% Q/R: source weights reused as DISCRETE per-stage weights for this experiment.
% They are not an exact discretization of the continuous-time cost integral.
% Kd/Pd: full-state discrete LQR reference/terminal weight for later MPC tests.
% The firmware currently omits some opposite-leg feedback terms; Kd is not
% a reproduction of that implementation and must not replace its K table.

if nargin < 1 || isempty(leg_lengths), leg_lengths = [0.24 0.24]; end
if nargin < 2 || isempty(sample_time_s), sample_time_s = 0.004; end
model_dir = fileparts(mfilename('fullpath'));
project_root = fileparts(fileparts(fileparts(model_dir)));
if nargin < 3 || isempty(source_path)
    source_path = fullfile(fileparts(project_root), 'LQR拟合', ...
        'WHU-Balance-Model-2025', 'LQR_Legged_Wheeled.m');
end
validateattributes(leg_lengths, {'numeric'}, ...
    {'real', 'finite', 'positive', 'vector', 'nonempty'});
if isscalar(leg_lengths), leg_lengths = [leg_lengths leg_lengths]; end
assert(numel(leg_lengths) == 2, 'WHU:MPC:LegLength', ...
    'Specify a scalar leg length or [left right], in metres.');
leg_lengths = double(reshape(leg_lengths, 1, 2));
validateattributes(sample_time_s, {'numeric'}, ...
    {'real', 'finite', 'positive', 'scalar'});
sample_time_s = double(sample_time_s);
assert(isfile(source_path), 'WHU:MPC:MissingSource', ...
    'Source model not found: %s', source_path);
assert(~isempty(which('sym')) && ~isempty(which('dlqr')), ...
    'WHU:MPC:Toolboxes', 'Symbolic Math and Control System toolboxes are required.');

source_text = fileread(source_path);
[Ac, Bc, Q, R, parameters, leg_tables] = read_source_model(source_text, leg_lengths);
assert(isequal(size(Ac), [10 10]) && isequal(size(Bc), [10 4]), ...
    'WHU:MPC:Dimensions', 'Expected 10 states and 4 inputs.');
assert(all(isfinite(Ac(:))) && all(isfinite(Bc(:))), ...
    'WHU:MPC:Finite', 'Model matrices contain NaN/Inf.');
assert(isequal(size(Q), [10 10]) && isequal(size(R), [4 4]), ...
    'WHU:MPC:Weights', 'Unexpected Q/R dimensions.');
assert(norm(Q-Q.', 'fro') < 1e-10 && min(eig(Q)) >= -1e-10, ...
    'WHU:MPC:Weights', 'Q must be symmetric positive semidefinite.');
assert(norm(R-R.', 'fro') < 1e-10 && min(eig(R)) > 0, ...
    'WHU:MPC:Weights', 'R must be symmetric positive definite.');

% Block exponential also works for singular Ac (position/yaw integrators).
zoh = expm([Ac Bc; zeros(4,14)] * sample_time_s);
Ad = zoh(1:10,1:10);
Bd = zoh(1:10,11:14);
% Independent toolbox discretization check.
sysd = c2d(ss(Ac, Bc, eye(10), zeros(10,4)), sample_time_s, 'zoh');
assert(norm(Ad-sysd.A,'fro') < 1e-10 && norm(Bd-sysd.B,'fro') < 1e-10, ...
    'WHU:MPC:Discretization', 'Zero-order-hold discretization check failed.');
[Kd, Pd, closed_loop_poles] = dlqr(Ad, Bd, Q, R);
assert(all(abs(closed_loop_poles) < 1), ...
    'WHU:MPC:Stability', 'Discrete LQR reference is not stable.');
riccati_residual = Ad.'*Pd*Ad - Pd + Q ...
    - (Ad.'*Pd*Bd)*((R+Bd.'*Pd*Bd)\(Bd.'*Pd*Ad));
relative_residual = norm(riccati_residual,'fro') / max(1,norm(Pd,'fro'));
assert(relative_residual < 1e-8, 'WHU:MPC:Riccati', ...
    'Discrete Riccati residual is too large.');

model.schema_version = 1;
model.model_kind = 'fixed_leg_linear_zero_equilibrium';
model.source_path = char(source_path);
info = dir(source_path);
model.source_modified = info.date;
model.source_bytes = info.bytes;
model.exported_at = char(datetime('now', 'TimeZone', 'Asia/Shanghai', ...
    'Format', 'yyyy-MM-dd''T''HH:mm:ss'));
model.export_time_zone = 'Asia/Shanghai';
model.nx = 10;
model.nu = 4;
model.leg_lengths_m = leg_lengths;
model.sample_time_s = sample_time_s;
model.state_names = {'s','ds','yaw','dyaw','theta_L','dtheta_L', ...
    'theta_R','dtheta_R','pitch','dpitch'};
model.state_units = {'m','m/s','rad','rad/s','rad','rad/s', ...
    'rad','rad/s','rad','rad/s'};
model.input_names = {'T_wL','T_wR','T_bL','T_bR'};
model.input_units = {'N*m','N*m','N*m','N*m'};
model.state_convention = ['Ground-frame model coordinates; yaw continuous; ' ...
    'theta_L=Q0_L+pitch, theta_R=-Q0_R+pitch; zero linearization equilibrium.'];
model.input_convention = ['Generalized model torques before motor polarity/VMC; ' ...
    'not physical joint-motor commands.'];
model.weight_convention = 'Source Q/R reused as discrete per-stage weights.';
model.parameters = parameters;
model.leg_tables = leg_tables;
model.Ac = Ac;
model.Bc = Bc;
model.Ad = Ad;
model.Bd = Bd;
model.Q = Q;
model.R = R;
model.Kd = Kd;
model.Pd = Pd;
model.validation.controllability_rank = rank(ctrb(Ad,Bd));
model.validation.closed_loop_poles_real_imag = ...
    [real(closed_loop_poles), imag(closed_loop_poles)];
model.validation.max_closed_loop_pole_magnitude = max(abs(closed_loop_poles));
model.validation.relative_riccati_residual = relative_residual;

output_dir = fullfile(model_dir, 'generated');
if ~isfolder(output_dir), mkdir(output_dir); end
stem = sprintf('fixed_leg_L%.3f_R%.3f_Ts%.6f', ...
    leg_lengths(1), leg_lengths(2), sample_time_s);
stem = strrep(stem, '.', 'p');
mat_path = fullfile(output_dir, [stem '.mat']);
json_path = fullfile(output_dir, [stem '.json']);
save(mat_path, '-struct', 'model', '-v7'); % scipy.io.loadmat compatible
fid = fopen(json_path, 'w', 'n', 'UTF-8');
assert(fid ~= -1, 'WHU:MPC:Export', 'Cannot open JSON output: %s', json_path);
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, '%s\n', jsonencode(model, 'PrettyPrint', true));
clear cleanup;
fprintf('Fixed legs [%.3f %.3f] m; Ts=%.6f s; nx=10, nu=4\n', ...
    leg_lengths(1), leg_lengths(2), sample_time_s);
fprintf('Controllability rank: %d; max closed-loop |pole|: %.9f\n', ...
    model.validation.controllability_rank, max(abs(closed_loop_poles)));
fprintf('Exported:\n  %s\n  %s\n', mat_path, json_path);
end

function [Ac, Bc, Q, R, parameters, leg_tables] = read_source_model(text, legs)
% Evaluate only the trusted local source's modeling/parameter prefix in this
% function workspace. No run(source_path): its clear all and fitting loop
% would affect the session and generate unrelated output.
marker = regexp(text, '(?m)^%[^\r\n]*Step 2\.5[^\r\n]*', 'start', 'once');
assert(~isempty(marker), 'WHU:MPC:SourceLayout', ...
    'Source Step 2.5 marker missing; review exporter for the new source layout.');
prefix = text(1:marker-1);
prefix = regexprep(prefix, '(?m)^\s*(clear all|clear|clc|tic)\s*;?\s*$', '');
eval(prefix);

assert(exist('Leg_data_l','var') && exist('Leg_data_r','var') ...
    && exist('A','var') && exist('B','var') && exist('lqr_Q','var') ...
    && exist('lqr_R','var'), 'WHU:MPC:SourceLayout', ...
    'Source does not provide the expected model, tables and weights.');
left = select_leg_row(Leg_data_l, legs(1));
right = select_leg_row(Leg_data_r, legs(2));
symbols = [R_w R_l l_l l_r l_wl l_wr l_bl l_br l_c ...
    m_w m_l m_b I_w I_ll I_lr I_b I_z g];
values = [R_w_ac R_l_ac legs(1) legs(2) left(2) right(2) ...
    left(3) right(3) l_c_ac m_w_ac m_l_ac m_b_ac I_w_ac ...
    left(4) right(4) I_b_ac I_z_ac g_ac];
Ac = double(subs(A, symbols, values));
Bc = double(subs(B, symbols, values));
Q = double(lqr_Q);
R = double(lqr_R);
names = {'R_w','R_l','l_l','l_r','l_wl','l_wr','l_bl','l_br','l_c', ...
    'm_w','m_l','m_b','I_w','I_ll','I_lr','I_b','I_z','g'};
parameters = cell2struct(num2cell(values), names, 2);
leg_tables.left = double(Leg_data_l);
leg_tables.right = double(Leg_data_r);
leg_tables.columns = {'leg_length_m','wheel_to_leg_com_m', ...
    'body_to_leg_com_m','leg_inertia_kg_m2'};
leg_tables.selection = 'Exact tabulated row where present; otherwise linear interpolation, no extrapolation.';
end

function row = select_leg_row(table, leg_length)
assert(size(table,2) == 4 && size(table,1) >= 2 ...
    && all(isfinite(table(:))) && all(diff(table(:,1)) > 0), ...
    'WHU:MPC:LegTable', 'Leg table must have four finite columns and increasing lengths.');
assert(leg_length >= table(1,1) && leg_length <= table(end,1), ...
    'WHU:MPC:LegRange', 'Requested leg length is outside the source table.');
index = find(abs(table(:,1)-leg_length) < 1e-12, 1);
if isempty(index)
    row = [leg_length interp1(table(:,1),table(:,2:4),leg_length,'linear')];
else
    row = double(table(index,:));
end
end
