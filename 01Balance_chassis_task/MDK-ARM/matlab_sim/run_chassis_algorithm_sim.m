function results = run_chassis_algorithm_sim()
%RUN_CHASSIS_ALGORITHM_SIM Replay the embedded five-bar/LQR algorithm.
%
% This is an algorithm-level digital twin of the code in chassis_task.c and
% LQR.c. It compares three paths at the firmware sampling period:
%   legacy  - behavior before the recent velocity/kinematics changes
%   problem - the on-vehicle behavior found in the modified worktree
%   fixed   - SI motor speed + exact five-bar dL/dt + corrected right leg
%
% No Simulink or extra toolbox is required. The script reads the active LQR
% and polynomial coefficients directly from the project source so that the
% MATLAB controller stays synchronized with the firmware.

sim_dir = fileparts(mfilename('fullpath'));
project_root = fileparts(fileparts(sim_dir));
p = load_project_parameters(project_root);

t = (0:p.Ts:4.0).';
w = 2*pi*1.35;
tau = 0.30;
env = 1-exp(-t/tau);
env_dot = exp(-t/tau)/tau;

% Q1/Q4 are the degree-valued joint positions passed to the C forward
% kinematics. S1/S4 are the HT protocol velocities in rad/s.
[q1L, s1L] = smooth_joint_motion(t, env, env_dot, -30, 5.0, w, 0.00);
[q4L, s4L] = smooth_joint_motion(t, env, env_dot,   0, 4.0, w, 0.35);
[q1R, s1R] = smooth_joint_motion(t, env, env_dot, -29, 3.5, w, 0.20); % joint 4
[q4R, s4R] = smooth_joint_motion(t, env, env_dot,   1, 5.0, w, 0.65); % joint 3

% A deterministic one-sample velocity pulse exposes the 500 Hz derivative
% path without inventing a plant model. A 2 rad/s outlier is still far below
% the HT protocol's +/-45 rad/s range.
pulse_index = round(2.0/p.Ts)+1;
s4L(pulse_index) = s4L(pulse_index)+2.0;

n = numel(t);
L = zeros(n,2);
Q = zeros(n,2);
dL_exact = zeros(n,2);
dL_problem = zeros(n,2);
dL_legacy = zeros(n,2);
gyro_exact = zeros(n,2);
gyro_problem = zeros(n,2);
gyro_legacy = zeros(n,2);

foot_legacy = zeros(n,2);
foot_problem = zeros(n,2);
foot_fixed = zeros(n,2);
joint_legacy = zeros(n,4);
joint_problem = zeros(n,4);
joint_fixed = zeros(n,4);
virtual_h_legacy = zeros(n,2);
virtual_h_problem = zeros(n,2);
virtual_h_fixed = zeros(n,2);
wheel_bias_only = zeros(n,2);

legacy_velocity_scale = 2*pi/60;
problem_forward_bias = 0.18;
fixed_forward_bias = p.forward_speed;

for k = 1:n
    fkL_problem = five_bar_forward(p, q1L(k), s1L(k), q4L(k), s4L(k), false);
    fkR_problem = five_bar_forward(p, q1R(k), s1R(k), q4R(k), s4R(k), false);
    fkL_fixed = five_bar_forward(p, q1L(k), s1L(k), q4L(k), s4L(k), true);
    fkR_fixed = five_bar_forward(p, q1R(k), s1R(k), q4R(k), s4R(k), true);
    if ~fkL_problem.valid || ~fkR_problem.valid || ~fkL_fixed.valid || ~fkR_fixed.valid
        error('Five-bar singularity at t = %.6f s.', t(k));
    end

    if k == 1
        L(k,:) = [fkL_fixed.L0, fkR_fixed.L0];
    else
        % Exact copy of: leg_length = 0.9*L0 + 0.1*last_leg_length.
        L(k,:) = 0.9*[fkL_fixed.L0, fkR_fixed.L0] + 0.1*L(k-1,:);
    end
    Q(k,:) = [fkL_fixed.Q0, -fkR_fixed.Q0];
    dL_exact(k,:) = [fkL_fixed.dL0, fkR_fixed.dL0];
    % angle_L = Q0 and angle_R = -Q0, so their derivatives must use the
    % same signs. The problem/legacy paths used the opposite signs.
    gyro_exact(k,:) = [fkL_fixed.S0, -fkR_fixed.S0];
    gyro_problem(k,:) = [-fkL_problem.S0, fkR_problem.S0];
    gyro_legacy(k,:) = [-fkL_problem.S0_legacy, fkR_problem.S0_legacy];

    NL = evaluate_vmc_coefficients(p.N, L(k,1), Q(k,1));
    NR = evaluate_vmc_coefficients(p.N, L(k,2), Q(k,2));

    % Problem version exactly matched the modified C code:
    %   left  = N11*qdot1 + N12*qdot2
    %   right = N11*qdot3 + N12*qdot4 (also reversed vs FK Q1/Q4 order)
    dL_problem(k,1) = NL(1)*s1L(k) + NL(2)*s4L(k);
    dL_problem(k,2) = NR(1)*s4R(k) + NR(2)*s1R(k);
    dL_legacy(k,:) = legacy_velocity_scale*dL_problem(k,:);

    K = lqr_at_length(p.K_fit, L(k,1), L(k,2));
    wheel_bias_only(k,:) = [K(1,2),-K(2,2)]*problem_forward_bias*p.TORQ_K;

    [foot_legacy(k,:), virtual_h_legacy(k,:), joint_legacy(k,:)] = ...
        controller_sample(p, K, L(k,:), Q(k,:), gyro_legacy(k,:), ...
                          gyro_legacy(k,2), 0.0);

    % The problem version overwrote right wheel feedback with leg_gyro_L.
    [foot_problem(k,:), virtual_h_problem(k,:), joint_problem(k,:)] = ...
        controller_sample(p, K, L(k,:), Q(k,:), gyro_problem(k,:), ...
                          gyro_problem(k,1), problem_forward_bias);

    [foot_fixed(k,:), virtual_h_fixed(k,:), joint_fixed(k,:)] = ...
        controller_sample(p, K, L(k,:), Q(k,:), gyro_exact(k,:), ...
                          gyro_exact(k,2), fixed_forward_bias);
end

ddL_legacy = filtered_derivative(dL_legacy, p.Ts, p.alpha_da);
ddL_problem = filtered_derivative(dL_problem, p.Ts, p.alpha_da);
ddL_fixed = filtered_derivative(dL_exact, p.Ts, p.alpha_da);

support_legacy = support_force_trace(p, L, Q, ddL_legacy, virtual_h_legacy);
support_problem = support_force_trace(p, L, Q, ddL_problem, virtual_h_problem);
support_fixed = support_force_trace(p, L, Q, ddL_fixed, virtual_h_fixed);

offground_legacy = support_legacy <= 0 & L > 0.13;
offground_problem = support_problem <= 0 & L > 0.13;
offground_fixed = support_fixed <= 0 & L > 0.13;

gyro_ratio = rms_local(gyro_problem(:))/max(rms_local(gyro_legacy(:)), eps);
gyro_fixed_problem_error = rms_local(gyro_problem(:)-gyro_exact(:)) / ...
                           max(rms_local(gyro_exact(:)), eps);
gyro_from_angle = [gradient(Q(:,1),p.Ts),gradient(Q(:,2),p.Ts)];
derivative_mask = true(n,1);
derivative_mask([1,n,max(1,pulse_index-1):min(n,pulse_index+1)]) = false;
fixed_gyro_derivative_error = rms_local(gyro_exact(derivative_mask,:)-gyro_from_angle(derivative_mask,:)) / ...
                              max(rms_local(gyro_from_angle(derivative_mask,:)),eps);
dlength_problem_error = rms_local(dL_problem(:)-dL_exact(:)) / ...
                        max(rms_local(dL_exact(:)), eps);
forward_bias_output = mean(abs(wheel_bias_only(:)));
problem_offground_samples = nnz(offground_problem & ~offground_fixed);
problem_joint_saturation = 100*nnz(abs(joint_problem) >= p.T_max-1e-6) / ...
                           numel(joint_problem);

summary = table(gyro_ratio, gyro_fixed_problem_error, fixed_gyro_derivative_error, ...
                dlength_problem_error, forward_bias_output, ...
                problem_offground_samples, problem_joint_saturation, ...
    'VariableNames', {'problem_to_legacy_gyro_rms_ratio', ...
                      'problem_gyro_relative_rms_error', ...
                      'fixed_gyro_vs_angle_derivative_error', ...
                      'problem_dlength_relative_rms_error', ...
                      'zero_command_wheel_bias_counts', ...
                      'problem_only_offground_samples', ...
                      'problem_joint_saturation_percent'});
writetable(summary, fullfile(sim_dir, 'sim_summary.csv'));

fig = figure('Color','w','Visible','off','Position',[100 100 1450 900]);
tl = tiledlayout(fig,3,2,'TileSpacing','compact','Padding','compact');
title(tl,'Embedded chassis algorithm replay: legacy vs problem vs fixed');

nexttile;
plot(t,gyro_legacy(:,1),'--',t,gyro_problem(:,1),':',t,gyro_exact(:,1),'-','LineWidth',1.1);
grid on; ylabel('rad/s'); title('Left leg angular velocity');
legend('legacy formula','problem sign','fixed d(angle)/dt','Location','best');

nexttile;
plot(t,dL_legacy(:,1),'--',t,dL_problem(:,1),':',t,dL_exact(:,1),'-','LineWidth',1.1);
grid on; ylabel('m/s'); title('Left leg length velocity');
legend('legacy scaled wrong J','problem wrong J','fixed exact dL/dt','Location','best');

nexttile;
plot(t,support_legacy(:,1),'--',t,support_problem(:,1),':',t,support_fixed(:,1),'-','LineWidth',1.1);
yline(0,'k-'); grid on; ylabel('N'); title('Estimated left support force');
legend('legacy','problem','fixed','0 N threshold','Location','best');

nexttile;
plot(t,foot_legacy(:,2),'--',t,foot_problem(:,2),':',t,foot_fixed(:,2),'-','LineWidth',1.1);
grid on; ylabel('command counts'); title('Right wheel LQR output');
legend('legacy','problem: left gyro + 0.18 m/s','fixed','Location','best');

nexttile;
plot(t,joint_problem,'LineWidth',0.9); hold on;
yline(p.T_max,'k--'); yline(-p.T_max,'k--'); grid on;
xlabel('time (s)'); ylabel('N m'); title('Problem-version joint commands after clamp');

nexttile;
stairs(t,any(offground_legacy,2),'--'); hold on;
stairs(t,any(offground_problem,2),':');
stairs(t,any(offground_fixed,2),'-');
ylim([-0.05 1.05]); grid on; xlabel('time (s)'); ylabel('flag');
title('Off-ground decision'); legend('legacy','problem','fixed','Location','best');

exportgraphics(fig, fullfile(sim_dir,'chassis_algorithm_sim_result.png'),'Resolution',160);
close(fig);

fprintf('\nChassis algorithm replay complete.\n');
fprintf('  Problem leg gyro RMS is %.3f x the legacy formula.\n', gyro_ratio);
fprintf('  Problem leg gyro relative RMS error vs angle derivative: %.1f %%\n', 100*gyro_fixed_problem_error);
fprintf('  Fixed leg gyro numerical derivative check error: %.3f %%\n', 100*fixed_gyro_derivative_error);
fprintf('  Problem dL/dt relative RMS error vs exact FK: %.1f %%\n', 100*dlength_problem_error);
fprintf('  Added 0.18 m/s zero-command wheel bias: %.1f command counts (mean abs).\n', forward_bias_output);
fprintf('  Problem-only false off-ground samples: %d of %d.\n', problem_offground_samples, 2*n);
fprintf('  Problem joint command saturation: %.1f %% of samples.\n', problem_joint_saturation);
fprintf('  Figure: %s\n', fullfile(sim_dir,'chassis_algorithm_sim_result.png'));
fprintf('  Summary: %s\n\n', fullfile(sim_dir,'sim_summary.csv'));

results = struct('time',t,'length',L,'angle',Q, ...
    'gyro_legacy',gyro_legacy,'gyro_problem',gyro_problem,'gyro_fixed',gyro_exact, ...
    'dlength_legacy',dL_legacy,'dlength_problem',dL_problem, ...
    'dlength_fixed',dL_exact,'support_legacy',support_legacy, ...
    'support_problem',support_problem,'support_fixed',support_fixed, ...
    'foot_legacy',foot_legacy,'foot_problem',foot_problem, ...
    'foot_fixed',foot_fixed,'summary',summary);
end

function p = load_project_parameters(project_root)
h_text = read_source_bytes(fullfile(project_root,'application','chassis_task.h'));
c_text = read_source_bytes(fullfile(project_root,'application','chassis_task.c'));
lqr_text = read_source_bytes(fullfile(project_root,'components','algorithm','LQR.c'));

p.Ts = parse_define(h_text,'CHASSIS_CONTROL_TIME');
p.L1 = parse_define(h_text,'L1');
p.L2 = parse_define(h_text,'L2');
p.L3 = parse_define(h_text,'L3');
p.L4 = parse_define(h_text,'L4');
p.L5 = parse_define(h_text,'L5');
p.T_max = parse_define(fullfile_text(project_root,'application','CAN_receive.h'),'T_MAX');
p.TORQ_K = parse_define(h_text,'TORQ_K');
p.FEED_f = parse_define(h_text,'FEED_f');
p.forward_speed = parse_define(c_text,'FORWARD_SPEED');
p.g = parse_assignment(c_text,'const\s+fp32\s+g');
p.m_w = parse_assignment(c_text,'const\s+fp32\s+m_w');
p.alpha_da = parse_assignment(c_text,'alpha_da');

names = {'N11','N12','N21','N22'};
p.N = zeros(4,6);
for r = 1:4
    for c = 0:5
        pattern = ['InverseJacobianCoefficient\.',names{r},'\.c',num2str(c), ...
                   '\s*=\s*([-+0-9.eE]+)f?'];
        token = regexp(c_text,pattern,'tokens','once');
        if isempty(token)
            error('Cannot parse %s.c%d from chassis_task.c.',names{r},c);
        end
        p.N(r,c+1) = str2double(token{1});
    end
end

no_comments = regexprep(lqr_text,'//[^\r\n]*','');
block = regexp(no_comments, ...
    'float\s+K_Fit\s*\[40\]\s*\[6\]\s*=\s*\{(?<body>.*?)\};', ...
    'names','once');
if isempty(block)
    error('Cannot parse active K_Fit matrix from LQR.c.');
end
tokens = regexp(block.body,'[-+]?(?:\d+\.\d*|\d*\.\d+|\d+)(?:[eE][-+]?\d+)?','match');
values = str2double(tokens);
if numel(values) ~= 240
    error('Expected 240 active K_Fit values, found %d.',numel(values));
end
p.K_fit = reshape(values,6,40).';
end

function text = fullfile_text(root,varargin)
text = read_source_bytes(fullfile(root,varargin{:}));
end

function text = read_source_bytes(path)
% The legacy Keil sources use GB18030 while newer files may use UTF-8. All
% parsed symbols are ASCII, so byte-preserving input is encoding-independent.
fid = fopen(path,'rb');
if fid < 0, error('Cannot open source file: %s',path); end
cleanup = onCleanup(@() fclose(fid));
text = char(fread(fid,Inf,'*uint8').');
end

function value = parse_define(text,name)
token = regexp(text,['#define\s+',name,'\s+([-+0-9.eE]+)f?'],'tokens','once');
if isempty(token), error('Cannot parse #define %s.',name); end
value = str2double(token{1});
end

function value = parse_assignment(text,name_pattern)
token = regexp(text,[name_pattern,'\s*=\s*([-+0-9.eE]+)f?'],'tokens','once');
if isempty(token), error('Cannot parse assignment %s.',name_pattern); end
value = str2double(token{1});
end

function [q_deg,s_rad] = smooth_joint_motion(t,env,env_dot,q0,amp,w,phase)
osc = sin(w*t+phase)-sin(phase);
q_deg = q0 + amp*env.*osc;
qdot_deg = amp*(env_dot.*osc + env.*w.*cos(w*t+phase));
s_rad = deg2rad(qdot_deg);
end

function out = five_bar_forward(p,Q1_deg,S1,Q4_deg,S4,correct_velocity_sign)
Q1 = deg2rad(180+Q1_deg);
Q4 = deg2rad(180-Q4_deg);
xb = -p.L5/2 + p.L1*cos(Q1);
xd =  p.L5/2 - p.L4*cos(Q4);
yb = p.L1*sin(Q1);
yd = p.L4*sin(Q4);
Lbd = (xd-xb)^2 + (yd-yb)^2;
A0 = 2*p.L2*(xd-xb);
B0 = 2*p.L2*(yd-yb);
C0 = p.L2^2 + Lbd - p.L3^2;
disc = A0^2+B0^2-C0^2;
out.valid = disc >= -1e-8;
disc = max(disc,0);
Q2 = 2*atan2(B0+sqrt(disc),A0+C0);
xc = xb+cos(Q2)*p.L2;
yc = yb+sin(Q2)*p.L2;
out.L0 = hypot(xc,yc);
out.Q0 = atan2(xc,yc);

vxb = -S1*p.L1*sin(Q1);
vyb =  S1*p.L1*cos(Q1);
vxd = -S4*p.L4*sin(Q4);
if correct_velocity_sign
    vyd = -S4*p.L4*cos(Q4); % Q4 = pi-q4, therefore Q4_dot = -S4
else
    vyd =  S4*p.L4*cos(Q4); % problem/legacy firmware behavior
end
Q3 = atan2(yc-yd,xc-xd);
den = p.L2*sin(Q3-Q2);
if abs(den) <= 1e-6 || out.L0 <= 1e-6
    out.valid = false;
    out.S0 = 0;
    out.S0_legacy = 0;
    out.dL0 = 0;
    return;
end
S2 = ((vxd-vxb)*cos(Q3)+(vyd-vyb)*sin(Q3))/den;
vxc = vxb-S2*p.L2*sin(Q2);
vyc = vyb+S2*p.L2*cos(Q2);
out.S0 = (cos(out.Q0)*vxc-sin(out.Q0)*vyc)/out.L0;
out.dL0 = (xc*vxc+yc*vyc)/out.L0;
out.S0_legacy = 3*(-sin(abs(out.Q0))*vxc-cos(out.Q0)*vyc);
end

function N = evaluate_vmc_coefficients(coeff,L0,Q0)
basis = [1,L0,Q0,L0^2,L0*Q0,Q0^2].';
N = coeff*basis; % [N11; N12; N21; N22]
end

function K = lqr_at_length(K_fit,LL,LR)
basis = [1,LL,LR,LL^2,LL*LR,LR^2].';
K = reshape(K_fit*basis,10,4).';
end

function [foot,horizontal,joint] = controller_sample(p,K,L,Q,gyro,right_wheel_gyro,forward_bias)
horizontal(1) = K(3,5)*(-Q(1)) + K(3,6)*(-gyro(1));
horizontal(2) = -(K(4,7)*(-Q(2)) + K(4,8)*(-gyro(2)));

foot(1) = (K(1,5)*(-Q(1)) + K(1,6)*(-gyro(1)) + ...
           K(1,2)*forward_bias)*p.TORQ_K;
foot(2) = -(K(2,7)*(-Q(2)) + K(2,8)*(-right_wheel_gyro))*p.TORQ_K ...
          -K(2,2)*forward_bias*p.TORQ_K;

NL = evaluate_vmc_coefficients(p.N,L(1),Q(1));
NR = evaluate_vmc_coefficients(p.N,L(2),Q(2));
joint = [horizontal(1)*(-NL(2))+p.FEED_f*NL(1), ...
         horizontal(1)*NL(4)-p.FEED_f*NL(3), ...
         horizontal(2)*(-NR(2))-p.FEED_f*NR(1), ...
         horizontal(2)*NR(4)+p.FEED_f*NR(3)];
joint = min(max(joint,-p.T_max),p.T_max);
end

function dd = filtered_derivative(d,Ts,alpha)
dd = zeros(size(d));
for k = 2:size(d,1)
    raw = (d(k,:)-d(k-1,:))/Ts;
    dd(k,:) = alpha*raw+(1-alpha)*dd(k-1,:);
end
end

function support = support_force_trace(p,L,Q,ddL,horizontal)
support = zeros(size(L));
for k = 1:size(L,1)
    generalized = p.FEED_f*cos(Q(k,:)) + horizontal(k,:).*sin(Q(k,:))./L(k,:);
    generalized = min(max(generalized,-100),100);
    raw = generalized + p.m_w*p.g - p.m_w*ddL(k,:).*cos(Q(k,:));
    if k == 1
        support(k,:) = raw;
    else
        support(k,:) = 0.7*raw+0.3*support(k-1,:);
    end
end
end

function y = rms_local(x)
y = sqrt(mean(x.^2));
end
