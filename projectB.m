% ============================================================
%  jansen_demo.m
%  IE410: Introduction to Robotics — Project Part B
%  12-Link Modified Jansen Mechanism Simulation
%  Jadav et al., SN Applied Sciences, 2024
% ============================================================
%  Run this script in MATLAB and screen-record for the video.
%
%  What it shows:
%    Figure 1 — Live animated mechanism (2 crank cycles)
%    Figure 2 — Foot trajectory + gait metrics table
%    Figure 3 — Sensitivity analysis (L1, L4, L8 variation)
%
%  Usage:
%    >> jansen_demo
% ============================================================

clear; close all; clc;

fprintf('==============================================\n');
fprintf('  IE410 Project Part B — Jansen Mechanism\n');
fprintf('  12-Link Modified Jansen Simulation\n');
fprintf('==============================================\n\n');

% ── Link lengths (cm) — Jadav et al. 2024 ──────────────────
L1  = 11.29;   % Crank            (adjustable)
L2  = 45.00;   % Upper coupler
L3  = 36.00;   % Upper rocker
L4  = 32.93;   % Ground pivot spacing (adjustable)
L5  = 48.50;   % Upper triangle
L6  = 41.50;   % Upper triangle
L7  = 60.50;   % Lower coupler
L8  = 41.78;   % Lower rocker     (adjustable)
L9  = 42.00;   % Lower triangle
L10 = 43.00;   % Lower triangle
L11 = 26.50;   % Foot link
L12 = 54.50;   % Foot link

% Fixed ground pivots
P0 = [0, 0];       % Crank pivot (origin)
P3 = [L4, 0];      % Secondary ground pivot

% ── Sweep: solve full crank revolution ──────────────────────
N      = 360;
thetas = linspace(0, 2*pi, N+1);
thetas(end) = [];

joints = nan(N, 8, 2);   % (frame, joint, xy)
foot   = nan(N, 2);

fprintf('Solving kinematics for %d crank angles...\n', N);
for i = 1:N
    th = thetas(i);

    P1 = P0 + L1 * [cos(th), sin(th)];

    P2 = cci(P1, L2, P3, L3, +1);   % Upper four-bar
    if any(isnan(P2)), continue; end

    P5 = cci(P1, L7, P3, L8, -1);   % Lower four-bar
    if any(isnan(P5)), continue; end

    P4 = cci(P2, L5, P3, L6, +1);   % Upper triangle
    if any(isnan(P4)), continue; end

    P6 = cci(P4, L10, P5, L9, +1);  % Lower triangle
    if any(isnan(P6)), continue; end

    PE = cci(P5, L11, P6, L12, -1); % Foot
    if any(isnan(PE)), continue; end

    joints(i,:,:) = [P0; P1; P2; P3; P4; P5; P6; PE];
    foot(i,:)     = PE;
end

% ── Gait metrics ─────────────────────────────────────────────
valid = ~isnan(foot(:,1));
fx = foot(valid, 1);
fy = foot(valid, 2);

x_span   = max(fx) - min(fx);
y_span   = max(fy) - min(fy);
y_thresh = min(fy) + 0.15 * y_span;

% Shoelace formula (AUCT)
fc = [fx; fx(1)];
gc = [fy; fy(1)];
area_auct = 0.5 * abs(sum(fc(1:end-1).*gc(2:end) - fc(2:end).*gc(1:end-1)));

stance_mask = fy <= y_thresh;
duty        = sum(stance_mask) / length(fy);

fprintf('\n===== GAIT METRICS =====\n');
fprintf('  Stride length (X-span):  %.2f cm  [ref: 50.02 cm]\n', x_span);
fprintf('  Step height   (Y-span):  %.2f cm  [ref: 12.81 cm]\n', y_span);
fprintf('  Enclosed area (AUCT):    %.2f cm^2\n', area_auct);
fprintf('  Duty factor:             %.1f%%    [human ~60%%]\n', duty*100);
fprintf('  Stride/Height ratio:     %.2f\n', x_span/y_span);
fprintf('========================\n\n');

% Grashof check on primary four-bar (L1, L2, L3, L4)
primary = sort([L1, L2, L3, L4]);
s_ = primary(1); p_ = primary(2); q_ = primary(3); l_ = primary(4);
if (s_ + l_) <= (p_ + q_)
    fprintf('Grashof condition: SATISFIED  (crank can rotate fully)\n\n');
else
    fprintf('Grashof condition: NOT satisfied\n\n');
end

% ── Axis limits ───────────────────────────────────────────────
all_x = joints(:,:,1); all_x = all_x(~isnan(all_x));
all_y = joints(:,:,2); all_y = all_y(~isnan(all_y));
pad   = 12;
xl    = [min(all_x)-pad, max(all_x)+pad];
yl    = [min(all_y)-pad, max(all_y)+pad];

% ═══════════════════════════════════════════════════════════════
%  FIGURE 1 — Live Mechanism Animation
% ═══════════════════════════════════════════════════════════════
fprintf('[Figure 1] Starting live animation (2 crank cycles)...\n');

BG = [0.04 0.055 0.09];

fig1 = figure('Name', 'Jansen Mechanism — Live Animation', ...
              'Color', BG, 'Position', [30 60 860 760]);
ax1  = axes(fig1, 'Color', BG, 'XColor', [0.6 0.6 0.6], ...
            'YColor', [0.6 0.6 0.6], 'GridColor', [0.12 0.17 0.25], ...
            'GridAlpha', 0.35);
hold(ax1, 'on');
grid(ax1, 'on');
axis(ax1, 'equal');
xlim(ax1, xl); ylim(ax1, yl);
xlabel(ax1, 'X Position (cm)', 'Color', [0.6 0.6 0.6], 'FontSize', 11);
ylabel(ax1, 'Y Position (cm)', 'Color', [0.6 0.6 0.6], 'FontSize', 11);
title(ax1, {'12-Link Modified Jansen Mechanism', ...
            'IE410: Introduction to Robotics — Project Part B'}, ...
      'Color', 'white', 'FontSize', 13, 'FontWeight', 'bold');

% Link colours (matching Python version)
lc = [
    1.00 0.42 0.21;   % L1  crank
    0.31 0.80 0.77;   % L2  upper coupler
    0.31 0.80 0.77;   % L3  upper rocker
    0.58 0.65 0.65;   % L4  ground
    0.18 0.80 0.44;   % L5  upper triangle
    0.18 0.80 0.44;   % L6  upper triangle
    0.91 0.30 0.24;   % L7  lower coupler
    0.91 0.30 0.24;   % L8  lower rocker
    0.61 0.35 0.71;   % L9  lower triangle
    0.61 0.35 0.71;   % L10 lower triangle
    0.95 0.77 0.06;   % L11 foot
    0.95 0.77 0.06;   % L12 foot
];

% Edge pairs [from, to]  (1-indexed: P0=1,P1=2,P2=3,P3=4,P4=5,P5=6,P6=7,PE=8)
edges = [1 2; 2 3; 3 4; 1 4; 3 5; 4 5; 2 6; 6 4; 6 7; 5 7; 6 8; 7 8];

h_links = gobjects(12, 1);
for k = 1:12
    h_links(k) = plot(ax1, nan, nan, '-', 'Color', lc(k,:), ...
                      'LineWidth', 3.8);
end

h_trace  = plot(ax1, nan, nan, '-',  'Color', [1 0.42 0.62], 'LineWidth', 2.2);
h_pivots = plot(ax1, nan, nan, 'd',  'MarkerSize', 10, ...
                'MarkerFaceColor', [0.91 0.35 0.56], ...
                'MarkerEdgeColor', 'white', 'LineWidth', 1.2);
h_jnts   = plot(ax1, nan, nan, 'o',  'MarkerSize', 6, ...
                'MarkerFaceColor', 'white', ...
                'MarkerEdgeColor', [0.17 0.24 0.31], 'LineWidth', 0.8);
h_foot   = plot(ax1, nan, nan, 'o',  'MarkerSize', 11, ...
                'MarkerFaceColor', [0.95 0.77 0.06], ...
                'MarkerEdgeColor', 'white', 'LineWidth', 1.2);

h_badge = text(ax1, xl(1)+6, yl(2)-9, '', ...
               'Color', [0.49 0.86 0.60], 'FontSize', 13, ...
               'FontWeight', 'bold', 'FontName', 'Courier New', ...
               'BackgroundColor', [0.07 0.13 0.22], ...
               'EdgeColor', [0.49 0.86 0.60], 'Margin', 4);

text(ax1, xl(1)+6, yl(1)+9, ...
     sprintf('L1=%.2f  L4=%.2f  L8=%.2f cm', L1, L4, L8), ...
     'Color', [0.58 0.65 0.65], 'FontSize', 9, 'FontName', 'Courier New', ...
     'BackgroundColor', [0.07 0.13 0.22], 'EdgeColor', [0.21 0.29 0.38]);

% Animate 2 full cycles
tx = []; ty = [];
for rep = 1:2
    for i = 1:N
        J = squeeze(joints(i,:,:));   % (8 x 2)
        if any(isnan(J(:,1))), continue; end

        % Foot trace accumulates
        tx(end+1) = J(8,1);
        ty(end+1) = J(8,2);
        set(h_trace, 'XData', tx, 'YData', ty);

        % Links
        for k = 1:12
            a = edges(k,1); b = edges(k,2);
            set(h_links(k), 'XData', [J(a,1), J(b,1)], ...
                            'YData', [J(a,2), J(b,2)]);
        end

        % Fixed pivots (joints 1 and 4 = P0 and P3)
        set(h_pivots, 'XData', J([1,4],1), 'YData', J([1,4],2));

        % Moving joints (2,3,5,6,7)
        mov = [2,3,5,6,7];
        set(h_jnts, 'XData', J(mov,1), 'YData', J(mov,2));

        % Foot marker
        set(h_foot, 'XData', J(8,1), 'YData', J(8,2));

        % Angle badge
        set(h_badge, 'String', sprintf('  theta = %6.1f deg  ', ...
            rad2deg(thetas(i))));

        drawnow limitrate;
        pause(0.022);
    end
end

% ═══════════════════════════════════════════════════════════════
%  FIGURE 2 — Foot Trajectory + Gait Metrics Table
% ═══════════════════════════════════════════════════════════════
fprintf('[Figure 2] Foot trajectory and metrics...\n');

fig2 = figure('Name', 'Foot Trajectory & Gait Metrics', ...
              'Color', BG, 'Position', [30 60 1100 680]);

% Left panel: coloured trajectory
ax2 = subplot(1, 2, 1);
ax2.Color  = BG;
ax2.XColor = [0.6 0.6 0.6];
ax2.YColor = [0.6 0.6 0.6];
hold(ax2, 'on'); grid(ax2, 'on'); axis(ax2, 'equal');
xlabel(ax2, 'X (cm)', 'Color', [0.6 0.6 0.6]);
ylabel(ax2, 'Y (cm)', 'Color', [0.6 0.6 0.6]);
title(ax2, 'Foot Trajectory — Stance vs Swing', ...
      'Color', 'white', 'FontWeight', 'bold', 'FontSize', 12);

plot(ax2, fx, fy, '-', 'Color', [0.6 0.6 0.6], 'LineWidth', 0.8);

h_st = plot(ax2, fx(stance_mask),  fy(stance_mask),  'o', ...
            'Color', [0.20 0.60 0.86], 'MarkerSize', 4, ...
            'MarkerFaceColor', [0.20 0.60 0.86], 'DisplayName', ...
            sprintf('Stance  (%.0f%%)', duty*100));
h_sw = plot(ax2, fx(~stance_mask), fy(~stance_mask), 'o', ...
            'Color', [1 0.42 0.62], 'MarkerSize', 4, ...
            'MarkerFaceColor', [1 0.42 0.62], 'DisplayName', ...
            sprintf('Swing  (%.0f%%)', (1-duty)*100));
legend(ax2, [h_st, h_sw], 'Location', 'northwest', ...
       'TextColor', 'white', 'Color', [0.07 0.13 0.22], ...
       'EdgeColor', [0.3 0.3 0.4]);

% Annotate metrics on the plot
text(ax2, min(fx)+1, max(fy)-2, ...
     sprintf('Stride: %.2f cm\nHeight: %.2f cm\nAUCT: %.1f cm^2', ...
             x_span, y_span, area_auct), ...
     'Color', [0.49 0.86 0.60], 'FontSize', 10, ...
     'BackgroundColor', [0.07 0.13 0.22], 'EdgeColor', [0.49 0.86 0.60], ...
     'VerticalAlignment', 'top');

% Right panel: results table
ax3 = subplot(1, 2, 2);
ax3.Color   = BG;
ax3.Visible = 'off';
title(ax3, 'Simulation Results vs Reference', ...
      'Color', 'white', 'FontWeight', 'bold', 'FontSize', 12, ...
      'Visible', 'on');

metrics_data = {
    'Stride length',       sprintf('%.2f cm', x_span),        '50.02 cm';
    'Step height',         sprintf('%.2f cm', y_span),        '12.81 cm';
    'Duty factor',         sprintf('%.1f%%',  duty*100),      '~60% (human)';
    'Area (AUCT)',         sprintf('%.1f cm^2', area_auct),   'N/A';
    'Grashof condition',   'SATISFIED',                       'Required';
};

tbl = uitable(fig2, ...
    'Data',        metrics_data, ...
    'ColumnName',  {'Metric', 'Our Result', 'Jadav 2024'}, ...
    'RowName',     {}, ...
    'Units',       'normalized', ...
    'Position',    [0.52 0.12 0.46 0.72], ...
    'FontSize',    12, ...
    'ColumnWidth', {180, 130, 150});

sgtitle(fig2, 'Simulation Results — IE410 Project Part B', ...
        'Color', 'white', 'FontSize', 14, 'FontWeight', 'bold');

% ═══════════════════════════════════════════════════════════════
%  FIGURE 3 — Sensitivity Analysis
% ═══════════════════════════════════════════════════════════════
fprintf('[Figure 3] Sensitivity analysis (varying L1, L4, L8)...\n');

fig3 = figure('Name', 'Sensitivity Analysis — Adjustable Links', ...
              'Color', BG, 'Position', [30 60 1350 600]);

link_keys = {'L1', 'L4', 'L8'};
base_vals  = [L1,   L4,   L8 ];
sens_col   = [0.20 0.60 0.86;   % L1  blue
              0.91 0.30 0.24;   % L4  red
              0.18 0.80 0.44];  % L8  green

Lbase = [L1, L2, L3, L4, L5, L6, L7, L8, L9, L10, L11, L12];
link_idx = [1, 4, 8];   % which element of Lbase each key maps to
scales    = linspace(0.85, 1.15, 7);

for panel = 1:3
    ax_s = subplot(1, 3, panel);
    ax_s.Color  = BG;
    ax_s.XColor = [0.6 0.6 0.6];
    ax_s.YColor = [0.6 0.6 0.6];
    hold(ax_s, 'on'); grid(ax_s, 'on'); axis(ax_s, 'equal');
    xlabel(ax_s, 'X (cm)', 'Color', [0.6 0.6 0.6]);
    if panel == 1
        ylabel(ax_s, 'Y (cm)', 'Color', [0.6 0.6 0.6]);
    end
    title(ax_s, sprintf('Varying %s (±15%%)', link_keys{panel}), ...
          'Color', 'white', 'FontWeight', 'bold', 'FontSize', 11);

    col = sens_col(panel, :);

    for s = scales
        Ltest = Lbase;
        Ltest(link_idx(panel)) = base_vals(panel) * s;

        [sfx, sfy] = sweep_foot(Ltest, N);
        if isempty(sfx), continue; end

        deviation = abs(s - 1.0) / 0.15;
        alph = 0.3 + 0.6 * deviation;
        lw   = 0.9;
        h_line = plot(ax_s, sfx, sfy, '-', 'Color', col, 'LineWidth', lw);
        h_line.Color(4) = min(alph, 0.95);
    end

    % Baseline in bold white
    plot(ax_s, fx, fy, 'w-', 'LineWidth', 2.8, 'DisplayName', 'Baseline');
    legend(ax_s, 'Location', 'northwest', 'TextColor', 'white', ...
           'Color', [0.07 0.13 0.22], 'EdgeColor', [0.3 0.3 0.4]);
end

sgtitle(fig3, 'Effect of Adjustable Links on Foot Trajectory', ...
        'Color', 'white', 'FontSize', 14, 'FontWeight', 'bold');

fprintf('\n[DONE] All 3 figures are open.\n');
fprintf('Screen-record this MATLAB session for your IE410 submission video.\n');

% ═══════════════════════════════════════════════════════════════
%  LOCAL FUNCTIONS
% ═══════════════════════════════════════════════════════════════

function pt = cci(c1, r1, c2, r2, branch)
%CCI  Circle-circle intersection.
%   branch = +1  selects the point to the LEFT  of c1->c2.
%   branch = -1  selects the point to the RIGHT of c1->c2.
    d_vec = c2 - c1;
    d     = norm(d_vec);
    pt    = [nan, nan];
    if d < 1e-12 || d > r1+r2+1e-9 || d < abs(r1-r2)-1e-9
        return;
    end
    a     = (r1^2 - r2^2 + d^2) / (2*d);
    h     = sqrt(max(r1^2 - a^2, 0));
    p_mid = c1 + (a/d) * d_vec;
    perp  = [-d_vec(2), d_vec(1)] / d;
    pt    = p_mid + branch * h * perp;
end


function [fx, fy] = sweep_foot(Lv, N)
%SWEEP_FOOT  Compute foot trajectory for a given set of 12 link lengths.
%   Lv : 1x12 vector [L1..L12]
    P0 = [0, 0];
    P3 = [Lv(4), 0];
    thetas = linspace(0, 2*pi, N+1);
    thetas(end) = [];
    fx = []; fy = [];
    for i = 1:N
        th = thetas(i);
        P1 = P0 + Lv(1)*[cos(th), sin(th)];
        P2 = cci(P1, Lv(2),  P3,  Lv(3),  +1); if any(isnan(P2)), continue; end
        P5 = cci(P1, Lv(7),  P3,  Lv(8),  -1); if any(isnan(P5)), continue; end
        P4 = cci(P2, Lv(5),  P3,  Lv(6),  +1); if any(isnan(P4)), continue; end
        P6 = cci(P4, Lv(10), P5,  Lv(9),  +1); if any(isnan(P6)), continue; end
        PE = cci(P5, Lv(11), P6,  Lv(12), -1); if any(isnan(PE)), continue; end
        fx(end+1) = PE(1);
        fy(end+1) = PE(2);
    end
end
