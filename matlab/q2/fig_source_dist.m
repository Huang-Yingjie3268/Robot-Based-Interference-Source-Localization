% Team project plotting utility; inputs: results/reference; outputs: results/generated/figures.

clear; clc; close all;

here = fileparts(mfilename('fullpath'));
addpath(here);
S = paper_style();
D = q2_data(here);

%% ---------------- 版面 ----------------
INSET = 0.88;                       % main.tex 中 \includegraphics[width=0.88\textwidth]
figW  = S.Wstdio * INSET;           % cm
lp = 1.42; gap = 1.55; rp = 0.30;
boxW = (figW - lp - gap - rp)/2;
boxH = 5.70;
bp = 1.05; tp = 0.28;
figH = boxH + bp + tp;

f   = figure('Color','w','Units','centimeters','Position',[2 2 figW figH]);
ax1 = axes(f,'Units','centimeters','Position',[lp, bp, boxW, boxH]);
ax2 = axes(f,'Units','centimeters','Position',[lp+boxW+gap, bp, boxW, boxH]);

tOpt = {'FontName',S.zhFont, 'Color',S.ink, 'Interpreter','tex'};

%% ================= (a) 交会直径 D_ideal 随源距的变化 =================
plot(ax1, D.r1, D.Dideal, '-', 'Color',S.c.diam, 'LineWidth',S.lw);
hold(ax1,'on');

% 谷底：内段最小 —— 与正文"r_1≈800 m（γ≈90°）处取谷底"一致（端点 r_1=5 处更小，
% 但那是源距下界的端点值，不是内段谷底，论文表 6 亦按此口径列出）
plot(ax1, D.r1(D.vly), D.Dideal(D.vly), 'v', 'MarkerSize',S.ms+2, ...
     'MarkerFaceColor','w', 'MarkerEdgeColor',S.c.diam, 'LineWidth',1.2);
% 全局最大：取在源距的右端邻域（口径B 下实测 r_1=1445 m，见 data/q2_theory_checks.csv
% 的 c3_global_max 行；标记位置与标注文字都由 CSV 决定，不写死横坐标）
plot(ax1, D.r1(D.pk), D.Dideal(D.pk), '^', 'MarkerSize',S.ms+2, ...
     'MarkerFaceColor',S.c.diam, 'MarkerEdgeColor','w', 'LineWidth',0.9);

% 避让标注：谷底放在曲线下方的空白带，全局最大放在曲线拱顶之上
text(ax1, D.r1(D.vly)+95, 31.6, ...
     sprintf('谷底 (%.0f, %.2f)', D.r1(D.vly), D.Dideal(D.vly)), tOpt{:}, ...
     'FontSize',S.fs_note-1, 'HorizontalAlignment','left', 'VerticalAlignment','bottom');
text(ax1, D.r1(D.pk), D.Dideal(D.pk)+1.6, ...
     sprintf('全局最大 (%.0f, %.2f)', D.r1(D.pk), D.Dideal(D.pk)), tOpt{:}, ...
     'FontSize',S.fs_note-1, 'HorizontalAlignment','right', 'VerticalAlignment','bottom');

xlim(ax1, [0 1550]);  ylim(ax1, [30 130]);
ax1.YTick = 30:10:130;               % 与原图刻度一致
xlabel(ax1, '源距 r_1 / m',             'FontSize',S.fs_label, 'Color',S.ink);
ylabel(ax1, '交会直径 D_{ideal} / m',   'FontSize',S.fs_label, 'Color',S.ink);

%% ================= (b) 交会角 γ 随源距的变化 =================
plot(ax2, D.r1, D.gamma, '-', 'Color',S.c.aux1, 'LineWidth',S.lw);
hold(ax2,'on');
% γ = 90° 参考线（谷底对应点）
yline(ax2, 90, '--', 'Color',S.guide, 'LineWidth',S.lw_thin);
plot(ax2, D.r1(D.vly), D.gamma(D.vly), 'o', 'MarkerSize',S.ms, ...
     'MarkerFaceColor',S.c.aux1, 'MarkerEdgeColor','w', 'LineWidth',0.9);
text(ax2, 60, 93, 'γ = 90°（谷底对应）', tOpt{:}, ...
     'FontSize',S.fs_note-1, 'Color',S.guide, 'HorizontalAlignment','left', 'VerticalAlignment','bottom');
text(ax2, D.r1(D.vly)+40, D.gamma(D.vly)-5, sprintf('γ = %.2f°', D.gamma(D.vly)), tOpt{:}, ...
     'FontSize',S.fs_note-1, 'Color',S.c.aux1, 'HorizontalAlignment','left', 'VerticalAlignment','top');

xlim(ax2, [0 1550]);  ylim(ax2, [30 150]);
ax2.YTick = 30:20:150;               % 口径B 下按新数据重设（见文件头）
xlabel(ax2, '源距 r_1 / m',     'FontSize',S.fs_label, 'Color',S.ink);
ylabel(ax2, '交会角 γ / (°)',   'FontSize',S.fs_label, 'Color',S.ink);

%% ================= 两幅统一外观 =================
for ax = [ax1 ax2]
    grid(ax,'on');  box(ax,'on');
    ax.FontSize           = S.fs_axes;
    ax.LineWidth          = S.lw_frame;
    ax.XColor             = S.ink;
    ax.YColor             = S.ink;
    ax.XTickLabelRotation = 0;
end
text(ax1, 0.035, 0.955, '（a）', 'Units','normalized', tOpt{:}, ...
     'FontSize',S.fs_note, 'VerticalAlignment','top');
text(ax2, 0.035, 0.955, '（b）', 'Units','normalized', tOpt{:}, ...
     'FontSize',S.fs_note, 'VerticalAlignment','top');

drawnow;
assert(abs(ax1.Position(3) - ax2.Position(3)) < 1e-9, '两子图宽度不一致');
assert(abs(ax1.Position(4) - ax2.Position(4)) < 1e-9, '两子图高度不一致');

% 坐标范围自检：四个范围显式钉死，必须逐位保持（口径B 下 (b) 的 y 已按数据重设，见文件头）。
% x 轴 [0 1550] 与原图一致。
assert(max(abs([ax1.XLim ax1.YLim] - [0 1550 30 130])) < 1e-9, ...
    '面板(a) 坐标范围被改动：%s', mat2str([ax1.XLim ax1.YLim]));
assert(max(abs([ax2.XLim ax2.YLim] - [0 1550 30 150])) < 1e-9, ...
    '面板(b) 坐标范围被改动：%s', mat2str([ax2.XLim ax2.YLim]));
% 数据必须完全落在轴内（这是本轮改 (b) 范围的唯一理由，把它变成断言而不是注释）
assert(min(D.gamma) >= 30 && max(D.gamma) <= 150, ...
    '面板(b) 数据越出轴范围：γ∈[%.4f, %.4f]°', min(D.gamma), max(D.gamma));
assert(min(D.Dideal) >= 30 && max(D.Dideal) <= 130, ...
    '面板(a) 数据越出轴范围：D_ideal∈[%.4f, %.4f] m', min(D.Dideal), max(D.Dideal));

paper_export(f, 'fig_source_dist');
close(f);

fprintf('图4 完成：谷底 r_1=%.0f m → D_ideal=%.6f m、γ=%.6f°；全局最大 r_1=%.0f m → D_ideal=%.6f m\n', ...
    D.r1(D.vly), D.Dideal(D.vly), D.gamma(D.vly), D.r1(D.pk), D.Dideal(D.pk));
