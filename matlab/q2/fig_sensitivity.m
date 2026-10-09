% Team project plotting utility; inputs: results/reference; outputs: results/generated/figures.

clear; clc; close all;

here = fileparts(mfilename('fullpath'));
addpath(here);
S = paper_style();
D = q2_data(here);

%% ---------------- 版面 ----------------
INSET = 0.88;                       % main.tex 中 \includegraphics[width=0.88\textwidth]
figW  = S.Wstdio * INSET;           % cm
lp = 1.30; gap = 1.55; rp = 0.30;
boxW = (figW - lp - gap - rp)/2;
boxH = 5.70;
bp = 1.05; tp = 0.28;
figH = boxH + bp + tp;

% 面板轴范围（与文末断言、与 verify_figs.py 的 rtok 三处必须一致）
XL1 = [0.8 1.2];   YL1 = [85 135];
XL2 = [1000 1500]; YL2 = [70 120];

f   = figure('Color','w','Units','centimeters','Position',[2 2 figW figH]);
ax1 = axes(f,'Units','centimeters','Position',[lp, bp, boxW, boxH]);
ax2 = axes(f,'Units','centimeters','Position',[lp+boxW+gap, bp, boxW, boxH]);

tOpt = {'FontName',S.zhFont, 'Color',S.ink, 'Interpreter','tex'};

%% ================= (a) 误差界 δ 的影响 =================
plot(ax1, D.dlt, D.dltJ, '-', 'Color',S.c.diam, 'LineWidth',S.lw);
hold(ax1,'on');
plot(ax1, D.dlt, D.dltJ, 'o', 'MarkerSize',S.ms-0.5, ...
     'MarkerFaceColor',S.c.diam, 'MarkerEdgeColor','w', 'LineWidth',0.9);

% 题面取值 δ = 1° 处用灰色参考线标出（该档同时是正文灵敏度表的主答案档）
xline(ax1, 1.0, '--', 'Color',S.guide, 'LineWidth',S.lw_thin);
% ★ 标注 y 一律由轴范围推出（放在上界下方 marginS 处），不写死绝对 y。
marginS = 0.055*diff(YL1);          % ≈2.8 m
text(ax1, 1.006, YL1(2)-0.004*diff(YL1), '题面取值', tOpt{:}, ...
     'FontSize',S.fs_note-1, 'Color',S.guide, ...
     'HorizontalAlignment','left', 'VerticalAlignment','top');

% 端点数值（与论文灵敏度表逐位一致）
% 数值标注放在"标记内侧"的空白处：贴轴底会被 x 刻度标签压住、贴轴顶会被轴框裁掉（实测）
text(ax1, XL1(1)+0.006, YL1(1)+0.35*marginS, sprintf('%.2f', D.dltJ(1)), tOpt{:}, ...
     'FontSize',S.fs_note-1, 'HorizontalAlignment','left', 'VerticalAlignment','bottom');
text(ax1, XL1(2)-0.010, D.dltJ(end)+0.9, sprintf('%.2f', D.dltJ(end)), tOpt{:}, ...
     'FontSize',S.fs_note-1, 'HorizontalAlignment','right', 'VerticalAlignment','bottom');

xlim(ax1, XL1);  ylim(ax1, YL1);
xticks(ax1, 0.8:0.1:1.2);
ax1.YTick = YL1(1):10:YL1(2);
xlabel(ax1, '示向误差上界 δ / (°)', 'FontSize',S.fs_label, 'Color',S.ink);
ylabel(ax1, '最坏后验直径 J / m',   'FontSize',S.fs_label, 'Color',S.ink);

%% ================= (b) 接收半径假设 R 的影响 =================
plot(ax2, D.Rr, D.RrJ, '-', 'Color',S.c.diam, 'LineWidth',S.lw);
hold(ax2,'on');
plot(ax2, D.Rr, D.RrJ, 's', 'MarkerSize',S.ms-0.5, ...
     'MarkerFaceColor',S.c.diam, 'MarkerEdgeColor','w', 'LineWidth',0.9);
% 正式取值（最坏保证）R = 1000 m：论文 §5.4 明示"正式答案取 R=1000 m"
plot(ax2, D.Rr(1), D.RrJ(1), 'o', 'MarkerSize',S.ms+4.5, ...
     'MarkerFaceColor','none', 'MarkerEdgeColor',S.guide, 'LineWidth',1.0);
% ★ 文字必须落在坐标范围内：text 不受坐标区裁剪，放到轴外会被导出画布直接切掉
%   （实测踩过：放在 ylim 之上，"正"字被切掉，只剩"式取值"）。
%   故这里改成"相对端点、相对轴跨"的偏移量。
text(ax2, D.Rr(1)+0.032*diff(XL2), D.RrJ(1)-0.052*diff(YL2), '正式取值', tOpt{:}, ...
     'FontSize',S.fs_note-1, 'Color',S.guide, ...
     'HorizontalAlignment','left', 'VerticalAlignment','top');

text(ax2, D.Rr(1)+0.032*diff(XL2), D.RrJ(1)+0.050*diff(YL2), ...
     sprintf('%.2f', D.RrJ(1)), tOpt{:}, ...
     'FontSize',S.fs_note-1, 'HorizontalAlignment','left', 'VerticalAlignment','bottom');
text(ax2, D.Rr(5)-0.052*diff(XL2), D.RrJ(5)+0.045*diff(YL2), ...
     sprintf('%.2f', D.RrJ(5)), tOpt{:}, ...
     'FontSize',S.fs_note-1, 'HorizontalAlignment','right', 'VerticalAlignment','bottom');

xlim(ax2, XL2);  ylim(ax2, YL2);
xticks(ax2, 1000:100:1500);
ax2.YTick = YL2(1):10:YL2(2);
xlabel(ax2, '接收半径假设 R / m', 'FontSize',S.fs_label, 'Color',S.ink);
ylabel(ax2, '最坏后验直径 J / m', 'FontSize',S.fs_label, 'Color',S.ink);

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

% 坐标范围自检：四个范围显式钉死，必须逐位保持（口径B 下已按新数据重设，见文件头）。
assert(max(abs([ax1.XLim ax1.YLim] - [XL1 YL1])) < 1e-9, ...
    '面板(a) 坐标范围被改动：%s', mat2str([ax1.XLim ax1.YLim]));
assert(max(abs([ax2.XLim ax2.YLim] - [XL2 YL2])) < 1e-9, ...
    '面板(b) 坐标范围被改动：%s', mat2str([ax2.XLim ax2.YLim]));
% 数据必须完全落在轴内（这是本轮改范围的唯一理由，把它变成断言而不是注释）
assert(min(D.dltJ) >= YL1(1) && max(D.dltJ) <= YL1(2), ...
    '面板(a) 数据越出轴范围：J*∈[%.4f, %.4f]，轴=[%g %g]', ...
    min(D.dltJ), max(D.dltJ), YL1(1), YL1(2));
assert(min(D.RrJ) >= YL2(1) && max(D.RrJ) <= YL2(2), ...
    '面板(b) 数据越出轴范围：J*∈[%.4f, %.4f]，轴=[%g %g]', ...
    min(D.RrJ), max(D.RrJ), YL2(1), YL2(2));

paper_export(f, 'fig_sensitivity');
close(f);

fprintf('图3 完成：(a) δ∈[%.1f, %.1f] → J*∈[%.4f, %.4f] m；(b) R∈[%.0f, %.0f] → J*∈[%.4f, %.4f] m\n', ...
    D.dlt(1), D.dlt(end), D.dltJ(1), D.dltJ(end), D.Rr(1), D.Rr(end), D.RrJ(1), D.RrJ(end));
