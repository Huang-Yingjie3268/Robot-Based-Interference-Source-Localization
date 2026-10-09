% Team project plotting utility; inputs: results/reference; outputs: results/generated/figures.

clear; clc; close all;

here = fileparts(mfilename('fullpath'));
addpath(here);
S = paper_style();
D = q2_data(here);

%% ---------------- 版面（轴盒宽高比必须等于数据范围之比，否则等比会撑大坐标范围）----------------
INSET = 0.62;                       % main.tex 中 \includegraphics[width=0.62\textwidth]
figW  = S.Wstdio * INSET;           % cm
xL    = [-400 2100];
yL    = [-1350 1350];
lp = 1.62; rp = 0.42; bp = 2.36; tp = 0.28;          % 左右下上留白（cm）
boxW = figW - lp - rp;
boxH = boxW * diff(yL)/diff(xL);
figH = boxH + bp + tp;

f  = figure('Color','w','Units','centimeters','Position',[2 2 figW figH]);
ax = axes(f,'Units','centimeters','Position',[lp bp boxW boxH]);
hold(ax,'on');

% 坐标范围与等比必须在"放文字 / 画引导线"之前设好，
% 否则 text 的数据坐标会按自动坐标范围解释，annotation 的归一化坐标就会越界。
% 用属性设置等比例，而不是 axis(ax,'equal')：后者会把 XLimMode/YLimMode 重置为 auto，
% 于是"手动设好的坐标范围"会被自动范围（即所有数据的并集）顶掉。
ax.DataAspectRatio     = [1 1 1];
ax.DataAspectRatioMode = 'manual';
xlim(ax, xL);  ylim(ax, yL);
grid(ax, 'on');  box(ax, 'on');

%% ---------------- 1. 干扰源可能区域 Ω（半径 1800 m 的圆域，题面附录 3）----------------
th   = linspace(0, 2*pi, 1441);
hOmg = plot(ax, 1800*cos(th), 1800*sin(th), ':', ...
            'Color',S.guide, 'LineWidth',S.lw_thin);

%% ---------------- 2. 物理支撑集 supp_1（θ_1 ± 1°，r ∈ [5, 1500] m）----------------
% 真实张角，不做任何夸张：远端半宽 = 1500·sin(1°) = 26.18 m。
thw  = linspace(-1, 1, 241) * pi/180;
rOut = 1500;  rIn = 5;
hSup = fill(ax, [rOut*cos(thw), rIn*cos(fliplr(thw))], ...
                [rOut*sin(thw), rIn*sin(fliplr(thw))], S.c.region, ...
            'FaceAlpha',0.40, 'EdgeColor',S.c.region, 'LineWidth',S.lw_thin);

%% ---------------- 3. 对称轴 θ_1 方向 ----------------
hAx = plot(ax, [0 rOut], [0 0], '--', 'Color',S.guide, 'LineWidth',S.lw_thin);

%% ---------------- 4. 保证接收可行域 F_rec 的边界（margin = 0 的等值线）----------------
% ★ 耦合接收半径口径（方案B）：R*(g) = max(1000, ||S_1-g||)，F_rec 的边界是
%   margin = max_g(||S_2-g||-R*(g)) = 0 的等值线，**不是** maxdist = 1000 的等值线。
%   ★★ 特别注意（2026-09-12 更正）：margin 场的取值集必须是候选集 Γ（近弧 r=5 端点 +
%      分界弧 r=1000），**不能**沿用 P1 的顶点表：h(g)=||S_2-g||-R*(g) 是两个凸函数之差、
%      关于 g 非凸，"最大值必在凸多边形顶点取到"失效；而远弧 r=1500 上的点恒被同方位的
%      r=1000 点支配（r≥1000 时 ∂h/∂r = (r-(S_2-S_1)·u)/||S_2-g|| - 1 ≤ 0）。
%      用顶点表会把 F_rec 放大 8.8%（A_2 由 12187 格点虚增到 13365 格点）。
% 注意：contour/contourf 只给一个输出时返回的是等值线矩阵而不是句柄，
% 必须写 [~, h] = contour(...) 才能拿到图例要用的图形句柄。
[~, hFrec] = contour(ax, D.X, D.Y, D.S, [0 0], '-', ...
                     'Color',S.c.region, 'LineWidth',S.lw_heavy);

%% ---------------- 5. 检测基线 S_1 → S_2^* ----------------
hLine = plot(ax, [D.S1(1) D.S2up(1)], [D.S1(2) D.S2up(2)], '-.', ...
             'Color',S.c.region, 'LineWidth',S.lw_thin);

%% ---------------- 6. 各关键点 ----------------
hBase = plot(ax, D.base(1), D.base(2), '^', 'MarkerSize',S.ms+0.5, ...
              'MarkerFaceColor','w', 'MarkerEdgeColor',S.guide, 'LineWidth',1.0);
% 最坏分支（式(10) 口径下 R_2 直径的两端点）：朱红 = "直径"族
hWorst = plot(ax, [D.worstEnd1(1) D.worstEnd2(1)], [D.worstEnd1(2) D.worstEnd2(2)], ...
              'o', 'MarkerSize',S.ms, 'MarkerFaceColor',S.c.worst, ...
              'MarkerEdgeColor','w', 'LineWidth',0.8);
plot(ax, D.S1(1), D.S1(2), 's', 'MarkerSize',S.ms, ...
     'MarkerFaceColor',S.c.region, 'MarkerEdgeColor','w', 'LineWidth',0.8);
hS2 = plot(ax, [D.S2up(1) D.S2dn(1)], [D.S2up(2) D.S2dn(2)], 'p', ...
           'MarkerSize',S.ms+4, 'MarkerFaceColor',S.c.region, ...
           'MarkerEdgeColor','w', 'LineWidth',0.8);
% 自检：最坏分支端点必须落在 supp_1 内（否则图上的红点与几何自相矛盾）
for p = [D.worstEnd1; D.worstEnd2]'
    rr = hypot(p(1), p(2));
    aa = atan2d(p(2), p(1));
    assert(rr <= rOut + 1e-6 && abs(aa) <= 1 + 1e-6, ...
        '最坏分支端点 (%.3f, %.3f) 落在 supp_1 之外（r=%.3f m, a=%.3f°）', ...
        p(1), p(2), rr, aa);
end

%% ---------------- 7. 图内文字（全中文，只保留变量符号与单位）----------------
% 文字宽度按"坐标区宽 8.7 cm 对应 2500 m"折算：1 个汉字≈96 m、1 个半角字符≈48 m，
% 下面每处都按这个尺度控制右边界不超出轴右端 2100 m。
tOpt = {'FontName',S.zhFont, 'Color',S.ink, 'Interpreter','tex', 'FontSize',S.fs_note};

text(ax, D.S1(1)-60, D.S1(2), 'S_1', tOpt{:}, ...
     'HorizontalAlignment','right', 'VerticalAlignment','middle');

text(ax, D.S2up(1)+52, D.S2up(2)+40, ...
     sprintf('S_2^*=(%.0f, %.0f) m', D.S2up(1), D.S2up(2)), tOpt{:}, ...
     'HorizontalAlignment','left', 'VerticalAlignment','bottom');
text(ax, D.S2dn(1)+52, D.S2dn(2)-40, ...
     sprintf('S_2^*=(%.0f, %.0f) m', D.S2dn(1), D.S2dn(2)), tOpt{:}, ...
     'HorizontalAlignment','left', 'VerticalAlignment','top');

% 两个端点相距仅约 0.1 km，在 2500 m 宽的画幅里几乎重合，故用一条引线指到其中点。
wm = [mean([D.worstEnd1(1) D.worstEnd2(1)]), mean([D.worstEnd1(2) D.worstEnd2(2)])];
tWorst = text(ax, wm(1), wm(2)-165, '最坏分支直径端点', tOpt{:}, ...
              'HorizontalAlignment','center', 'VerticalAlignment','top', ...
              'Color',S.c.worst);

% supp_1 太窄，单独用引导线标注（文字放在 F_rec 右上方的空白带，不压任何几何元素）
tSup = text(ax, 1250, 120, '物理支撑集', tOpt{:}, ...
            'HorizontalAlignment','center', 'VerticalAlignment','bottom');
paper_arrow(ax, tSup,   [1250 25]);
paper_arrow(ax, tWorst, [wm(1), wm(2)-34], S.c.worst*0.55);

%% ---------------- 8. 图例（只承担"颜色/线型 ↔ 元素"的映射；已就地标注的不再重复）----------------
lg = legend(ax, [hOmg hFrec hAx hLine hS2 hBase hWorst], { ...
    '源可能区域 Ω', '可行域边界', '对称轴 θ_1', ...
    '检测基线', '最优点 S_2^*', '基线选点', '最坏分支端点'}, ...
    'Location','southoutside', 'Orientation','horizontal', 'NumColumns',4, ...
    'FontSize',S.fs_leg, 'Interpreter','tex');
lg.ItemTokenSize = [14 6];

%% ---------------- 9. 坐标区外观 ----------------
xlabel(ax, 'x / m', 'FontSize',S.fs_label, 'Color',S.ink);
ylabel(ax, 'y / m', 'FontSize',S.fs_label, 'Color',S.ink);
ax.FontSize           = S.fs_axes;
ax.LineWidth          = S.lw_frame;
ax.XColor             = S.ink;
ax.YColor             = S.ink;
ax.XTickLabelRotation = 0;          % 刻度标签强制水平

% 轴盒宽高比必须与数据范围之比一致，否则等比会白白撑大坐标范围、制造大面积留白。
% ★ 复位必须放在最后：legend('southoutside') 与 colorbar 都会改写坐标区位置，
%   位置一变、等比就会为了凑比例而撑大坐标范围 —— 这正是原图"范围被撑大、
%   数据四周留白"的成因。故要等所有会动版式的元素都画完，再重设坐标范围 + 坐标区位置。
% 先复位位置、再设范围，交替两轮让它收敛：
% 若顺序反了，等比会按"被 legend 改过的旧位置"去撑大坐标范围，
% 之后即使把位置改回来，被撑大的范围也不会自动还原。
for k = 1:2
    ax.Units = 'centimeters';  ax.Position = [lp bp boxW boxH];
    ax.DataAspectRatio     = [1 1 1];
    ax.DataAspectRatioMode = 'manual';
    xlim(ax, xL);  ylim(ax, yL);
    drawnow;
end

ratioNow = ax.Position(3)/ax.Position(4);
assert(abs(ratioNow - diff(xL)/diff(yL)) < 5e-3, ...
    '轴盒宽高比 %.6f 与数据范围之比 %.6f 不一致（等比会因此留白）', ...
    ratioNow, diff(xL)/diff(yL));
assert(isequal(ax.DataAspectRatioMode,'manual') && ...
       max(abs(ax.DataAspectRatio - [1 1 1])) < 1e-9, '等比未生效');
assert(max(abs([diff(ax.XLim) diff(ax.YLim)] - [diff(xL) diff(yL)])) < 1e-9, ...
    '坐标范围被改动：xlim=%s ylim=%s，违反"不得修改坐标范围"', ...
    mat2str(ax.XLim), mat2str(ax.YLim));

paper_export(f, 'fig_geometry');
close(f);

fprintf('图1 完成：S_2^*=(%.0f, %.0f) m，检测基线长 %.4f m，基线选点长 %.4f m；', ...
        D.S2c(1), abs(D.S2c(2)), D.optLen, D.baseLen);
fprintf('F_rec 左端点 %.4f m，最坏分支端点 (%.2f, %.2f)/(%.2f, %.2f) m\n', ...
        D.frecLeft, D.worstEnd1(1), D.worstEnd1(2), D.worstEnd2(1), D.worstEnd2(2));
