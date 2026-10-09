% Team project plotting utility; inputs: results/reference; outputs: results/generated/figures.

clear; clc; close all;

here = fileparts(mfilename('fullpath'));
addpath(here);
S = paper_style();
D = q2_data(here);

%% ---------------- 版面 ----------------
INSET = 0.58;                       % main.tex 中 \includegraphics[width=0.58\textwidth]
figW  = S.Wstdio * INSET;           % cm
xL    = [-50 1050];
yL    = [-1000 1000];
pbRat = diff(xL) / diff(yL);        % 轴盒宽高比 = 数据宽高比 ⇒ 等比显示（1:2.4444）
lp = 1.46; cbGap = 0.22; cbW = 0.40; rp = 2.06;   % 右侧依次放色标刻度（≥1000）与色标标签
bp = 2.20; tp = 0.28;                             % 底部：xlabel + 刻度 + 单行图例
boxW = figW - lp - cbGap - cbW - rp;
boxH = boxW / pbRat;
figH = boxH + bp + tp;

f  = figure('Color','w','Units','centimeters','Position',[2 2 figW figH]);
ax = axes(f,'Units','centimeters','Position',[lp bp boxW boxH]);
hold(ax,'on');

% 坐标范围与纵横比要在"放文字 / 画引线"之前设好：
% paper_arrow 用 figure 归一化坐标，依赖当时的 xlim/ylim 才能算准；
% 且不能用 axis(ax,'equal')（它会把 XLimMode/YLimMode 重置为 auto，顶掉手动范围）。
% 等比的正确写法是 DataAspectRatio=[1 1 1] 且置 manual，同时让 PlotBoxAspectRatio
% 回到 auto，由 MATLAB 按数据宽高比定轴盒；轴盒位置的比例已按同一比值给定，
% 故不会在轴盒内留下多余边距（见文末版式自检）。
xlim(ax, xL);  ylim(ax, yL);
ax.DataAspectRatio     = [1 1 1];
ax.DataAspectRatioMode = 'manual';
ax.PlotBoxAspectRatioMode = 'auto';

%% ---------------- 1. J 的有定义区：截断到显示量程上限 ----------------
% 论文口径是 ">1000 m 截断"（图注原话）。这里把截断值画成色标最深一档，
% 而不是留成白色空洞——白色会被读成"无数据/域外"，与本图要表达的"最坏区"相反。
% ★ 色标下界必须 ≤ F_rec 内的最小 J（口径B 下 F_rec 内的最小 J = J*，约 111 m）：
%   早期版本把下界写死成 130 m，那是口径A（J* = 134.10 m）下的取值；
%   口径B 下 J* 降到 111 m 左右，若不改就会把最优区整片留成白色空洞。
%   故下界由数据推出，上界保持"1000 m 以上单列一档"的论文口径。
jmin = min(D.J(D.inF));
lv0  = 10*floor(jmin/10);
lv = [lv0:20:990, 1000, 1001];                    % 末档 [1000,1001] 即"超出量程"
assert(lv(1) <= jmin, '色标下界 %.1f m 高于可行域内最小 J %.4f m', lv(1), jmin);
assert(jmin - lv(1) < 20, '色标下界 %.1f m 与最小 J %.4f m 相差过大，白洞会很明显', lv(1), jmin);
assert(lv(end) == 1001 && lv(end-1) == 1000, '">1000 截断"的末两档被改动');
Jp = min(D.J, 1001);
Jp(~D.inF) = NaN;                                 % 可行域之外不画
[~, hJ] = contourf(ax, D.X, D.Y, Jp, lv, 'LineColor','none');
caxis(ax, [lv(1) lv(end)]);
colormap(ax, paper_cmap('red', numel(lv)-1));

%% ---------------- 2. 候选区域三层结构的后两层 ----------------
[~, hQ2] = contour(ax, D.X, D.Y, D.J, [D.tau D.tau], '--', ...
                   'Color',S.c.area, 'LineWidth',S.lw_thin);
selA = ~isnan(D.J) & D.inF & D.J <= D.thrNear;
hAeta = plot(ax, D.X(selA), D.Y(selA), 'o', 'MarkerSize',7.0, ...
             'MarkerFaceColor',S.c.area, 'MarkerEdgeColor','w', 'LineWidth',0.8);
assert(nnz(selA) == D.nAeta, 'A_eta 格点数 %d ≠ 论文表 5 的 %d', nnz(selA), D.nAeta);

%% ---------------- 3. 可行域边界（画在填充之上）----------------
[~, hFrec] = contour(ax, D.X, D.Y, D.S, [0 0], '-', ...
                     'Color',S.c.region, 'LineWidth',S.lw_heavy);

%% ---------------- 4. 两个镜像最优点 ----------------
hS2 = plot(ax, D.S2up(1), D.S2up(2), 'p', 'MarkerSize',S.ms+4, ...
           'MarkerFaceColor',S.c.region, 'MarkerEdgeColor','w', 'LineWidth',0.8);
plot(ax, D.S2dn(1), D.S2dn(2), 'p', 'MarkerSize',S.ms+4, ...
     'MarkerFaceColor',S.c.region, 'MarkerEdgeColor','w', 'LineWidth',0.8);

tOpt = {'FontName',S.zhFont, 'Color',S.ink, 'Interpreter','tex'};
% 两个星标落在 F_rec 的上下尖顶，周围没有放得下整串坐标的空白，
% 故把坐标写到可行域**外侧的空白带**再用引线指回星标。
% ★ 方案B（耦合口径）后 F_rec 的左端由 500.11 m 放宽到 0，可行域扩展到 x∈[0,1005]，
%   原来"左上角空白"的位置（x≈496）已被填色区覆盖，故标注必须移到更外侧：
%   F_rec 的 |y| 上界是 938.147 m（出现在 x=351 m），取 y = ±960 m 即在域外留白带上。
tUp = text(ax, 30, 960, ...
     sprintf('S_2^*=(%.0f, %.0f) m', D.S2up(1), D.S2up(2)), tOpt{:}, ...
     'FontSize',S.fs_note-0.5, 'HorizontalAlignment','left', 'VerticalAlignment','middle');
tDn = text(ax, 30, -960, ...
     sprintf('S_2^*=(%.0f, %.0f) m', D.S2dn(1), D.S2dn(2)), tOpt{:}, ...
     'FontSize',S.fs_note-0.5, 'HorizontalAlignment','left', 'VerticalAlignment','middle');
paper_arrow(ax, tUp, [D.S2up(1)-14, D.S2up(2)+10]);
paper_arrow(ax, tDn, [D.S2dn(1)-14, D.S2dn(2)-10]);

%% ---------------- 5. 图例（无"无界分支"项：模型(10) 下不存在）----------------
lg = legend(ax, [hFrec hQ2 hAeta], { ...
    '可行域边界', '优质域边界', '近优域格点'}, ...
    'Location','southoutside', 'Orientation','horizontal', 'NumColumns',2, ...
    'FontSize',S.fs_leg, 'Interpreter','tex');
lg.ItemTokenSize = [14 6];

%% ---------------- 6. 坐标区外观 ----------------
grid(ax, 'on');  box(ax, 'on');
xticks(ax, 0:200:1000);             % ★ 扩窗后 x 跨度变成 [0,1005]，刻度随范围铺满；
                                    %   原图窗口只有 [480,1020]，故原刻度是 500:100:1000
xlabel(ax, 'x / m', 'FontSize',S.fs_label, 'Color',S.ink);
ylabel(ax, 'y / m', 'FontSize',S.fs_label, 'Color',S.ink);
ax.FontSize           = S.fs_axes;
ax.LineWidth          = S.lw_frame;
ax.XColor             = S.ink;
ax.YColor             = S.ink;
ax.XTickLabelRotation = 0;

%% ---------------- 7. 色标（刻度与原图逐项一致；截断口径写进轴标签）----------------
% ★ 刻度必须是原图的 200:100:1000 共 9 档，一档都不能少（"不改刻度"的红线）。
%   原图把 ">1000 截断" 写进色标轴标签（J (m), >1000 截断），此处照此保留、
%   只把字体语言与单位写法规范化，不把信息挪去别处。
% 放在最后创建：colorbar 会改写坐标区位置，所以先建色标、再统一复位版式。
cb = colorbar(ax);
cb.Ticks          = 200:100:1000;
cb.Label.String   = 'J / m（>1000 截断）';
cb.Label.FontName = S.zhFont;
cb.Label.FontSize = S.fs_label;
cb.FontSize  = S.fs_axes - 1;
cb.LineWidth = S.lw_frame;
cb.Color     = S.ink;

%% ---------------- 8. 版式复位 ----------------
% ★ 必须在 colorbar / legend 都建好之后，再重设坐标范围与坐标区位置：
%   colorbar 会改写坐标区位置，位置一变、等比就会引出多余边距。
% 先复位位置、再设范围，交替两轮让它收敛（colorbar 同理会改写坐标区位置）
for k = 1:2
    ax.Units = 'centimeters';  ax.Position = [lp bp boxW boxH];
    cb.Units = 'centimeters';  cb.Position = [lp+boxW+cbGap, bp, cbW, boxH];
    ax.DataAspectRatio     = [1 1 1];
    ax.DataAspectRatioMode = 'manual';
    ax.PlotBoxAspectRatioMode = 'auto';
    xlim(ax, xL);  ylim(ax, yL);
    drawnow;
end

% 版式自检：轴盒位置比例必须等于数据宽高比（否则轴盒内会出现多余边距）
ratioNow = ax.Position(3)/ax.Position(4);
assert(abs(ratioNow - pbRat) < 5e-3, ...
    '轴盒宽高比 %.6f 与等比设定 %.6f 不一致', ratioNow, pbRat);
assert(max(abs([diff(ax.XLim) diff(ax.YLim)] - [diff(xL) diff(yL)])) < 1e-9, ...
    '坐标范围被改动：xlim=%s ylim=%s，违反"不得修改坐标范围"', ...
    mat2str(ax.XLim), mat2str(ax.YLim));

paper_export(f, 'fig_J');
close(f);

% 不再打印"J 未定义格点数"：口径B（式(10)）下 R_2 = P_1∩C 对任意 θ_2∈Θ(S_2) 恒非空，
% 该计数恒为 0，打印它只会让人以为还存在"无界分支"（q2_data 已就此设断言）。
fprintf('图2 完成：优质域阈值 tau=%.6f m，近优域阈值 %.6f m，近优格点 %d 个\n', ...
        D.tau, D.thrNear, nnz(selA));
