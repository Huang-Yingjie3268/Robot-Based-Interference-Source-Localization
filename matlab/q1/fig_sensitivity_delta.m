% Team project plotting utility; inputs: results/reference; outputs: results/generated/figures.

clear; clc; close all;
S = paper_style();

here    = fileparts(mfilename('fullpath'));
root    = fileparts(fileparts(here));
dataDir = fullfile(root,'results','reference','q1');

% 按列位置取数（不依赖表头名，规避 BOM/编码差异）
Tc = readcell(fullfile(dataDir,'sensitivity_delta.csv'),'Delimiter',',');
assert(size(Tc,2)==4 && size(Tc,1)>=3, 'sensitivity_delta.csv 维度异常');
A        = cell2mat(Tc(2:end,1:4));
deltaDeg = A(:,1);      % 示向误差上界 δ / (°)
diamM    = A(:,2);      % 直径 D_k / m
areaM2   = A(:,3);      % 面积 / m^2
rminM    = A(:,4);      % 最小覆盖圆半径 R_min / m
assert(all(isfinite(A(:))),'sensitivity_delta.csv 含非有限值');

%% 绘图
f  = figure('Visible','off','Units','centimeters','Position',[2 2 15.2 9.8]);
ax = axes(f); hold(ax,'on');

% ① 题面给定取值 δ=1° 的参考线（仅作阅读指引，不进图例）
d0 = 1.0;
plot(ax,[d0 d0],[10 45],'--','Color',S.guide,'LineWidth',1.0,'HandleVisibility','off');
text(ax, d0+0.008, 43.2, '题面取值 \delta = 1^\circ', ...
     'Color',S.guide,'FontSize',8.5,'HorizontalAlignment','left','VerticalAlignment','top');

% ② 左纵轴：长度量（直径 D_k、最小覆盖圆半径 R_min）
yyaxis(ax,'left');
p1 = plot(ax, deltaDeg, diamM, '-o', 'Color',S.c.diam, 'LineWidth',1.8, ...
          'MarkerSize',6,'MarkerFaceColor',S.c.diam,'DisplayName','直径 D_k');
p2 = plot(ax, deltaDeg, rminM, '--s','Color',S.c.circle,'LineWidth',1.8, ...
          'MarkerSize',6,'MarkerFaceColor','w','DisplayName','最小覆盖圆半径 R_{\rm min}');
ylabel(ax,'长度 / m');
ylim(ax,[10 45]); yticks(ax,10:5:45);

% ③ 右纵轴：面积（辅助量，细线弱化）
yyaxis(ax,'right');
p3 = plot(ax, deltaDeg, areaM2, '-^','Color',S.c.area,'LineWidth',1.3, ...
          'MarkerSize',6,'MarkerFaceColor','w','DisplayName','面积');
ylabel(ax,'面积 / m^2');
ylim(ax,[100 800]); yticks(ax,100:100:800);

% ④ 横轴与整体观感
xlim(ax,[0.8 1.2]); xticks(ax,0.8:0.05:1.2);
xlabel(ax,'示向误差上界 \delta / (^\circ)');

ax.XAxis.Color = S.ink;
ax.YAxis(1).Color = S.ink;                 % 两轴同色，颜色只用于区分数据系列
ax.YAxis(2).Color = S.ink;
ax.GridColor = S.ink;
ax.GridAlpha = 0.15;                       % 网格只作背景参考（两轴 7 等分，网格线重合）

lg = legend(ax,[p1 p2 p3],'Location','northwest','Box','off','FontSize',9);
lg.ItemTokenSize = [16 8];

%% 导出
paper_export(f,'sensitivity_delta');
close(f);
