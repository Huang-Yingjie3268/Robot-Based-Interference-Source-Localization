function S = paper_style()
% Team project plotting utility; inputs: results/reference; outputs: results/generated/figures.

cand = {'SimSun','宋体','SimHei','黑体','Microsoft YaHei','微软雅黑', ...
        'PingFang SC','Noto Serif CJK SC'};
avail = listfonts;
zhFont = cand{1};
for k = 1:numel(cand)
    if any(strcmpi(avail, cand{k}))
        zhFont = cand{k};
        break;
    end
end

%% 2. 全局默认（作用于之后新建的所有图窗、坐标区、线条、文本）
set(groot,'defaultFigureColor','w');
set(groot,'defaultAxesFontName',zhFont);
set(groot,'defaultTextFontName',zhFont);
set(groot,'defaultLegendFontName',zhFont);
set(groot,'defaultAxesFontSize',10.5);
set(groot,'defaultTextFontSize',10.5);
set(groot,'defaultLegendFontSize',9);
set(groot,'defaultAxesLineWidth',0.75);      % 坐标框线
set(groot,'defaultLineLineWidth',1.8);       % 数据线
set(groot,'defaultAxesBox','on');
set(groot,'defaultAxesXGrid','on');
set(groot,'defaultAxesYGrid','on');
set(groot,'defaultAxesGridLineStyle','-');
set(groot,'defaultAxesGridAlpha',0.15);      % 网格只作背景参考，不抢数据
set(groot,'defaultAxesTickDir','out');
set(groot,'defaultAxesTickLength',[0.012 0.020]);
set(groot,'defaultAxesLayer','bottom');      % 网格置于数据之下
set(groot,'defaultAxesXMinorTick','off');
set(groot,'defaultAxesYMinorTick','off');

% 固定随机种子（全文一致，便于复现；本套图无随机量，仅保持规范统一）
rng(2026,'twister');

%% 3. 国赛常用配色（色盲友好色序，RGB 0-1；顺序与《MATLAB 高质量绘图手册》1.2 节一致）
palette = [   0 114 178;    % 1 蓝
            230 159   0;    % 2 橙
              0 158 115;    % 3 绿
            213  94   0;    % 4 朱红
            204 121 167;    % 5 紫
             86 180 233;    % 6 天蓝
            240 228  66]/255;   % 7 黄

%% 4. 语义配色表（全篇引用此表，禁止在脚本里另写 RGB）
S.font    = zhFont;
S.palette = palette;
S.c = struct( ...
    'region', palette(1,:), ...   % 定位区域 / 定位域边界 —— 蓝
    'circle', palette(2,:), ...   % 最小覆盖圆、最小覆盖圆半径 R_min —— 橙
    'area',   palette(3,:), ...   % 面积（辅助量，细线弱化）—— 绿
    'diam',   palette(4,:), ...   % 直径 D_k —— 朱红
    'aux1',   palette(5,:), ...   % 备用量（问题二/三/四可用）
    'aux2',   palette(6,:), ...
    'aux3',   palette(7,:));
S.ink     = [0.15 0.15 0.15];     % 文字/刻度主色（近黑，避免纯黑发死）
S.guide   = [0.50 0.50 0.50];     % 参考线（如题面取值）
S.zhFont  = zhFont;

% 让默认色序与国赛色序一致，避免手工取色
set(groot,'defaultAxesColorOrder',palette);
end
