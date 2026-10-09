% Team project plotting utility; inputs: results/reference; outputs: results/generated/figures.

clear; clc; close all;
S = paper_style();                        % 统一风格 + 国赛配色

here    = fileparts(mfilename('fullpath'));            % matlab/q1
root    = fileparts(fileparts(here));                  % 工程根目录
dataDir = fullfile(root,'results','reference','q1');

V = readcell(fullfile(dataDir,'validation_vertices.csv'),'Delimiter',',');
M = readcell(fullfile(dataDir,'validation_summary.csv' ),'Delimiter',',');
assert(size(V,2)==3 && size(V,1)>=4, 'validation_vertices.csv 维度异常');
assert(size(M,2)==3 && size(M,1)>=6, 'validation_summary.csv 维度异常');

P      = cell2mat(V(2:end,2:3));          % 顶点坐标（唯一几何输入）
vnames = string(V(2:end,1));              % 顶点名（V1/V2/V3…）
metric = string(M(2:end,1));
value  = string(M(2:end,2));
D  = str2double(value(metric=="diameter"));
Rm = str2double(value(metric=="minimum_enclosing_radius"));
assert(isscalar(D) && isscalar(Rm) && isfinite(D) && isfinite(Rm),'summary 数值异常');

% —— 以下两项为纯几何导出量，不是新增数据 ——
% 最小覆盖圆圆心：正三角形的外心即形心
c = mean(P,1);
% 直径端点：最远顶点对
best = -inf; pair = [1 2];
for i = 1:size(P,1)
    for j = i+1:size(P,1)
        d = norm(P(i,:)-P(j,:));
        if d > best, best = d; pair = [i j]; end
    end
end
ang = linspace(0,2*pi,720);

%% 绘图
f  = figure('Visible','off','Units','centimeters','Position',[2 2 13.6 12.0]);
ax = axes(f); hold(ax,'on');

% ① 定位区域（弱填充，只作底衬）
fill(ax, P(:,1), P(:,2), S.c.region, ...
     'FaceAlpha',0.12,'EdgeColor','none','DisplayName','定位区域');

% ② 定位域边界
plot(ax, [P(:,1);P(1,1)], [P(:,2);P(1,2)], '-', ...
     'Color',S.c.region,'LineWidth',1.8,'DisplayName','定位域边界');

% ③ 直径 D_k（关键指标，用强调色加粗）
plot(ax, P(pair,1), P(pair,2), '-', ...
     'Color',S.c.diam,'LineWidth',2.6,'DisplayName','直径 D_k');
plot(ax, P(pair,1), P(pair,2), 'o', 'Color',S.c.diam, ...
     'MarkerFaceColor',S.c.diam,'MarkerSize',4.5,'HandleVisibility','off');

% ④ 最小覆盖圆（虚线，与图2的 R_min 同色同线型）
plot(ax, c(1)+Rm*cos(ang), c(2)+Rm*sin(ang), '--', ...
     'Color',S.c.circle,'LineWidth',1.8,'DisplayName','最小覆盖圆');
plot(ax, c(1), c(2), 'o', 'Color',S.c.circle, ...
     'MarkerFaceColor','w','MarkerSize',4.5,'LineWidth',1.2,'HandleVisibility','off');

% ⑤ 顶点标注（白底衬字，跨线时仍清晰）
for i = 1:size(P,1)
    nm  = char(vnames(i));
    pfx = regexprep(nm,'\d+$','');
    idx = regexprep(nm,'^\D+','');
    text(ax, P(i,1)+1.0, P(i,2)+1.0, [pfx '_' idx], ...
         'Color',S.ink,'BackgroundColor','w','Margin',0.8,'FontSize',10);
end

%% 坐标与标注
axis(ax,'equal');                       % 几何图必须等比例，防止形状失真
xlim(ax,[-20 20]);  xticks(ax,-20:5:20);
ylim(ax,[-18 18]);  yticks(ax,-15:5:15);   % 刻度与原图一致；仅留少量边距避免圆与图框相切
xlabel(ax,'x / m');
ylabel(ax,'y / m');
title(ax, {['定位域直径 D_k = ' num2str(D,'%.4f') ' m'], ...
           ['最小覆盖圆半径 R_{\rm min} = ' num2str(Rm,'%.4f') ' m']}, ...
      'FontSize',11);

lg = legend(ax,'Location','southoutside','Orientation','horizontal', ...
            'NumColumns',4,'Box','off');
lg.ItemTokenSize = [14 8];

%% 导出
paper_export(f,'legal_counterexample');
close(f);
