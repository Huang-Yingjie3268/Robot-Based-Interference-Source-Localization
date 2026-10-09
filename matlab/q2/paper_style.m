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
set(groot,'defaultLegendBox','off');         % 图例去边框

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
%  ------------------------------------------------------------------
%  【问题一】
%    蓝  region    定位区域填充 / 定位域边界
%    橙  circle    最小覆盖圆及其半径 R_min（乙口径）
%    绿  area      定位域面积（辅助量，细线弱化 + 空心标记）
%    朱红 diam      直径 D_k 本身
%         diamcirc  以 D_k 为直径的圆（甲口径的"指定直径圆"）—— 与 diam 同色同族，
%                   靠线型区分（diam 粗实线、diamcirc 虚线）
%         leak      漏出量 Delta 的尺寸线（甲口径的量化证据）
%  ------------------------------------------------------------------
%  【问题二 · 已登记占用】"同一含义同色"的跨问延续：
%    蓝  region    保证接收可行域 F_rec 的边界与填充（问题一"定位域"族的延续）；
%                  最优第二检测点 S_2^*（最优点落在可行域边界上，与边界同族）
%    朱红 diam      最坏后验直径 J（图2 顺序色标、图3 曲线、图4(a) 曲线）——
%                  与问题一的直径 D_k 同族同色
%         worst     最坏源位置（"最坏情形"的标记，与"最坏直径"同族）
%    绿  area      质量约束层 Q_2(tau) 边界与近优域 A_eta 格点（辅助量，细线弱化）
%    灰  guide     源可能区域 Omega、对称轴 theta_1、基线选点、参考线
%    紫  aux1      交会角 gamma（与"直径"并列的几何指量；问题二首次占用）
%  ------------------------------------------------------------------
%  【问题三 / 问题四】备用位：天蓝 aux2、黄 aux3（占用前先在此登记语义）
%  ------------------------------------------------------------------
S.font    = zhFont;
S.palette = palette;
S.c = struct( ...
    'region',   palette(1,:), ...   % 定位区域 / 定位域边界 / 可行域 F_rec —— 蓝
    'circle',   palette(2,:), ...   % 最小覆盖圆、最小覆盖圆半径 R_min —— 橙
    'area',     palette(3,:), ...   % 面积 / 质量约束层（辅助量，细线弱化）—— 绿
    'diam',     palette(4,:), ...   % 直径 D_k / 最坏后验直径 J —— 朱红
    'diamcirc', palette(4,:), ...   % 以 D_k 为直径的圆（甲口径）—— 朱红，虚线
    'leak',     palette(4,:), ...   % 漏出量 Delta 尺寸线 —— 朱红，最粗
    'worst',    palette(4,:), ...   % 最坏源位置（"最坏情形"标记）—— 朱红
    'aux1',     palette(5,:), ...   % 问题二：交会角 gamma —— 紫
    'aux2',     palette(6,:), ...   % 备用 —— 天蓝
    'aux3',     palette(7,:));      % 备用 —— 黄

S.ink     = [0.15 0.15 0.15];     % 文字/刻度主色（近黑，避免纯黑发死）
S.guide   = [0.50 0.50 0.50];     % 参考线、对照量（如题面取值、基线选点）
S.unbnd   = [0.82 0.82 0.82];     % 无定义区（如"交会多边形无界"）的填充灰
S.lfill   = 0.12;                 % 区域填充不透明度（弱化为底衬，勿调高以免抢镜）
S.zhFont  = zhFont;

%% 5. 尺寸参数（脚本引用这些字段，避免各图各写一套数字）
S.lw       = 1.8;     % 常规数据线
S.lw_thin  = 1.3;     % 辅助量/参考线
S.lw_frame = 0.75;    % 坐标框线
S.lw_heavy = 2.6;     % 强调线（直径 / 可行域边界）
S.lw_leak  = 2.6;     % 漏出量尺寸线
S.ms       = 6.5;     % 标记尺寸
S.fs_axes  = 10.5;    % 坐标区字号
S.fs_label = 10.5;    % 轴标签字号
S.fs_note  = 9.5;     % 图内注释字号
S.fs_leg   = 9;       % 图例字号

%% 6. 版面换算常数（保证全篇插入后缩放一致、字号一致）
%  图窗宽度 figW(cm) = S.Wstdio × 正文 \includegraphics 的 \textwidth 比例。
%  这样所有图被插入正文后，缩放系数相同，印出的字号也相同（约 9.5 pt）。
S.Wstdio = 16.855;    % cm / \textwidth（由问题一定稿：0.62\textwidth ↔ 10.45 cm）

% 让默认色序与国赛色序一致，避免手工取色
set(groot,'defaultAxesColorOrder',palette);
end
