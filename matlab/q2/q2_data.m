function D = q2_data(scriptDir)
% Team project plotting utility; inputs: results/reference; outputs: results/generated/figures.

dat = q2_data_dir(scriptDir);
g = @(f) fullfile(dat, f);

%% ---------- 1. 最大可测距离场 grid_maxdist.csv（x 外循环、y 内循环）----------
[hg, Cg] = paper_read_csv(g('grid_maxdist.csv'));
x = paper_col(Cg, paper_colidx(hg,'x'));
y = paper_col(Cg, paper_colidx(hg,'y'));
m = paper_col(Cg, paper_colidx(hg,'maxdist'));
s = paper_col(Cg, paper_colidx(hg,'margin'));
[X, Y, M] = paper_grid3(x, y, m);
[~, ~, Sp] = paper_grid3(x, y, s);
D.X = X;  D.Y = Y;  D.M = M;  D.S = Sp;
D.xs = unique(x);  D.ys = unique(y);
% ★ 耦合接收半径口径（方案B）：F_rec 的判据是 margin ≤ 0，**不是** maxdist ≤ 1000。
%   margin(S2) = max_g(||S2-g|| - R*(g))，R*(g) = max(1000, ||S1-g||)。
D.inF = (D.S <= 0);                        % 保证接收可行域 F_rec（硬可测域 A_2）

chk('网格规模 ny×nx', size(M,1)*size(M,2), 22311, 0);
chk('F_rec 格点数（论文表 7：A_2=1218700）', nnz(D.inF), 12187, 0);
chk('F_rec 包围盒 x 下界（耦合口径左端=0）', min(X(D.inF)), 0, 1e-9);
chk('F_rec 包围盒 x 上界', max(X(D.inF)), 1000, 1e-9);
chk('F_rec 包围盒 y 下界', min(Y(D.inF)), -850, 1e-9);
chk('F_rec 包围盒 y 上界', max(Y(D.inF)), 850, 1e-9);
chk('maxdist 最小值', min(M(:)), 750.2284, 1e-3);
mgs = min(D.S(:));
chk('margin 最小值（S1 自身在 F_rec 内，应为 0 或负）', mgs, -499.8477, 1e-3);

%% ---------- 2. 中心构型主答案 q2_center_result.csv ----------
% ★ 必须放在最前面：J* 是后面所有阈值与标注的唯一来源，不允许在脚本里另写一份。
[hc, Cc] = paper_read_csv(g('q2_center_result.csv'));
assert(size(Cc,1) == 1, 'q2_center_result.csv 应恰有 1 行数据');
gc = @(n) paper_col(Cc, paper_colidx(hc, n));
D.S2c       = [gc('S2_x'), gc('S2_y')];
D.Jstar     = gc('J_worst_m');             % 主答案：最坏后验直径
D.JstarCSV  = D.Jstar;                     % 同名别名，供断言与文档引用
D.Rmin      = gc('R_min_m');
D.dmax      = gc('maxdist_m');
D.theta2w   = gc('theta2_worst_deg');

chk('J*（论文表 2：110.97）',              D.Jstar, 110.96931861216545, 1e-9);
chk('R_min（论文表 2：55.4847）',          D.Rmin,  55.484659306082726, 1e-9);
chk('d_max(S_2*)（论文表 2：1000.0000）',   D.dmax,  1000.0, 1e-9);
chk('最坏 theta_2（论文表 2：42.0387）',   D.theta2w, 42.0401352657, 1e-9);
chk('R_min 与 J*/2 的关系（论文表 4 口径）', D.Rmin, D.Jstar/2, 1e-6);

% 候选区域三层结构的阈值（论文 §5.5：tau = 1.2*J*，eta = 0.6 m 与网格分辨率同阶）
D.eta     = 0.6;                           % 论文给定的近优域容差
D.tau     = 1.2 * D.Jstar;                 % 优质域阈值
D.thrNear = D.Jstar + D.eta;               % 近优域阈值
chk('优质域阈值 1.2*J*（论文表 7：133.16）', D.tau, 133.163182, 1e-6);
chk('近优域阈值 J*+eta（论文表 7：111.57）', D.thrNear, 111.56931861216545, 1e-6);

%% ---------- 3. 最坏后验直径场 grid_J.csv ----------
[hj, Cj] = paper_read_csv(g('grid_J.csv'));
xj = paper_col(Cj, paper_colidx(hj,'x'));
yj = paper_col(Cj, paper_colidx(hj,'y'));
jj = paper_col(Cj, paper_colidx(hj,'J'));      % 空串 → NaN（该点交会多边形无界）
[Xj, Yj, J] = paper_grid3(xj, yj, jj);
assert(isequal(Xj, X) && isequal(Yj, Y), 'grid_J 与 grid_maxdist 的网格不一致');
D.J = J;
D.noJ = isnan(J) & D.inF;                      % 口径B 下 P1 有界 ⟹ R2 恒有界 ⟹ 该集合恒空
D.aboveRange = (J > 1000);                     % 超出显示量程（论文口径：>1000 截断）

chk('J 有定义的点数', nnz(~isnan(J)), 12187, 0);
chk('可行域内 J 未定义（无界）点数（口径B 下恒为 0）', nnz(D.noJ), 0, 0);
% ★ 耦合口径下 F_rec 扩到 x∈[0,1005]、|y|≤850（分界弧 r=1000 定界，见 Γ 构造），
%   新增区的最坏直径可以超过 1000 m
%   （几何量 diam(P1)=1495.0031 m 是上界），故本行不再恒为 0——这正是图 2 中 ">1000 截断" 的来源。
chk('J>1000 的点数（耦合口径下新增区的合法取值）', nnz(D.aboveRange), 291, 0);

%% ---------- 4. 关键点 keypoints.csv ----------
[hk, Ck] = paper_read_csv(g('keypoints.csv'));
nmk = paper_coltxt(Ck, 1);
kx  = paper_col(Ck, paper_colidx(hk,'x'));
ky  = paper_col(Ck, paper_colidx(hk,'y'));
D.S1    = kp(nmk, kx, ky, 'S1');
D.S2up  = kp(nmk, kx, ky, 'S2_opt_up');
D.S2dn  = kp(nmk, kx, ky, 'S2_opt_dn');
D.worstEnd1 = kp(nmk, kx, ky, 'worst_end_1');
D.worstEnd2 = kp(nmk, kx, ky, 'worst_end_2');
D.base  = kp(nmk, kx, ky, 'baseline');
chk('S_2^* 上镜像 x', D.S2up(1), 843.0442010349119, 1e-6);
chk('S_2^* 上镜像 y', D.S2up(2), 545.5139116553951, 1e-6);
chk('S_2^* 下镜像 x', D.S2dn(1), 843.0442010349119, 1e-6);
chk('S_2^* 下镜像 y', D.S2dn(2), -545.5139116553951, 1e-6);
chk('最坏分支端点 r（应在 supp_1 外弧 R_max 上）', max(hypot(D.worstEnd1(1),D.worstEnd1(2)), ...
                                                        hypot(D.worstEnd2(1),D.worstEnd2(2))), 1500.0000002713778, 5e-3);
% 关键点与中心构型结果必须同源（同一份 S_2^*，不允许两处各写一份）
chk('S_2^* 与 J* 同源一致', norm(D.S2c - D.S2dn), 0, 1e-9);

% 最坏源必须落在 supp_1 下直边、r=1500 处（论文表 4 的说法）
% 最坏分支的直径一端落在 supp_1 的远端直边上（由 q2_worst_branch.csv 转存，保留 6 位小数）
rw = hypot(1499.7715430000001, 26.178609999999999);
aw = atan2d(26.178609999999999, 1499.7715430000001);
chk('最坏源半径（= R_max）', rw, 1500.0000002713778, 5e-3);
chk('最坏源方位角（supp_1 直边 ±delta）', abs(abs(aw) - 1.0), 0, 5e-3);

% 基线选点（论文 §5.6：S_1+750u(theta_1)+(0,500)）
chk('基线选点 x', D.base(1), 750, 1e-9);
chk('基线选点 y', D.base(2), 500, 1e-9);
D.baseLen = hypot(D.base(1), D.base(2));            % 901.3878 m
D.optLen  = hypot(D.S2up(1), D.S2up(2));            % 1004.1459 m

%% ---------- 5. 灵敏度 q2_sensitivity_delta.csv / q2_sensitivity_R.csv ----------
[hd, Cd] = paper_read_csv(g('q2_sensitivity_delta.csv'));
D.dlt   = paper_col(Cd, paper_colidx(hd,'delta_deg'));
D.dltJ  = paper_col(Cd, paper_colidx(hd,'J_worst_m'));
[hr, Cr] = paper_read_csv(g('q2_sensitivity_R.csv'));
D.Rr    = paper_col(Cr, paper_colidx(hr,'R_recv_m'));
D.RrJ   = paper_col(Cr, paper_colidx(hr,'J_worst_m'));
assert(numel(D.dlt) == 5 && numel(D.Rr) == 5, '灵敏度表应为 5 档');
chk('delta=0.8 对应 J*（论文表 5：90.32）', D.dltJ(1), 90.32224517403515, 1e-9);
dJd = D.dltJ(3) - D.Jstar;   % δ=1.0 档：趋势搜索的可行点值，必 >= J*
chk('delta=1.0 档不得低于全局最小 J*',          min(dJd, 0), 0, 1e-9);
chk('delta=1.0 档与 J* 的间距（趋势搜索分辨率内）', dJd, 0, 1e-3);
chk('delta=1.2 对应 J*（论文表 5：130.92）', D.dltJ(5), 130.92399937062262, 1e-9);
dJr = D.RrJ(1) - D.Jstar;    % R=1000 档：同上
chk('R=1000 档不得低于全局最小 J*',            min(dJr, 0), 0, 1e-9);
chk('R=1000 档与 J* 的间距（趋势搜索分辨率内）',   dJr, 0, 1e-3);
chk('R=1500 对应 J*（论文表 6：76.92）',     D.RrJ(5), 76.9154909810497, 1e-9);
assert(all(diff(D.dltJ) > 0),  'J* 关于 delta 必须单调增（论文 §5.4 结论）');
assert(all(diff(D.RrJ)  < 0),  'J* 关于 R 必须单调减（论文 §5.4 结论）');

%% ---------- 6. 源距曲线 q2_source_dist.csv ----------
[hs, Cs] = paper_read_csv(g('q2_source_dist.csv'));
D.r1     = paper_col(Cs, paper_colidx(hs,'r1_m'));
D.gamma  = paper_col(Cs, paper_colidx(hs,'gamma_deg'));
D.Dideal = paper_col(Cs, paper_colidx(hs,'D_ideal_m'));
assert(numel(D.r1) == 300, 'q2_source_dist.csv 应为 300 行');
assert(all(~isnan(D.Dideal)), 'q2_source_dist.csv 的 D_ideal 不应含 inf/空值（status 全为 polygon）');
assert(all(diff(D.gamma) <= 1e-9), '交会角关于源距必须单调减（论文图 3(b) 结论）');
[~, iv] = min(D.Dideal(D.r1 >= 800));                 % 只在内段找谷底（与论文口径一致）
ii = find(D.r1 >= 800);  D.vly = ii(iv);
[~, ip] = max(D.Dideal);  D.pk = ip;                  % 全局最大
chk('谷底源距（论文表 4 标注 840）',  D.r1(D.vly), 840, 1e-9);
chk('谷底 D_ideal（论文表 4 标注 35.04）', D.Dideal(D.vly), 35.044732000000003, 1e-6);
chk('谷底交会角（论文表 4 标注 90.32°）',  D.gamma(D.vly), 90.319732, 1e-6);
chk('全局最大源距（论文表 4 标注 1445，内部极大）', D.r1(D.pk), 1445, 1e-9);
chk('全局最大 D_ideal（论文表 4 标注 110.35）', D.Dideal(D.pk), 110.353126, 1e-6);
% 论文 §5.3 的形态结论（口径B）：谷底 -> 内部极大 -> 因 supp_1 远弧截断而回落。
% ★ 早先版本断言"最大在端点 r_1=1500 m""r_1>=800 单调不减"，那是口径A（未截断双锥交）的形态；
%   口径B 下 P_1 有界（r<=1500 来自 S_1），源逼近远弧时锥窗被 P_1 自身截断，极大被迫内移。
%   故把"形态"换成四条可证伪的断言：极大严格在内部、右端点显著低于该极大、
%   并且 [谷底, 极大] 增与 [极大, 右端点] 减各成一调。
assert(D.pk < numel(D.r1), ...
    '口径B 下全局最大应取在内部（实测 r_1=%.0f m）：源逼近远弧时被 supp_1 截断', D.r1(D.pk));
assert(D.Dideal(end) < D.Dideal(D.pk) - 1e-6, ...
    '右端点 %.4f m 应显著低于内部极大 %.4f m（远弧截断效应）', D.Dideal(end), D.Dideal(D.pk));
s1 = D.Dideal(D.r1 >= D.r1(D.vly) & D.r1 <= D.r1(D.pk));
s2 = D.Dideal(D.r1 >= D.r1(D.pk));
assert(all(diff(s1) >= -1e-6), 'r_1∈[谷底 %.0f, 内部极大 %.0f] 段 D_ideal 应单调不减', ...
       D.r1(D.vly), D.r1(D.pk));
assert(all(diff(s2) <=  1e-6), 'r_1∈[内部极大 %.0f, 右端点] 段 D_ideal 应单调不增', D.r1(D.pk));

%% ---------- 7. 关键检查值 q2_checks.csv（图 1 标注 F_rec 左端点）----------
[hq, Cq] = paper_read_csv(g('q2_checks.csv'));
nmq = paper_coltxt(Cq, 1);
vrq = paper_coltxt(Cq, 2);
% ★ value 列必须按**文本**读入、只把命中的那一格转成数：
%   q2_checks.csv 里 key = k9_sens_quant / k10_theta_full_circle 的若干行是复合串
%   （如 "+10.4124/+10.2347/+10.0616/+9.8931"）或结论文字。整列转数值会直接报
%   "无法解析为数值" 而使**全部四张图**都出不来——这类行与图元无关，不能因为
%   顺手整列转换就废掉出图。改用 pickNum：先按 key/variant 命中行，再单独转那一格。
vlq = paper_coltxt(Cq, paper_colidx(hq,'value'));
D.frecLeft = pickNum(nmq, vrq, vlq, 'k4_frec_left', 'theta1轴');
D.k1_1m    = pickNum(nmq, vrq, vlq, 'k1_grid', '1m');
D.k6_impr  = pickNum(nmq, vrq, vlq, 'k6_improvement', '(J_base-J*)/J_base');
chk('F_rec 左端点（论文 §5.1：0.0000 m，耦合口径）', D.frecLeft, 0.0000, 1e-6);
chk('1 m 网格 J*（论文表 9：110.97）',       D.k1_1m,    110.9735, 1e-6);
chk('相对基线改进（论文表 2：14.47%）',      D.k6_impr,  14.4688,    1e-6);
% 改进率必须能由 CSV 里的两个数复算出来（禁止"结果先行"的陪衬数字）
chk('改进率可由 (J_base-J*)/J_base 复算', D.k6_impr, (129.741396-D.Jstar)/129.741396*100, 1e-3);

%% ---------- 8. 候选区域三层结构 q2_candidate_layers.csv ----------
[hl, Cl] = paper_read_csv(g('q2_candidate_layers.csv'));
nm = paper_coltxt(Cl, 1);
cellv = paper_col(Cl, paper_colidx(hl,'cells'));
areav = paper_col(Cl, paper_colidx(hl,'area_m2'));
D.nA2   = pickN(nm, cellv,  'A2_hard_feasible');
D.nQ2   = pickN(nm, cellv,  'Q2_quality');
D.nAeta = pickN(nm, cellv,  'A_eta_near_optimal');
D.aA2   = pickN(nm, areav,  'A2_hard_feasible');
D.aQ2   = pickN(nm, areav,  'Q2_quality');
D.aAeta = pickN(nm, areav,  'A_eta_near_optimal');
chk('A_2 格点数（论文表 7）',      D.nA2,   12187, 0);
chk('Q_2 格点数（论文表 7）',      D.nQ2,   1310, 0);
chk('A_eta 格点数（论文表 7）',    D.nAeta,   8, 0);
chk('A_2 面积 m^2（论文表 7）',    D.aA2,  1218700, 0);
chk('Q_2 面积 m^2（论文表 7）',    D.aQ2,  131000, 0);
chk('A_eta 面积 m^2（论文表 7）',  D.aAeta,  800, 0);
chk('A_2 格点数与网格复算一致',    D.nA2, nnz(D.inF), 0);
chk('Q_2 格点数与网格复算一致',    D.nQ2, nnz(~isnan(J) & D.inF & J <= D.tau), 0);
chk('A_eta 格点数与网格复算一致',  D.nAeta, nnz(~isnan(J) & D.inF & J <= D.thrNear), 0);
end

%% ============================== 局部函数 ==============================
function chk(name, got, want, tol)
if abs(got - want) > tol
    error('q2_data:assert', ...
        '【数值断言失败】%s：CSV 实得 %.17g，论文期望 %.17g（容差 %g）', ...
        name, got, want, tol);
end
end

function p = kp(nm, kx, ky, name)
i = find(strcmp(nm, name), 1);
assert(~isempty(i), 'keypoints.csv 缺少关键点 "%s"', name);
p = [kx(i), ky(i)];
end

function v = pick(nmA, nmB, val, a, b)
i = find(strcmp(nmA,a) & strcmp(nmB,b), 1);
assert(~isempty(i), 'q2_checks.csv 缺少 "%s"/"%s" 行', a, b);
v = val(i);
end

function v = pickNum(nmA, nmB, valTxt, a, b)
%PICKNUM  从 q2_checks.csv 命中一行并只把该格的文本转成数值
%   （value 列含 k9/k10 的复合串，不能整列 paper_col）
i = find(strcmp(nmA,a) & strcmp(nmB,b), 1);
assert(~isempty(i), 'q2_checks.csv 缺少 "%s"/"%s" 行', a, b);
v = str2double(valTxt{i});
assert(~isnan(v), 'q2_checks.csv 的 "%s"/"%s" 行取值 "%s" 不是数值', a, b, valTxt{i});
end

function v = pickN(nm, val, name)
i = find(strcmp(nm,name), 1);
assert(~isempty(i), 'q2_candidate_layers.csv 缺少层 "%s"', name);
v = val(i);
end
