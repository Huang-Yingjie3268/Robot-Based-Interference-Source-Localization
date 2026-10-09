% Team project plotting utility; inputs: results/reference; outputs: results/generated/figures.

here = fileparts(mfilename('fullpath'));
addpath(here);
cd(here);

fprintf('== 图1：物理支撑集 / 保证接收可行域 / 最优第二检测点 ==\n');
fig_geometry;

fprintf('== 图2：最坏后验直径 J 的分布与候选区域三层结构 ==\n');
fig_J;

fprintf('== 图3：双参数灵敏度（δ 与 R）==\n');
fig_sensitivity;

fprintf('== 图4：源距对交会直径与交会角的影响 ==\n');
fig_source_dist;

fprintf('问题二插图重绘完毕。\n');
