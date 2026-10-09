% Team project plotting utility; inputs: results/reference; outputs: results/generated/figures.

here = fileparts(mfilename('fullpath'));
addpath(here);
cd(here);

fprintf('== 图1：正三角形合法反例 ==\n');
fig_legal_counterexample;

fprintf('== 图2：示向误差上界灵敏度 ==\n');
fig_sensitivity_delta;

fprintf('问题一插图重绘完毕。\n');
