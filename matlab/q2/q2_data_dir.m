function d = q2_data_dir(scriptDir)
% Team project plotting utility; inputs: results/reference; outputs: results/generated/figures.

cand = {fullfile(scriptDir, '..', '..', 'results', 'reference', 'q2')};

d = '';
for k = 1:numel(cand)
    c = char(cand{k});
    if exist(fullfile(c, 'grid_maxdist.csv'), 'file')
        d = c;
        return;
    end
end
error('q2_data_dir:notFound', ...
    '未找到问题二数据目录（应含 grid_maxdist.csv）；已试：%s', strjoin(cand, ' | '));
end
