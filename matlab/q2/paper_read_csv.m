function [hdr, C] = paper_read_csv(path)
% Team project plotting utility; inputs: results/reference; outputs: results/generated/figures.

if ~exist(path,'file')
    error('paper_read_csv:missing', '数据文件不存在：%s', path);
end
c = readcell(path, 'Delimiter', ',');
assert(iscell(c), '读取结果不是元胞：%s', path);
assert(size(c,1) >= 2, 'CSV 至少要「表头 + 1 行数据」：%s（实际 %d 行）', path, size(c,1));
assert(size(c,2) >= 1, 'CSV 列数异常：%s', path);

hdr = cell(1, size(c,2));
for j = 1:size(c,2)
    if isa(c{1,j},'missing')
        error('paper_read_csv:badHeader', '第 1 行第 %d 列的表头是空字段：%s', j, path);
    end
    hdr{j} = char(string(c{1,j}));
end
C = c(2:end, :);
end
