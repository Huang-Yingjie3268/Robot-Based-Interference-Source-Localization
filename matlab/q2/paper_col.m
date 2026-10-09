function v = paper_col(C, j)
% Team project plotting utility; inputs: results/reference; outputs: results/generated/figures.

n = size(C,1);
v = nan(n,1);
for i = 1:n
    x = C{i,j};
    if isa(x,'missing')
        continue;                                    % 空字段 → NaN
    end
    if isnumeric(x)
        assert(isscalar(x), '第 %d 行第 %d 列不是标量', i+1, j);
        v(i) = double(x);
    elseif ischar(x) || isstring(x)
        s = strtrim(char(x));
        if isempty(s), continue; end                 % 空值 → NaN
        d = str2double(s);
        assert(~isnan(d), '第 %d 行第 %d 列无法解析为数值："%s"', i+1, j, s);
        v(i) = d;
    else
        error('第 %d 行第 %d 列类型不支持：%s', i+1, j, class(x));
    end
end
end
