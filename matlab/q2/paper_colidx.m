function j = paper_colidx(hdr, name)
% Team project plotting utility; inputs: results/reference; outputs: results/generated/figures.

j = find(strcmp(hdr, name), 1);
if isempty(j)
    error('paper_colidx:notFound', ...
        'CSV 中找不到列 "%s"；实际列为：%s', name, strjoin(hdr, ' | '));
end
end
