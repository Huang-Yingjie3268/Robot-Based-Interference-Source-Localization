function t = paper_coltxt(C, j)
% Team project plotting utility; inputs: results/reference; outputs: results/generated/figures.

n = size(C,1);
t = cell(n,1);
for i = 1:n
    x = C{i,j};
    if isa(x,'missing')
        t{i} = '';
    elseif ischar(x) || isstring(x)
        t{i} = char(x);
    else
        t{i} = char(string(x));
    end
end
end
