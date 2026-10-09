function [X, Y, Z, xs, ys] = paper_grid3(x, y, z)
% Team project plotting utility; inputs: results/reference; outputs: results/generated/figures.

xs = unique(x);
ys = unique(y);
nx = numel(xs);
ny = numel(ys);
assert(numel(x) == nx*ny, ...
    '网格点数(%d) ≠ nx*ny(%d×%d=%d)，说明长表有缺行或多行', numel(x), nx, ny, nx*ny);

X = reshape(x, ny, nx);
Y = reshape(y, ny, nx);
Z = reshape(z, ny, nx);

Xref = repmat(xs(:)', ny, 1);
Yref = repmat(ys(:),  1, nx);
assert(isequal(size(X), size(Xref)) && isequal(size(Y), size(Yref)), ...
    '网格尺寸与 unique(x)/unique(y) 不一致：X %s vs %s', mat2str(size(X)), mat2str(size(Xref)));
assert(max(abs(X - Xref), [], 'all') < 1e-9, 'x 不是规整网格（x 外循环假设不成立）');
assert(max(abs(Y - Yref), [], 'all') < 1e-9, 'y 不是规整网格（y 内循环假设不成立）');
end
