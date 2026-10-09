function cm = paper_cmap(name, n)
% Team project plotting utility; inputs: results/reference; outputs: results/generated/figures.

if nargin < 2 || isempty(n), n = 256; end
assert(isnumeric(n) && isscalar(n) && n >= 2, 'n 必须是不小于 2 的整数');

switch lower(name)
    case 'red'      % 朱红族（端点含国赛色序的"橙"与"朱红"）
        stops = [ 1.000 0.972 0.925;   % 浅米
                  0.992 0.855 0.640;   % 浅橙
                  0.902 0.616 0.000;   % 橙   230 157 0
                  0.835 0.369 0.000;   % 朱红 213  94 0
                  0.470 0.180 0.020];  % 深棕红
        pos = [0 0.28 0.58 0.80 1.00];
    case 'blue'     % 蓝族（端点含国赛色序的"天蓝"与"蓝"）
        stops = [ 0.945 0.972 0.988;   % 极浅蓝
                  0.694 0.843 0.925;   % 浅天蓝
                  0.337 0.706 0.882;   % 天蓝  86 180 233
                  0.000 0.447 0.698];  % 蓝     0 114 178
        pos = [0 0.35 0.70 1.00];
    otherwise
        error('paper_cmap:unknown', '未知色标名：%s（可用：red / blue）', name);
end

t = linspace(0,1,n)';
cm = zeros(n,3);
for k = 1:3
    cm(:,k) = interp1(pos, stops(:,k), t, 'pchip');
end
cm = min(max(cm,0),1);      % pchip 可能轻微过冲，夹到 [0,1]

% 单调亮度自检：亮度必须随 t 单调（顺序色标的硬要求）
Y = 0.2126*cm(:,1) + 0.7152*cm(:,2) + 0.0722*cm(:,3);
assert(all(diff(Y) <= 1e-9), '色标亮度不单调，违反顺序色标要求');
end
