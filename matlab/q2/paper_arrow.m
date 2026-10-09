function paper_arrow(ax, hText, target, col)
% Team project plotting utility; inputs: results/reference; outputs: results/generated/figures.

if nargin < 4 || isempty(col), col = [0.35 0.35 0.35]; end

drawnow;
u0 = ax.Units;                      % 图窗建图时常用厘米，这里临时换成归一化读位置
ax.Units = 'normalized';
axPos = ax.Position;
ax.Units = u0;
xl = xlim(ax);  yl = ylim(ax);
toFig = @(x,y) [axPos(1) + axPos(3)*(x-xl(1))/diff(xl), ...
                axPos(2) + axPos(4)*(y-yl(1))/diff(yl)];

e = get(hText, 'Extent');           % 文字框（数据坐标）
if target(2) < e(2)                 % 目标在文字下方 → 从底边中点出发
    p0 = [e(1)+e(3)/2, e(2)];
elseif target(2) > e(2)+e(4)        % 目标在上方 → 从顶边中点出发
    p0 = [e(1)+e(3)/2, e(2)+e(4)];
elseif target(1) < e(1)             % 目标在左侧 → 从左边中点出发
    p0 = [e(1), e(2)+e(4)/2];
else                                % 目标在右侧 → 从右边中点出发
    p0 = [e(1)+e(3), e(2)+e(4)/2];
end

q0 = toFig(p0(1), p0(2));
q1 = toFig(target(1), target(2));
assert(all(q0 > 0 & q0 < 1) && all(q1 > 0 & q1 < 1), ...
    'paper_arrow:outOfRange', ...
    '引导线端点越出 figure 范围（文字过宽或位置越界）：q0=[%.3f %.3f] q1=[%.3f %.3f]', ...
    q0(1), q0(2), q1(1), q1(2));

annotation('line', [q0(1) q1(1)], [q0(2) q1(2)], 'Color', col, 'LineWidth', 0.7);
end
