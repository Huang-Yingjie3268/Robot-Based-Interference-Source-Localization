function out = paper_export(f, name, outDir)
% Team project plotting utility; inputs: results/reference; outputs: results/generated/figures.

here = fileparts(mfilename('fullpath'));          % matlab/q2
if nargin < 3 || isempty(outDir)
    outDir = fullfile(here, '..', '..', 'results', 'generated', 'figures', 'q2');
end
if ~exist(outDir,'dir'); mkdir(outDir); end
fprintf('  导出目录：%s\n', outDir);

% —— 关闭交互工具栏，防止被写进导出图 ——
try, set(f,'Toolbar','none');  catch, end
try, set(f,'MenuBar','none');  catch, end
ax = findall(f,'Type','axes');
for k = 1:numel(ax)
    try
        if isprop(ax(k),'Toolbar') && ~isempty(ax(k).Toolbar)
            ax(k).Toolbar.Visible = 'off';
        end
    catch
    end
end
drawnow;

pdfPath = fullfile(outDir,[name '.pdf']);
pngPath = fullfile(outDir,[name '.png']);
exportgraphics(f, pdfPath, 'ContentType','vector','Padding','figure');
exportgraphics(f, pngPath, 'Resolution',200,'Padding','figure');

info = dir(pdfPath);
fprintf('已导出矢量图：%s （%.0f B）\n', pdfPath, info.bytes);
fprintf('已导出预览图：%s\n', pngPath);
out = pdfPath;
end
