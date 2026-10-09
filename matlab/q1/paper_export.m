function paper_export(f, name, outDir)
% Team project plotting utility; inputs: results/reference; outputs: results/generated/figures.

if nargin < 3 || isempty(outDir)
    here   = fileparts(mfilename('fullpath'));   % matlab/q1
    root   = fileparts(fileparts(here));         % 工程根目录
    figDir = fullfile(root,'results','generated','figures','q1');           % Regenerated figures (ignored by Git)
    pngDir = fullfile(root,'results','generated','figures','q1');
else
    figDir = outDir;
    pngDir = outDir;
end
if ~exist(figDir,'dir'); mkdir(figDir); end
if ~exist(pngDir,'dir'); mkdir(pngDir); end

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

pdfPath = fullfile(figDir,[name '.pdf']);
pngPath = fullfile(pngDir,[name '.png']);
exportgraphics(f, pdfPath, 'ContentType','vector');
exportgraphics(f, pngPath, 'Resolution',200);

fprintf('已导出矢量图：%s\n', pdfPath);
fprintf('已导出预览图：%s\n', pngPath);
end
