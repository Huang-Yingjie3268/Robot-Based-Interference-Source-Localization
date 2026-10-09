function plot_directional_coverage_geometry()
% Plot the retained 24-waypoint geometry; output: results/generated/figures/q4.

    here = fileparts(mfilename('fullpath'));
    setappdata(0,'Q4_FIGDIR',fullfile(here,'..','..','results','generated','figures','q4'));
    if ~exist(getappdata(0,'Q4_FIGDIR'),'dir'), mkdir(getappdata(0,'Q4_FIGDIR')); end
    C.R0=1800; C.RrecvLo=1000; C.RrecvHi=1500;
    C.nInner=10; C.rInner=1000; C.nOuter=13; C.rOuter=1880;
    C.clear=20; C.phiWorst=173.545; C.gapMargin=6.455;
    q4_fig1_geometry(C);
end

function fp = figout(name)
    d = getappdata(0,'Q4_FIGDIR');
    if isempty(d), d = fullfile(pwd,'figures'); end
    fp = fullfile(d, name);
end

function W = waypoints(C)
    W = [0 0];
    for k = 0:C.nInner-1
        a = 2*pi*k/C.nInner;
        W(end+1,:) = [C.rInner*cos(a), C.rInner*sin(a)];
    end
    for k = 0:C.nOuter-1
        a = 2*pi*(k+0.5)/C.nOuter;
        W(end+1,:) = [C.rOuter*cos(a), C.rOuter*sin(a)];
    end
end

% =====================================================================
function q4_fig1_geometry(C)
%Fig1  24 点双环构型 + 最坏角度间隙（已修复文字重叠）
    W = waypoints(C);
    f = figure('Position',[60 60 1000 780]);
    hold on; axis equal;

    tt = linspace(0,2*pi,721);
    hArena = patch(C.R0*cos(tt), C.R0*sin(tt), [0.97 0.97 1.0], ...
          'EdgeColor',[0.25 0.25 0.25],'LineWidth',1.8,'FaceAlpha',0.5);

    pw = [1800 0];
    hRecv = plot(pw(1)+1000*cos(tt), pw(2)+1000*sin(tt), '--', ...
         'Color',[0.85 0.45 0.15],'LineWidth',1.4);

    inIdx  = 2:1+C.nInner;
    outIdx = 2+C.nInner:1+C.nInner+C.nOuter;
    hIn  = plot(W(inIdx,1),  W(inIdx,2),  'o', 'MarkerSize',9, ...
         'MarkerFaceColor',[0.20 0.44 0.74],'MarkerEdgeColor','k','LineStyle','none');
    hOut = plot(W(outIdx,1), W(outIdx,2), 's', 'MarkerSize',8, ...
         'MarkerFaceColor',[0.90 0.45 0.13],'MarkerEdgeColor','k','LineStyle','none');
    hCen = plot(W(1,1), W(1,2), 'p', 'MarkerSize',15, ...
         'MarkerFaceColor',[0.13 0.62 0.30],'MarkerEdgeColor','k','LineStyle','none');

    cnt = 0; pts = [];
    for i = 1:size(W,1)
        if norm(W(i,:)-pw) <= 1000+1e-9
            cnt = cnt+1; pts(cnt,:) = W(i,:);
        end
    end
    for i = 1:size(pts,1)
        plot([pw(1) pts(i,1)], [pw(2) pts(i,2)], ':', ...
             'Color',[0.45 0.45 0.55],'LineWidth',1.0);
    end
    hWorst = plot(pw(1), pw(2), 'r*', 'MarkerSize',17, 'LineWidth',2.0);

    % 标注移到图框上方（增大 ylim 上限），避免与圆弧/数据点重叠
    text(-2050, 2400, ...
        sprintf('Sampled worst point (1800, 0): maximum gap = %.3f deg\nMargin below 180 deg = %.3f deg', ...
        C.phiWorst, C.gapMargin), ...
        'FontSize',13,'Color',[0.75 0.10 0.10], ...
        'HorizontalAlignment','left','VerticalAlignment','top', ...
        'EdgeColor',[0.75 0.10 0.10],'BackgroundColor',[1 1 1], 'Interpreter','none');

    text(-2050, -2400, sprintf('Outer ring: %d m; observation points extend beyond the 1800 m target region.', ...
        C.rOuter), 'FontSize',12,'Color',[0.80 0.35 0.05], ...
        'HorizontalAlignment','left','BackgroundColor',[1 1 1], 'Interpreter','none');

    xlabel('x (m)','FontSize',15); ylabel('y (m)','FontSize',15);
    title('Directional coverage: 24-waypoint geometry','FontSize',14.5,'FontWeight','bold','Interpreter','none');
    legend([hArena hRecv hIn hOut hCen hWorst], ...
           {'Target region: 1800 m','Reception disk: 1000 m','Inner ring: 10 points at 1000 m', ...
            'Outer ring: 13 points at 1880 m','Center','Sampled worst point'}, ...
           'Location','southoutside','Orientation','horizontal','FontSize',11.5,'NumColumns',3,'Interpreter','none');
    grid on; set(gca,'GridLineStyle',':','GridAlpha',0.5,'FontSize',13);
    xlim([-2250 2900]); ylim([-2700 2650]);
    exportgraphics(f, figout('q4_directional_coverage.png'), 'Resolution', 600);
    close(f);
end

% =====================================================================
