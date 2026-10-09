% Team project plotting utility; inputs: results/reference; outputs: results/generated/figures.

clear; close all; clc;
R=1800; rmin=1000;
t=linspace(0,2*pi,720);
default=[0 0; 1000*cos((0:7)'*pi/4) 1000*sin((0:7)'*pi/4)];
r6=R/(2*cos(pi/6)); ring6=[r6*cos((0:5)'*pi/3) r6*sin((0:5)'*pi/3)];
fig=figure('Color','w','Position',[100 100 1450 650]);
tiledlayout(1,2,'Padding','compact','TileSpacing','compact');
nexttile; hold on; axis equal; grid on;
plot(R*cos(t),R*sin(t),'k-','LineWidth',1.2);
for i=1:size(default,1), plot(default(i,1)+rmin*cos(t),default(i,2)+rmin*sin(t),'-','LineWidth',0.6); end
scatter(default(:,1),default(:,2),40,'filled');
title('默认9点：中心+8环@1000 m'); xlabel('x / m'); ylabel('y / m');
text(-1650,-1650,'最坏覆盖距离 956.05 m；余量 43.95 m','FontSize',10);
xlim([-1900 1900]); ylim([-1900 1900]);
nexttile; hold on; axis equal; grid on;
plot(R*cos(t),R*sin(t),'k-','LineWidth',1.2);
for i=1:size(ring6,1), plot(ring6(i,1)+rmin*cos(t),ring6(i,2)+rmin*sin(t),'-','LineWidth',0.6); end
scatter(ring6(:,1),ring6(:,2),40,'filled');
title('ring6反例'); xlabel('x / m'); ylabel('y / m');
text(-1650,-1650,'最坏覆盖距离 1039.23 m > 1000 m','FontSize',10);
xlim([-1900 1900]); ylim([-1900 1900]);
out=fullfile(fileparts(mfilename('fullpath')),'..','..','results','generated','figures','q3_coverage.pdf');
if ~exist(fileparts(out),'dir'), mkdir(fileparts(out)); end
exportgraphics(fig,out,'ContentType','vector');
