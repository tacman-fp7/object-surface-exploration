function plotMesh(this, contactLocations, plotPC, figNum, nPoints, plotTitle)

if nargin < 6; plotTitle = ''; end;
if nargin < 5; nPoints = this.nPoints; end;
if nargin < 4; figure; else figure(figNum); end;
if nargin < 3; plotPC = false; end;

[az, el] = view();
x = contactLocations(:, 1);
y = contactLocations(:, 2);
surface = contactLocations(:, 3);


xlin = linspace(min(x),max(x), nPoints);
ylin = linspace(min(y),max(y), nPoints);
[XT,YT] = meshgrid(xlin,ylin);

f = scatteredInterpolant(x,y,surface, 'natural');
ZT = f(XT,YT);

mesh(XT, YT, ZT);
title(plotTitle, 'fontsize', 24);

if(plotPC)
    hold on
    scatter3(...
        contactLocations(this.nPadding + 1:end, 1),...
        contactLocations(this.nPadding + 1:end, 2),...
        contactLocations(this.nPadding + 1:end, 3),...
        'fill', 'markerFaceColor', 'blue', 'sizeData', [90]);
    hold off
end

view([az, el]);
end