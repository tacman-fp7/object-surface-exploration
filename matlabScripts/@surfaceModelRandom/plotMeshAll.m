function plotMeshAll(this, contactLocations, plotTitle, viewPars)





x = contactLocations(:, 1);
y = contactLocations(:, 2);
surface = contactLocations(:, 3);


xlin = linspace(min(x),max(x), this.nPoints/2);
ylin = linspace(min(y),max(y), this.nPoints/2);
[XT,YT] = meshgrid(xlin,ylin);

f = scatteredInterpolant(x,y,surface, 'natural');
ZT = f(XT,YT);

mesh(XT, YT, ZT);
set(gca, 'fontname', 'Bitstream Charter','fontsize', 15);
%title(plotTitle, 'fontsize', 20, 'interpreter', 'tex');


hold on
scatter3(...
    contactLocations(this.nPadding+1:end, 1),...
    contactLocations(this.nPadding+1:end, 2),...
    contactLocations(this.nPadding+1:end, 3) + 2/1000,...
    'fill', 'markerFaceColor', 'blue', 'sizeData', [40]);
hold off
axis('equal');
view(viewPars);

end