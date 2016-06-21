function plotMeshAll(this, contactLocations, plotTitle, viewPars)





x = contactLocations(:, 1);
y = contactLocations(:, 2);
surface = contactLocations(:, 3);


xlin = linspace(min(x),max(x), this.nPoints);
ylin = linspace(min(y),max(y), this.nPoints);
[XT,YT] = meshgrid(xlin,ylin);

f = scatteredInterpolant(x,y,surface, 'natural');
ZT = f(XT,YT);

mesh(XT, YT, ZT);
set(gca, 'fontname', 'Bitstream Charter','fontsize', 15);
%xlabel('y-axis [m])', 'fontsize', 25, 'interpreter', 'tex');
%xlabel('x-axis [m])', 'fontsize', 25, 'interpreter', 'tex');
title(plotTitle, 'fontsize', 20, 'interpreter', 'tex');


% % hold on
% % scatter3(...
% %     contactLocations(:, 1),...
% %     contactLocations(:, 2),...
% %     contactLocations(:, 3),...
% %     'fill', 'markerFaceColor', 'blue', 'sizeData', [90]);
% % hold off
axis('equal');
view(viewPars);

end