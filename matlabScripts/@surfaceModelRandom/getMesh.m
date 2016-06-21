function getMesh(this, nContacts, plotTitle, viewPars)

if nargin < 3; plotTitle = ''; end;





x = this.contactLocations(1:this.nPadding + nContacts, 1);
y = this.contactLocations(1:this.nPadding + nContacts, 2);
surface = this.contactLocations(1:this.nPadding + nContacts, 3);


xlin = linspace(min(x),max(x), this.nPoints);
ylin = linspace(min(y),max(y), this.nPoints);
[XT,YT] = meshgrid(xlin,ylin);

f = scatteredInterpolant(x,y,surface, 'natural');
ZT = f(XT,YT);

mesh(XT, YT, ZT);
set(gca, 'fontname', 'Bitstream Charter','fontsize', 15);
title(plotTitle, 'fontsize', 20, 'interpreter', 'tex');

    hold on
    scatter3(...
        this.contactLocations(this.nPadding + 1:this.nPadding + nContacts, 1),...
        this.contactLocations(this.nPadding + 1:this.nPadding + nContacts, 2),...
        this.contactLocations(this.nPadding + 1:this.nPadding + nContacts, 3),...
        'fill', 'markerFaceColor', 'blue', 'sizeData', [90]);
    hold off

    zlim([min(this.contactLocations(:,3)) max(this.contactLocations(:,3))]);
axis('equal');
view(viewPars);
end