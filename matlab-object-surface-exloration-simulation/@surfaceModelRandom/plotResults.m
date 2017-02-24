function figNum = plotResults(this)

figNum = 2;
plotMesh(this, this.contactLocations, true, figNum, this.nPoints, 'Contact Locations');
figNum = figNum + 1;
plotNextLocation(this, max(this.contactLocations(:,3)), figNum -1);

figure(figNum)
plot(this.surfaceRMSE);
figNum = figNum + 1;

end

