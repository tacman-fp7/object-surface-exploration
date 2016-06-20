function evaluateRMSE(this)

%
x = this.referenceSurface(:, 1);
y = this.referenceSurface(:, 2);


xlin = linspace(min(x),max(x), this.nPoints);
ylin = linspace(min(y),max(y), this.nPoints);
fReference = scatteredInterpolant(x, y, this.referenceSurface(:, 3), 'natural');
fEstimated = scatteredInterpolant([this.contactLocations(this.nPadding:end,1); this.cornerPoints(:,1)],...
    [this.contactLocations(this.nPadding:end,2); this.cornerPoints(:,2)],...
    [this.contactLocations(this.nPadding:end,3); this.cornerPoints(:,3)], 'natural');

[XT,YT] = meshgrid(xlin,ylin);
referenceSurfaceMesh = fReference(XT, YT);
estimatedSurfaceMesh = fEstimated(XT, YT);

refSurface = [reshape(XT, numel(XT), 1),...
    reshape(YT, numel(YT), 1),...
    reshape(referenceSurfaceMesh, numel(referenceSurfaceMesh), 1)];

estimatedSurface = [reshape(XT, numel(XT), 1), ...
    reshape(YT, numel(YT), 1), reshape(estimatedSurfaceMesh, numel(estimatedSurfaceMesh), 1)];

this.surfaceRMSE = [this.surfaceRMSE; sqrt( sum( power(refSurface(:,3) - estimatedSurface(:,3), 2) ) / numel( refSurface(:,3) ))];

end