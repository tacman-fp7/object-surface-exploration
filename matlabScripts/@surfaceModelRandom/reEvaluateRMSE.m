function reEvaluateRMSE(this)

%
this.surfaceRMSE = [];
for i = this.nPadding + 1: length(this.contactLocations)
    x = this.referenceSurface(1:4:end, 1);
    y = this.referenceSurface(1:4:end, 2);
    
    
    xlin = linspace(min(x),max(x), this.nPoints);
    ylin = linspace(min(y),max(y), this.nPoints);
    fReference = scatteredInterpolant(x, y, this.referenceSurface(1:4:end, 3), 'natural');
    fEstimated = scatteredInterpolant([this.contactLocations(this.nPadding + 1:i, 1); this.cornerPoints(:,1)],...
        [this.contactLocations(this.nPadding + 1:i, 2); this.cornerPoints(:,2)],...
        [this.contactLocations(this.nPadding + 1:i, 3); this.cornerPoints(:,3)], 'natural');
    
    [XT,YT] = meshgrid(xlin,ylin);
    
    referenceSurfaceMesh = fReference(XT, YT);
    estimatedSurfaceMesh = fEstimated(XT, YT);
    
% %     lowerQuartile = min(this.referenceSurface(:, 3)) + 0.3 * (max(this.referenceSurface(:, 3)) - min(this.referenceSurface(:, 3)));
% %     
% %     referenceSurfaceMesh(referenceSurfaceMesh < lowerQuartile) = 0;
% %     estimatedSurfaceMesh(referenceSurfaceMesh < lowerQuartile) = 0;
    

    figure(11)
    mesh(XT, YT, referenceSurfaceMesh);
    figure(12)
    mesh(XT, YT, estimatedSurfaceMesh);
    
    refSurface = [reshape(XT, numel(XT), 1),...
        reshape(YT, numel(YT), 1),...
        reshape(referenceSurfaceMesh, numel(referenceSurfaceMesh), 1)];
    
    estimatedSurface = [reshape(XT, numel(XT), 1), ...
        reshape(YT, numel(YT), 1), reshape(estimatedSurfaceMesh, numel(estimatedSurfaceMesh), 1)];
    
    this.surfaceRMSE = [this.surfaceRMSE; sqrt( sum( power(refSurface(:,3) - estimatedSurface(:,3), 2) ) / numel( refSurface(:,3) ))];
    
end

end