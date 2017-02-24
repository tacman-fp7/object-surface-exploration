function evaluateRMSE(this)

% paddingPoints = getPaddingPoints(this, this.referenceSurface);
% %
% x = [this.referenceSurface(1:4:end, 1); paddingPoints(:,1)];
% y = [this.referenceSurface(1:4:end, 2); paddingPoints(:,2)];
% 
% 
% 
% xlin = linspace(min(x),max(x), this.nPoints);
% ylin = linspace(min(y),max(y), this.nPoints);
% fReference = scatteredInterpolant(x, y, [this.referenceSurface(1:4:end, 3); paddingPoints(:,3)], 'natural'); %; this.contactLocations(1:this.nPadding, 3)
% % fEstimated = scatteredInterpolant([this.contactLocations(this.nPadding + 1:end,1); this.cornerPoints(:,1)],...
% %     [this.contactLocations(this.nPadding + 1:end,2); this.cornerPoints(:,2)],...
% %     [this.contactLocations(this.nPadding + 1:end,3); this.cornerPoints(:,3)], 'natural');
% 
fEstimated = scatteredInterpolant([this.contactLocations(:,1); this.refSurf.paddingPoints(:,1)],...
    [this.contactLocations(:,2); this.refSurf.paddingPoints(:,2)],...
    [this.contactLocations(:,3); this.refSurf.paddingPoints(:,3)], 'natural');

% [XT,YT] = meshgrid(xlin,ylin);
% 
% referenceSurfaceMesh = fReference(XT, YT);

referenceSurfaceMesh = this.refSurf.referenceSurfaceMesh;
estimatedSurfaceMesh = fEstimated(this.refSurf.XT, this.refSurf.YT);


% % figure(11)
% % mesh(this.refSurf.XT, this.refSurf.YT, this.refSurf.referenceSurfaceMesh);
% % 
% % figure(12)
% % mesh(this.refSurf.XT, this.refSurf.YT, estimatedSurfaceMesh);


% % refSurface = [reshape(XT, numel(XT), 1),...
% %     reshape(YT, numel(YT), 1),...
% %     reshape(referenceSurfaceMesh, numel(referenceSurfaceMesh), 1)];
% % 
% % estimatedSurface = [reshape(XT, numel(XT), 1), ...
% %     reshape(YT, numel(YT), 1), reshape(estimatedSurfaceMesh, numel(estimatedSurfaceMesh), 1)];
this.refSurf.estimatedSurfaceMesh = estimatedSurfaceMesh;

this.surfaceRMSE = [this.surfaceRMSE; sqrt( sum(sum( (referenceSurfaceMesh - estimatedSurfaceMesh).^2) ) / numel( referenceSurfaceMesh ))];

end

