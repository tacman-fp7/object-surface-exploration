function evaluateRMSE(this)

paddingPoints = getPaddingPoints(this, this.referenceSurface);
%
x = [this.referenceSurface(1:4:end, 1); paddingPoints(:,1)];
y = [this.referenceSurface(1:4:end, 2); paddingPoints(:,2)];



xlin = linspace(min(x),max(x), this.nPoints);
ylin = linspace(min(y),max(y), this.nPoints);
fReference = scatteredInterpolant(x, y, [this.referenceSurface(1:4:end, 3); paddingPoints(:,3)], 'natural'); %; this.contactLocations(1:this.nPadding, 3)
% fEstimated = scatteredInterpolant([this.contactLocations(this.nPadding + 1:end,1); this.cornerPoints(:,1)],...
%     [this.contactLocations(this.nPadding + 1:end,2); this.cornerPoints(:,2)],...
%     [this.contactLocations(this.nPadding + 1:end,3); this.cornerPoints(:,3)], 'natural');

fEstimated = scatteredInterpolant([this.contactLocations(:,1); paddingPoints(:,1)],...
    [this.contactLocations(:,2); paddingPoints(:,2)],...
    [this.contactLocations(:,3); paddingPoints(:,3)], 'natural');

[XT,YT] = meshgrid(xlin,ylin);

referenceSurfaceMesh = fReference(XT, YT);
estimatedSurfaceMesh = fEstimated(XT, YT);


% % figure(11)
% % mesh(XT, YT, referenceSurfaceMesh);
% % 
% % figure(12)
% % mesh(XT, YT, estimatedSurfaceMesh);

refSurface = [reshape(XT, numel(XT), 1),...
    reshape(YT, numel(YT), 1),...
    reshape(referenceSurfaceMesh, numel(referenceSurfaceMesh), 1)];

estimatedSurface = [reshape(XT, numel(XT), 1), ...
    reshape(YT, numel(YT), 1), reshape(estimatedSurfaceMesh, numel(estimatedSurfaceMesh), 1)];

this.surfaceRMSE = [this.surfaceRMSE; sqrt( sum( power(refSurface(:,3) - estimatedSurface(:,3), 2) ) / numel( refSurface(:,3) ))];

end

function paddingPoints = getPaddingPoints(this, contactPoints)


xlin = linspace(min(contactPoints(:,1)), max(contactPoints(:,1)), this.nPoints);
ylin = linspace(min(contactPoints(:,2)), max(contactPoints(:,2)), this.nPoints);

paddingPoints = [...
    xlin', repmat(min(contactPoints(:,2)), length(xlin), 1),  zeros(length(xlin), 1);...
    xlin', repmat(max(contactPoints(:,2)), length(xlin), 1),  zeros(length(xlin), 1);...
    repmat(min(contactPoints(:,1)), length(ylin) - 2, 1), ylin(2 : end - 1)', zeros(length(ylin) - 2, 1);...
    repmat(max(contactPoints(:,1)), length(ylin) - 2, 1), ylin(2 : end - 1)', zeros(length(ylin) - 2, 1)...
    ];

for i = 1:length(paddingPoints)
    contactIndex = nearestneighbour([paddingPoints(i, 1) paddingPoints(i,2)]', contactPoints(:,1:2)');
    paddingPoints(i,3) = contactPoints(contactIndex, 3);
end


end
