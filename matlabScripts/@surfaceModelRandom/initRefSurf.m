function initRefSurf(this)
paddingPoints = getPaddingPoints(this, this.referenceSurface);
%
x = [this.referenceSurface(1:4:end, 1); paddingPoints(:,1)];
y = [this.referenceSurface(1:4:end, 2); paddingPoints(:,2)];



xlin = linspace(min(x),max(x), this.nPoints);
ylin = linspace(min(y),max(y), this.nPoints);
fReference = scatteredInterpolant(x, y, [this.referenceSurface(1:4:end, 3); paddingPoints(:,3)], 'natural'); %; this.contactLocations(1:this.nPadding, 3)


[XT,YT] = meshgrid(xlin,ylin);

this.refSurf.referenceSurfaceMesh = fReference(XT, YT);
this.refSurf.XT = XT;
this.refSurf.YT = YT;
this.refSurf.paddingPoints = paddingPoints;
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
