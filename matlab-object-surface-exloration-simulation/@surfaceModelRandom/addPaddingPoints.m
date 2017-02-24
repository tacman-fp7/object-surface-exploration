function addPaddingPoints(this, objectSurface)
xlin = linspace(objectSurface.xMin, objectSurface.xMax, this.nPoints);
ylin = linspace(objectSurface.yMin, objectSurface.yMax, this.nPoints);

this.contactLocations = [...
    xlin', repmat(objectSurface.yMin, length(xlin), 1),  repmat(objectSurface.zMin, length(xlin), 1);...
    xlin', repmat(objectSurface.yMax, length(xlin), 1),  repmat(objectSurface.zMin, length(xlin), 1);...
    repmat(objectSurface.xMin, length(ylin) - 2, 1), ylin(2 : end - 1)', repmat(objectSurface.zMin, length(ylin) - 2, 1);...
    repmat(objectSurface.xMax, length(ylin) - 2, 1), ylin(2 : end - 1)', repmat(objectSurface.zMin, length(ylin) - 2, 1)...
    ];

for i = 1:length(this.contactLocations)
    contactIndex = nearestneighbour([this.contactLocations(i, 1) this.contactLocations(i,2)]', objectSurface.objectSurface(:,1:2)');
    this.contactLocations(i,3) = objectSurface.objectSurface(contactIndex, 3);
end

this.objectName = objectSurface.objectName;
this.nPadding = length(this.contactLocations(:,3));

end