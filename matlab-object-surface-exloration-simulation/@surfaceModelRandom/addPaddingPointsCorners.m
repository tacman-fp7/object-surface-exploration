function addPaddingPointsCorners(this, objectSurface)
this.contactLocations = [...
    objectSurface.xMin objectSurface.yMin objectSurface.zMin;...
    objectSurface.xMin objectSurface.yMax objectSurface.zMin;...
    objectSurface.xMax objectSurface.yMin objectSurface.zMin;...
    objectSurface.xMax objectSurface.yMax objectSurface.zMin];
end