function generateCornerPoints(this, objectSurface)
% Add four corner points for mesh creation

%kdNSearcher = createns(objectSurface.objectSurface(:,1:2), 'NSMethod', 'exhaustive', 'distance', 'euclidean');
%[contactIndex, ~] = knnsearch(kdNSearcher,[objectSurface.xMin objectSurface.yMin], 'K', 1);
contactIndex = nearestneighbour([objectSurface.xMin objectSurface.yMin]', objectSurface.objectSurface(:,1:2)');
this.cornerPoints = [this.cornerPoints;...
    [objectSurface.xMin objectSurface.yMin objectSurface.objectSurface(contactIndex, 3)]];

%[contactIndex, ~] = knnsearch(kdNSearcher,[objectSurface.xMin objectSurface.yMax], 'K', 1);
contactIndex = nearestneighbour([objectSurface.xMin objectSurface.yMax]', objectSurface.objectSurface(:,1:2)');
this.cornerPoints = [this.cornerPoints;...
    [objectSurface.xMin objectSurface.yMax objectSurface.objectSurface(contactIndex, 3)]];

%[contactIndex, ~] = knnsearch(kdNSearcher,[objectSurface.xMax objectSurface.yMin], 'K', 1);
contactIndex = nearestneighbour([objectSurface.xMax objectSurface.yMin]', objectSurface.objectSurface(:,1:2)');
this.cornerPoints = [this.cornerPoints;...
    [objectSurface.xMax objectSurface.yMin objectSurface.objectSurface(contactIndex, 3)]];

%[contactIndex, ~] = knnsearch(kdNSearcher,[objectSurface.xMax objectSurface.yMax], 'K', 1);
contactIndex = nearestneighbour([objectSurface.xMax objectSurface.yMax]', objectSurface.objectSurface(:,1:2)');
this.cornerPoints = [this.cornerPoints;...
    [objectSurface.xMax objectSurface.yMax objectSurface.objectSurface(contactIndex, 3)]];

end