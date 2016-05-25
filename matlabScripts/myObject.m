classdef  myObject < handle
    
    properties
        kdNSearcher;
        xMin;
        xMax;
        yMin;
        yMax;
        zMin;
        objectName;
        objectSurface;
    end
    
    methods
        function initialise(this, objectName, surfacePoints)
            this.objectName = objectName;
            this.objectSurface = surfacePoints;
            this.kdNSearcher = createns(surfacePoints(:,1:2), 'NSMethod', 'exhaustive', 'distance', 'euclidean');
            this.xMin = min(surfacePoints(:, 1));
            this.xMax = max(surfacePoints(:, 1));
            this.yMin = min(surfacePoints(:, 2));
            this.yMax = max(surfacePoints(:, 2));
            this.zMin = min(surfacePoints(:, 3));
        end
        
        function contactPoint = sampleObject(this, location)
            [contactIndex, ~] = knnsearch(this.kdNSearcher, location, 'K', 1);
            contactPoint = this.objectSurface(contactIndex,:);
        end
        
        function plotMesh(this)
            plotMesh(this);
        end
    end
end

function plotMesh(this)

figure;
x = this.objectSurface(:, 1);
y = this.objectSurface(:, 2);
z = this.objectSurface(:, 3);


xlin = linspace(min(x),max(x),60);
ylin = linspace(min(y),max(y),60);
[XT,YT] = meshgrid(xlin,ylin);

f = scatteredInterpolant(x,y,z, 'natural');
ZT = f(XT,YT);


mesh(XT, YT, ZT);

hold on
scatter3(...
    this.objectSurface(:, 1),...
    this.objectSurface(:, 2),...
    this.objectSurface(:, 3),...
    'fill', 'markerFaceColor', 'blue', 'sizeData', [90]);
hold off
end
