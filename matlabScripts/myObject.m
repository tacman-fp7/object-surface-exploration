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
        
        function plotMesh(this, plotPC, figNum)
            plotMesh(this, plotPC, figNum);
        end
    end
end

function plotMesh(this, plotPC, figNum, nPoints)

if nargin < 4; nPoints = 60; end;
if nargin < 3; figure; else figure(figNum); end;
if nargin < 2; plotPC = false; end;

x = this.objectSurface(:, 1);
y = this.objectSurface(:, 2);
z = this.objectSurface(:, 3);


xlin = linspace(min(x),max(x), nPoints);
ylin = linspace(min(y),max(y), nPoints);
[XT,YT] = meshgrid(xlin,ylin);

f = scatteredInterpolant(x,y,z, 'natural');
ZT = f(XT,YT);


mesh(XT, YT, ZT);
title('Object Mesh', 'fontsize', 24);
if(plotPC)
    hold on
    scatter3(...
        this.objectSurface(:, 1),...
        this.objectSurface(:, 2),...
        this.objectSurface(:, 3),...
        'fill', 'markerFaceColor', 'blue', 'sizeData', [90]);
    hold off
end

end
