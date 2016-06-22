classdef  myObject < handle
    
    properties
        %kdNSearcher;
        xMin;
        xMax;
        yMin;
        yMax;
        zMin;
        zMax;
        objectName;
        objectSurface;
    end
    
    methods
        function initialise(this, objectName, surfacePoints)
            this.objectName = objectName;
            this.objectSurface = surfacePoints;
            %this.kdNSearcher = createns(surfacePoints(:,1:2), 'NSMethod', 'exhaustive', 'distance', 'euclidean');
            this.xMin = min(surfacePoints(:, 1));
            this.xMax = max(surfacePoints(:, 1));
            this.yMin = min(surfacePoints(:, 2));
            this.yMax = max(surfacePoints(:, 2));
            this.zMin = min(surfacePoints(:, 3));
            this.zMax = max(surfacePoints(:, 3));
        end
        
        function contactPoint = sampleObject(this, location)
            %[contactIndex, ~] = knnsearch(this.kdNSearcher, location, 'K', 1);
            contactIndex = nearestneighbour(location', this.objectSurface(:,1:2)'); 
            contactPoint = this.objectSurface(contactIndex,:);
        end
        
        function plotMesh(this, plotPC, figNum)
            plotMesh(this, plotPC, figNum);
        end
    end
end


function contactBins = bin(this, nBins)

binSize = (max(this.objectSurface(:, 3)) - min(this.objectSurface(:, 3))) / (nBins + 1);
contactBins = zeros(length(this.objectSurface(:,3)), nBins) - 1;

for i = 1:length(this.objectSurface(:, 3))
    height = this.objectSurface(i, 3) - min(this.objectSurface(:, 3));
    
    for j = 1:nBins
       if(height < binSize * j)
           contactBins(i, j) = 1;
           break;
       end
    end
end

end


function quantizedSurf = binSurf(this, nBins)
% try 10 bins
binSize = (max(this.objectSurface(:,3)) - min(this.objectSurface(:,3)))/nBins;

quantizedSurf = zeros(length(this.objectSurface(:,3)), 1);
minVal = min(this.objectSurface(:,3));

for i = 1: length(quantizedSurf)
    quantizedSurf(i) = floor((this.objectSurface(i, 3) - minVal )/binSize);
end

end

function quantizedSurf = quantizeSurf(this)

%quantizedSurf = sign(detrend(this.objectSurface(:,3), 'constant'));% - mean(this.objectSurface(:,3)));
quantizedSurf = sign(this.objectSurface(:,3) - ...
    (max(this.objectSurface(:,3)) - min(this.objectSurface(:,3)))/2);

end

function plotMesh(this, plotPC, figNum, nPoints)

if nargin < 4; nPoints = 60; end;
if nargin < 3; figure; else figure(figNum); end;
if nargin < 2; plotPC = false; end;

x = this.objectSurface(:, 1);
y = this.objectSurface(:, 2);
z = this.objectSurface(:, 3);
%z = quantizeSurf(this);
%z = binSurf(this, 5);
%z = bin(this, 1);

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
        this.objectSurface(:, 2),... %this.objectSurface(:, 3),...
        z,...
        'fill', 'markerFaceColor', 'blue', 'sizeData', [90]);
    hold off
end

end
