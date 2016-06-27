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
        
        function expandObject(this)
            expandObject(this);
        end
    end
end




function expandObject(this)

nPoints = 10;
paddingPoints = [];

for expansion = linspace(0, 0.019, 15)
    xMin = min(this.objectSurface(:, 1)) - expansion;
    xMax = max(this.objectSurface(:, 1)) + expansion;
    yMin = min(this.objectSurface(:, 2)) - expansion;
    yMax = max(this.objectSurface(:, 2)) + expansion;
    a = sort(this.objectSurface(:, 3));
    zMin = median(a(1:31));
    
    xlin = linspace(xMin, xMax, nPoints);
    ylin = linspace(yMin, yMax, nPoints);
    
    paddingPoints = [paddingPoints;...
        xlin', repmat(yMin, length(xlin), 1),  repmat(zMin, length(xlin), 1);...
        xlin', repmat(yMax, length(xlin), 1),  repmat(zMin, length(xlin), 1);...
        repmat(xMin, length(ylin) - 2, 1), ylin(2 : end - 1)', repmat(zMin, length(ylin) - 2, 1);...
        repmat(xMax, length(ylin) - 2, 1), ylin(2 : end - 1)', repmat(zMin, length(ylin) - 2, 1)...
        ];
    
end

for i = 1:length(paddingPoints)
    location = [paddingPoints(i, 1)+rand*expansion/2, paddingPoints(i,2) + rand * expansion];
    %contactIndex = nearestneighbour(location', this.objectSurface(:,1:2)', 'NumberOfNeighbours', 3);
    %paddingPoints(i,3) = median(this.objectSurface(contactIndex, 3));
    paddingPoints(i,1:2) = location;
end


this.objectSurface = [this.objectSurface; paddingPoints];

this.xMin = min(this.objectSurface(:, 1));
this.xMax = max(this.objectSurface(:, 1));
this.yMin = min(this.objectSurface(:, 2));
this.yMax = max(this.objectSurface(:, 2));
this.zMin = min(this.objectSurface(:, 3));
this.zMax = max(this.objectSurface(:, 3));

% % nPoints = 60;
% % x = this.objectSurface(:, 1);
% % y = this.objectSurface(:, 2);
% % z = this.objectSurface(:, 3);
% %
% % xlin = linspace(min(x),max(x), nPoints);
% % ylin = linspace(min(y),max(y), nPoints);
% % [XT,YT] = meshgrid(xlin,ylin);
% %
% % f = scatteredInterpolant(x,y,z, 'natural');
% % ZT = f(XT,YT);
% %
% %
% % mesh(XT, YT, ZT);
% % title('Object Mesh', 'fontsize', 24);
% %
% % hold on
% % scatter3(...
% %     this.objectSurface(:, 1),...
% %     this.objectSurface(:, 2),... %this.objectSurface(:, 3),...
% %     z,...
% %     'fill', 'markerFaceColor', 'blue', 'sizeData', [90]);
% % hold off


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
        this.objectSurface(:, 2),... %this.objectSurface(:, 3),...
        z,...
        'fill', 'markerFaceColor', 'blue', 'sizeData', [90]);
    hold off
end

end
