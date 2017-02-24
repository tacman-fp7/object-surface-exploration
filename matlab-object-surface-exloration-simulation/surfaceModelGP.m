classdef surfaceModelGP < handle
    
    properties
        contactLocations;
        gpModel;
        nPoints = 60;
        objectName;
        nextSamplingLocation = zeros(1,2);
        inputTesting;
        outputTesting;
    end
    
    methods
        function nextSamplingLocation = getNextSamplingPoint(this)
            nextSamplingLocation = this.nextSamplingLocation;
        end
        
        function addContactLocation(this, contactLocation)
            this.contactLocations = [this.contactLocations; contactLocation];
            updateModel(this);
        end
        
        function initialiseLimitted(this, objectSurface)
            this.contactLocations = [...
                objectSurface.xMin objectSurface.yMin objectSurface.zMin;...
                objectSurface.xMin objectSurface.yMax objectSurface.zMin;...
                objectSurface.xMax objectSurface.yMin objectSurface.zMin;...
                objectSurface.xMax objectSurface.yMax objectSurface.zMin];
            this.objectName = objectSurface.objectName;
            createGPModel(this);
            updateModel(this);
        end
        function initialise(this, objectSurface)
            xlin = linspace(objectSurface.xMin, objectSurface.xMax, this.nPoints);
            ylin = linspace(objectSurface.yMin, objectSurface.yMax, this.nPoints);
            
            this.contactLocations = [...
                xlin', repmat(objectSurface.yMin, length(xlin), 1),  repmat(objectSurface.zMin, length(xlin), 1);...
                xlin', repmat(objectSurface.yMax, length(xlin), 1),  repmat(objectSurface.zMin, length(xlin), 1);...
                repmat(objectSurface.xMin, length(ylin) - 2, 1), ylin(2 : end - 1)', repmat(objectSurface.zMin, length(ylin) - 2, 1);...
                repmat(objectSurface.xMax, length(ylin) - 2, 1), ylin(2 : end - 1)', repmat(objectSurface.zMin, length(ylin) - 2, 1)...
                ];
            
            this.objectName = objectSurface.objectName;
            updateModel(this);
            
        end
        
        function plotMesh(this, plotPC, figNum)
            plotMesh(this, plotPC, figNum);
        end
    end
end


function createGPModel(this)

name = this.objectName;
this.gpModel = gurls_defopt(name);
this.gpModel.seq = {'split:ho', 'paramsel:siglamhogpregr', 'kernel:rbf',...
    'rls:gpregr', 'predkernel:traintest', 'pred:gpregr'};
this.gpModel.process{1} = [2,2,2,2,1,1];
this.gpModel.process{2} = [3,3,3,3,2,2];
this.gpModel.epochs = 100000;
this.gpModel.hoperf = @perf_abserr;
this.gpModel.save = -1;

this.gpModel.nholdouts = 4;
this.gpModel.hoproportion = 0.1;


xlin = linspace(min(this.contactLocations( : , 1)), max(this.contactLocations( : , 1)), this.nPoints);
ylin = linspace(min(this.contactLocations( : , 2)), max(this.contactLocations( : , 2)), this.nPoints);

this.inputTesting = [];
for i = 1:length(xlin)
    this.inputTesting =  [this.inputTesting; repmat(xlin(i), length(ylin), 1) ylin'];
end

this.outputTesting = zeros(size(this.inputTesting, 1),1); % dummy to help me evalate

end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%          update the model         %%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function updateModel(this)

% % Display the 
figNum = 1;

plotMesh(this, this.contactLocations, true, figNum, this.nPoints, 'Contact Locations');
figNum = figNum + 1;

% Convert grad points to surface
gradSurface = normalise(getContactMeshGradient(this));

plotMesh(this, [this.inputTesting, gradSurface], false, figNum, this.nPoints, 'Contact Gradients');
figNum = figNum + 1;

[surfaceUncertainty, surfaceModel] = runGURLS(this);

surfaceUncertainty = normalise(surfaceUncertainty);
plotMesh(this, [this.inputTesting, surfaceUncertainty], false, figNum, this.nPoints, 'Surface uncertaintly');
figNum = figNum +1;

complexUncertainty = surfaceUncertainty + gradSurface;
plotMesh(this, [this.inputTesting, complexUncertainty], false, figNum, this.nPoints, 'Surface uncertaintly * gradient');
figNum = figNum +1;


[~, b] = max(complexUncertainty);


this.nextSamplingLocation = [this.inputTesting(b,1), this.inputTesting(b,2)];


end

function normalised = normalise(data)
normalised = (data(:, end) - min(data(:,end)))/...
    (max(data(:,end)) - min(data(:, end)));
end
function gradSurface = getContactMeshGradient(this)

f = scatteredInterpolant(this.contactLocations( : , 1),this.contactLocations( : , 2), this.contactLocations( : , 3), 'natural');
gradSurface = abs(gradient( f(this.inputTesting(:,1), this.inputTesting(:,2))));



end


function [surfaceUncertainty, surfaceModel] = runGURLS(this)

createGPModel(this);
jobID = 1;
gurls(this.contactLocations( : , 1:2),...
      this.contactLocations( : , 3), this.gpModel, jobID);

%%% Testing

gurls(this.inputTesting, this.outputTesting , this.gpModel, 2);
surfaceUncertainty = this.gpModel.pred.vars;
surfaceModel = this.gpModel.pred.means;

end










function plotMesh(this, contactLocations, plotPC, figNum, nPoints, plotTitle)

if nargin < 6; plotTitle = ''; end;
if nargin < 5; nPoints = this.nPoints; end;
if nargin < 4; figure; else figure(figNum); end;
if nargin < 3; plotPC = false; end;

[az el] = view();
x = contactLocations(:, 1);
y = contactLocations(:, 2);
surface = contactLocations(:, 3);


xlin = linspace(min(x),max(x), nPoints);
ylin = linspace(min(y),max(y), nPoints);
[XT,YT] = meshgrid(xlin,ylin);

f = scatteredInterpolant(x,y,surface, 'natural');
ZT = f(XT,YT);

mesh(XT, YT, ZT);
title(plotTitle, 'fontsize', 24);

if(plotPC)
    hold on
    scatter3(...
        contactLocations(:, 1),...
        contactLocations(:, 2),...
        contactLocations(:, 3),...
        'fill', 'markerFaceColor', 'blue', 'sizeData', [90]);
    hold off
end

view([az, el]);
end
