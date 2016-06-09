classdef activeSurfaceModelGP < handle
    
    properties
        contactLocations;
        gpModel;
        nPoints = 20;
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
            %plotMesh(this, plotPC, figNum);
            plotMesh(this, this.contactLocations, plotPC, figNum, this.nPoints, 'Contact Locations');
        end
    end
end


function createGPModel(this)

name = this.objectName;
this.gpModel = gurls_defopt(name);
this.gpModel.seq = {'split:ho', 'paramsel:siglamho', 'kernel:rbf',...
    'rls:dual', 'predkernel:traintest', 'pred:dual'};
this.gpModel.process{1} = [2,2,2,2,1,1];
this.gpModel.process{2} = [3,3,3,3,2,2];
this.gpModel.epochs = 100;
this.gpModel.hoperf = @perf_abserr;
this.gpModel.save = -1;

this.gpModel.nholdouts = 1;
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

% Classification
surfaceModel = runGURLS(this);
surfUncertainty = abs(surfaceModel);
surfUncertainty = 1 - ((surfUncertainty - min(surfUncertainty)) ./ (max(surfUncertainty) - min(surfUncertainty)));

% Regression
gpUncertainty = runGURLSRegression(this);

complexUncertainty = (gpUncertainty * 0.25 + surfUncertainty * 0.75);


[~, next_idx] = max(complexUncertainty);


this.nextSamplingLocation = [this.inputTesting(next_idx, 1), this.inputTesting(next_idx, 2)];


% % Display 
figNum = 1;
plotMesh(this, this.contactLocations, true, figNum, this.nPoints, 'Contact Locations');
figNum = figNum + 1;
plotNextLocation(this, max(this.contactLocations(:,3)), figNum -1);

% % plotMesh(this, [this.contactLocations(:,1) this.contactLocations(:,2) quantizeContacts(this.contactLocations(:,3))],...
% %     true, figNum, this.nPoints, 'Quantized');
% % figNum = figNum + 1;



plotMesh(this, [this.inputTesting surfUncertainty], false, figNum, this.nPoints, 'Surface uncertainty');
figNum = figNum + 1;
plotNextLocation(this, max(abs(surfUncertainty)), figNum -1);


plotMesh(this, [this.inputTesting surfaceModel./abs(surfaceModel)], false, figNum, this.nPoints, 'Predicted Surface');
figNum = figNum + 1;



plotMesh(this, [this.inputTesting, complexUncertainty ], false, figNum, this.nPoints, 'GP Vars');
figNum = figNum + 1;
plotNextLocation(this, max(complexUncertainty), figNum -1);

end

function gpUncertainty = runGURLSRegression(this, nBins)

% quick test
name = 'test';
gpModel = gurls_defopt(name);
gpModel.seq = {'split:ho', 'paramsel:siglamhogpregr', 'kernel:rbf',...
    'rls:gpregr', 'predkernel:traintest', 'pred:gpregr'};
gpModel.process{1} = [2,2,2,2,1,1];
gpModel.process{2} = [3,3,3,3,2,2];
gpModel.epochs = 100000;
gpModel.hoperf = @perf_abserr;
gpModel.save = -1;

gpModel.nholdouts = 4;
gpModel.hoproportion = 0.1;


jobID = 1;
gurls(this.contactLocations( : , 1:2),...
      this.contactLocations(:, 3), gpModel, jobID);
  

gurls(this.inputTesting, this.outputTesting , gpModel, 2);


gpUncertainty = gpModel.pred.vars;
gpUncertainty = (gpUncertainty - min(gpUncertainty)) ./ (max(gpUncertainty) - min(gpUncertainty));

end

function plotNextLocation(this, zPos, figNum)

figure(figNum);
hold on;
scatter3(this.nextSamplingLocation(1), this.nextSamplingLocation(2), zPos, ...
    'fill', 'markerFaceColor', 'red', 'sizeData', [100]);
hold off;

end

function contacts = quantizeContacts(contacts)
 
contacts = sign(contacts - (max(contacts) + min(contacts))/2);
%contacts = sign(contacts - median(contacts));
end


function normalised = normalise(data)
normalised = (data(:, end) - min(data(:,end)))/...
    (max(data(:,end)) - min(data(:, end)));
end

function gradSurface = getContactMeshGradient(this)

f = scatteredInterpolant(this.contactLocations( : , 1),this.contactLocations( : , 2), this.contactLocations( : , 3), 'natural');
gradSurface = abs(gradient( f(this.inputTesting(:,1), this.inputTesting(:,2))));



end


function contactBins = bin(this, nBins)

binSize = (max(this.contactLocations(:, 3)) - min(this.contactLocations(:, 3))) / (nBins + 1);
contactBins = zeros(length(this.contactLocations(:,3)), nBins) - 1;

for i = 1:length(this.contactLocations(:, 3))
    height = this.contactLocations(i, 3) - min(this.contactLocations(:, 3));
    
    for j = 1:nBins
       if(height < binSize * j)
           contactBins(i, j) = 1;
           break;
       end
    end
end

end

function surfaceModel = runGURLS(this)

nBins = 2;
createGPModel(this);
jobID = 1;
gurls(this.contactLocations( : , 1:2),...
      quantizeContacts(this.contactLocations(:, 3)), this.gpModel, jobID); % <-----------

%%% Testing

gurls(this.inputTesting, this.outputTesting , this.gpModel, 2);
surfaceModel = this.gpModel.pred;

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
