classdef activeSurfaceModelGP < handle
    
    properties
        contactLocations;
        gpModel;
        nPoints = 60;
        objectName;
        nextSamplingLocation = zeros(1,2);
        inputTesting;
        outputTesting;
        nBins = 1;
        firstBinThreshold = 20;
    end
    
    methods
        function nextSamplingLocation = getNextSamplingPoint(this)
            nextSamplingLocation = this.nextSamplingLocation;
        end
        
        function addContactLocation(this, contactLocation)
            this.contactLocations = [this.contactLocations; contactLocation];
            if(length(this.contactLocations) > (this.firstBinThreshold + (this.nPoints * 4 - 2)))
                this.nBins = 10;
            end
            updateModel(this);
        end
        
        function initialiseLimitted(this, objectSurface)
            this.contactLocations = [...
                objectSurface.xMin objectSurface.yMin objectSurface.zMin;...
                objectSurface.xMin objectSurface.yMax objectSurface.zMin;...
                objectSurface.xMax objectSurface.yMin objectSurface.zMin;...
                objectSurface.xMax objectSurface.yMax objectSurface.zMin];
            this.objectName = objectSurface.objectName;
            %createGPModel(this);
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
    'rls:dual', 'predkernel:traintest', 'pred:dual',...
    'perf:macroavg', 'perf:precrec', 'conf:boltzman'};

this.gpModel.process{1} = [2,2,2,2,0,0,0,0,0];
this.gpModel.process{2} = [3,3,3,3,2,2,2,2,2];
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
for bin = 1: this.nBins
    surfUncertainty(:, bin) = 1 - ((surfUncertainty(:, bin) - min(surfUncertainty(:, bin))) ./...
        (max(surfUncertainty(:, bin)) - min(surfUncertainty(:, bin))));
end
% Regression
% %gpUncertainty = runGURLSRegression(this);



gpUncertainty = zeros(length(surfUncertainty),1); % Disable the regression part

alphaE = 0;

complexUncertainty = zeros(length(surfUncertainty), 1);
for bin = 1:this.nBins
    complexUncertainty = complexUncertainty + (gpUncertainty * alphaE + surfUncertainty(:,bin) * (1 - alphaE));
end

complexUncertainty = complexUncertainty/this.nBins;

[~, next_idx] = max(complexUncertainty);


this.nextSamplingLocation = [this.inputTesting(next_idx, 1), this.inputTesting(next_idx, 2)];



% % Display
figNum = 2;
plotMesh(this, this.contactLocations, true, figNum, this.nPoints, 'Contact Locations');
figNum = figNum + 1;
plotNextLocation(this, max(this.contactLocations(:,3)), figNum -1);




% % for bin = 1:this.nBins
% %     
% %     plotMesh(this, [this.inputTesting surfUncertainty(:,bin)], false, figNum, this.nPoints, sprintf('Surface uncertainty: bin %02d', bin));
% %     figNum = figNum + 1;
% %     plotNextLocation(this, max(abs(surfUncertainty(:, bin))), figNum -1);
% %     
% % end

% % for bin = 1:this.nBins
% %     plotMesh(this, [this.inputTesting surfaceModel(:, bin)./abs(surfaceModel(:, bin))], false, figNum, this.nPoints, sprintf('Predicted Surface: %02d', bin));
% %     figNum = figNum + 1;
% %     
% % end

plotMesh(this, [this.inputTesting superimposeSurface(this, surfaceModel)], false, figNum, this.nPoints, 'Superimposed Surface');
figNum = figNum + 1;
plotNextLocation(this, max(superimposeSurface(this, surfaceModel)), figNum -1);

plotMesh(this, [this.inputTesting, complexUncertainty ], false, figNum, this.nPoints, 'GP Vars');
figNum = figNum + 1;
plotNextLocation(this, max(complexUncertainty), figNum -1);

figure(figNum);
hist(this.contactLocations((this.nPoints * 4 - 2):end,3), this.nBins);

end

function surf = superimposeSurface(this, surface)

for bin = 1:this.nBins;
    
    surface(:, bin) = surface(:, bin)./abs(surface(:, bin));
    
end;

if (this.nBins == 1)
    surf = surface;
    surf(surf == -1) = 0;
    return;
end

surf = zeros(length(surface), 1);

for i = 1:length(surf)
    for bin = this.nBins:-1:1
        if(surface(i,bin) == 1)
            surf(i) = bin;
            break;
        end
    end
end

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


function contactBins = binContacts(this)

nBins = this.nBins;

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

% Check that each bin has at least one positive and one negative value
% GURLS fails if there is none. They also have to be mutually exclusive

for j = 1:nBins
    
    if(sum(ismember(contactBins(:, j), 1)) == 0)
        contactBins(j,j) = 1; % Changing diagonally to make sure they are mutually exclusive
        
    end
end

if(nBins == 1)
    contactBins = contactBins * -1; % for asthetics
end

end

function surfaceModel = runGURLS(this)


createGPModel(this);
jobID = 1;
target = binContacts(this);

gurls(this.contactLocations( : , 1:2),...
    target, this.gpModel, jobID); % <-----------

%%% Testing

gurls(this.inputTesting, repmat(this.outputTesting, 1, this.nBins) , this.gpModel, 2);
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
        contactLocations((this.nPoints * 4 - 2):end, 1),...
        contactLocations((this.nPoints * 4 - 2):end, 2),...
        contactLocations((this.nPoints * 4 - 2):end, 3),...
        'fill', 'markerFaceColor', 'blue', 'sizeData', [90]);
    hold off
end

view([az, el]);
end
