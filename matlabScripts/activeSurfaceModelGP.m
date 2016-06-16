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
        firstBinThreshold = 10;
        objectModel_CAD; % object model from cad design
        objectModel_exp; % object model from exploration data
        exp_additionalContacts;
        %referenceSurface;
        Ricp; % ICP rotation
        Ticp; % ICP translation
        surfaceError;
        surfaceError_exp;
        nPadding = 0;
    end
    
    methods
        function nextSamplingLocation = getNextSamplingPoint(this)
            nextSamplingLocation = this.nextSamplingLocation;
        end
        
        function addContactLocation(this, contactLocation)
           
            
            this.contactLocations = [this.contactLocations; contactLocation];
            
             evaluate_objectModelExp(this);
             evaluate_objectModelCAD(this);
            if(length(this.contactLocations) > (this.firstBinThreshold * 5 + (this.nPoints * 4 - 2)))
                this.nBins = 12;
            elseif(length(this.contactLocations) > (this.firstBinThreshold * 4 + (this.nPoints * 4 - 2)))
                this.nBins = 10;
            elseif(length(this.contactLocations) > (this.firstBinThreshold * 3 + (this.nPoints * 4 - 2)))
                this.nBins = 8;
            elseif(length(this.contactLocations) > (this.firstBinThreshold * 2 + (this.nPoints * 4 - 2)))
                this.nBins = 4;
            elseif(length(this.contactLocations) > (this.firstBinThreshold + (this.nPoints * 4 - 2)))
                this.nBins = 2;
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
            this.nPadding = length(this.contactLoctions(:,3));
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
            this.nPadding = length(this.contactLocations(:,3));
            
            % Add four corner points for mesh creation
            
            kdNSearcher = createns(objectSurface.objectSurface(:,1:2), 'NSMethod', 'exhaustive', 'distance', 'euclidean');
            [contactIndex, ~] = knnsearch(kdNSearcher,[objectSurface.xMin objectSurface.yMin], 'K', 1);
            this.exp_additionalContacts = [this.exp_additionalContacts;...
                [objectSurface.xMin objectSurface.yMin objectSurface.objectSurface(contactIndex, 3)]];
            
            [contactIndex, ~] = knnsearch(kdNSearcher,[objectSurface.xMin objectSurface.yMax], 'K', 1);
            this.exp_additionalContacts = [this.exp_additionalContacts;...
                [objectSurface.xMin objectSurface.yMax objectSurface.objectSurface(contactIndex, 3)]];
            
            [contactIndex, ~] = knnsearch(kdNSearcher,[objectSurface.xMax objectSurface.yMin], 'K', 1);
            this.exp_additionalContacts = [this.exp_additionalContacts;...
                [objectSurface.xMax objectSurface.yMin objectSurface.objectSurface(contactIndex, 3)]];
            
            [contactIndex, ~] = knnsearch(kdNSearcher,[objectSurface.xMax objectSurface.yMax], 'K', 1);
            this.exp_additionalContacts = [this.exp_additionalContacts;...
                [objectSurface.xMax objectSurface.yMax objectSurface.objectSurface(contactIndex, 3)]];
            
            updateModel(this);
            
        end
        
        function plotMesh(this, plotPC, figNum)
            %plotMesh(this, plotPC, figNum);
            plotMesh(this, this.contactLocations, plotPC, figNum, this.nPoints, 'Contact Locations');
        end
        
        function setReferenceSurface(this, referenceSurface, objectSurface)
            setReferenceSurface(this, referenceSurface, objectSurface);
        end
    end
end

function setReferenceSurface(this, referenceSurface, objectSurface)

this.objectModel_CAD = referenceSurface;
this.objectModel_exp = objectSurface;

xlin = linspace(min(referenceSurface(:,1)), max(referenceSurface(:,1)), 60);
ylin = linspace(min(referenceSurface(:,2)), max(referenceSurface(:,2)), 60);
fRef = scatteredInterpolant(referenceSurface(:, 1), referenceSurface(:, 2), referenceSurface(:, 3), 'natural');

[XRef, YRef] = meshgrid(xlin, ylin);
ZRef = fRef(XRef, YRef);

xlin = linspace(min(objectSurface(:,1)), max(objectSurface(:,1)), 60);
ylin = linspace(min(objectSurface(:,2)), max(objectSurface(:,2)), 60);
fEst = scatteredInterpolant(objectSurface(:,1), objectSurface(:,2), objectSurface(:,3), 'natural');

[XEst, YEst] = meshgrid(xlin, ylin);
ZEst = fEst(XEst, YEst);

objectRef = [reshape(XRef, numel(XRef), 1),...
    reshape(YRef, numel(YRef), 1),...
    reshape(ZRef, numel(ZRef), 1)];

objectEst = [reshape(XEst, numel(XEst), 1), ...
    reshape(YEst, numel(YEst), 1), reshape(ZEst, numel(ZEst), 1)];


itr = 200;

[Ricp Ticp] = icp(objectRef', objectEst', itr, 'Matching', 'kDtree', 'Extrapolation', true);


objectEst = transpose(Ricp * objectEst' + repmat(Ticp, 1, length(objectEst)));

scatter3(objectRef(:,1), objectRef(:,2), objectRef(:,3));
hold on
scatter3(objectEst(:,1), objectEst(:,2), objectEst(:,3));
hold off;

this.Ricp = Ricp;
this.Ticp = Ticp;

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



nElements = hist(this.contactLocations((this.nPoints * 4 - 4):end,3), this.nBins);

[~, minElements] = min(nElements);

for bin = 1: this.nBins
    surfUncertainty(:, bin) = 1 - ((surfUncertainty(:, bin) - min(surfUncertainty(:, bin))) ./...
        (max(surfUncertainty(:, bin)) - min(surfUncertainty(:, bin))));
end
% Regression
% %gpUncertainty = runGURLSRegression(this);



%gpUncertainty = zeros(length(surfUncertainty),1); % Disable the regression part

gpUncertainty = maxProb(this, surfaceModel);
alphaE = 0.25;

complexUncertainty = zeros(length(surfUncertainty), 1);
for bin = 1:this.nBins
    complexUncertainty = complexUncertainty + (gpUncertainty * alphaE + surfUncertainty(:,bin) * (1 - alphaE));
end

complexUncertainty = complexUncertainty/this.nBins;

%complexUncertainty = surfUncertainty(:, minElements);

[~, next_idx] = max(complexUncertainty);


this.nextSamplingLocation = [this.inputTesting(next_idx, 1), this.inputTesting(next_idx, 2)];



% % Display
figNum = 2;
plotMesh(this, this.contactLocations, true, figNum, this.nPoints, 'Contact Locations');
figNum = figNum + 1;
plotNextLocation(this, max(this.contactLocations(:,3)), figNum -1);

% % figure(figNum)
% % plot(surfaceModel);
% % figNum = figNum + 1;


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

plotMesh(this, [this.inputTesting maxProbSurface(this, surfaceModel)], false, figNum, this.nPoints, 'Max prob surface');
figNum = figNum + 1;
plotNextLocation(this, max(maxProbSurface(this, surfaceModel)), figNum -1);

plotMesh(this, [this.inputTesting maxProb(this, surfaceModel)], false, figNum, this.nPoints, 'Classification Uncertainty');
figNum = figNum + 1;
plotNextLocation(this, max(maxProb(this, surfaceModel)), figNum -1);

plotMesh(this, [this.inputTesting, complexUncertainty ], false, figNum, this.nPoints, 'Combined Uncertainty');
figNum = figNum + 1;
plotNextLocation(this, max(complexUncertainty), figNum -1);

figure(figNum);
hist(this.contactLocations((this.nPoints * 4 - 4):end,3), this.nBins);

end

function surf = maxProb(this, surface)

if(size(surface, 2) > 1)
surf = zeros(length(surface), 1);
for i = 1:length(surface)
   [prob, ~] = max(surface(i,:));
   surf(i) = prob/sum(surface(i,:));
end
else
    surf = surface;
end

% convert to uncertainty
surf = 1 - ((surf - min(surf))/ (max(surf) - min(surf)));

end

function surf = maxProbSurface(this, surface)
if(size(surface, 2) == 1)
    surf = surface./abs(surface);
    surf(surf == -1) = 0;
    return;
end
surf = zeros(length(surface), 1);
for i = 1:length(surface)
   [~, max_idx] = max(surface(i,:));
   surf(i) = max_idx;
end

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



function evaluate_objectModelCAD(this)

objectSurface =  transpose(this.Ricp * this.contactLocations'  + repmat(this.Ticp, 1, size(this.contactLocations',2))); 
referenceSurface = this.objectModel_CAD;

referenceSurface = [referenceSurface; [min(objectSurface(:,1)) min(objectSurface(:,2)) 0]];
referenceSurface = [referenceSurface; [min(objectSurface(:,1)) max(objectSurface(:,2)) 0]];
referenceSurface = [referenceSurface; [max(objectSurface(:,1)) min(objectSurface(:,2)) 0]];
referenceSurface = [referenceSurface; [max(objectSurface(:,1)) max(objectSurface(:,2)) 0]];

fReference = scatteredInterpolant(referenceSurface(:, 1), referenceSurface(:, 2), referenceSurface(:, 3), 'natural');
fEstimated = scatteredInterpolant(objectSurface(:,1), objectSurface(:,2), objectSurface(:,3), 'natural');

xlin = linspace(min(objectSurface(:,1)), max(objectSurface(:,1)), this.nPoints);
ylin = linspace(min(objectSurface(:,2)), max(objectSurface(:,2)), this.nPoints);
[XT, YT] = meshgrid(xlin, ylin);


referenceSurfaceMesh = fReference(XT, YT);
estimatedSurfaceMesh = fEstimated(XT, YT);

refSurface = [reshape(XT, numel(XT), 1),...
    reshape(YT, numel(YT), 1),...
    reshape(referenceSurfaceMesh, numel(referenceSurfaceMesh), 1)];

estimatedSurface = [reshape(XT, numel(XT), 1), ...
    reshape(YT, numel(YT), 1), reshape(estimatedSurfaceMesh, numel(estimatedSurfaceMesh), 1)];


%figure(9)
%mesh(referenceSurfaceMesh);

% figure(10)
% mesh(referenceSurfaceMesh);
% hold on;
% mesh(estimatedSurfaceMesh)
% hold off;

% % figure (10)
% % scatter3(refSurface(:,1), refSurface(:,2), refSurface(:,3));
% % hold on
% % %scatter3(referenceSurface(:, 1), referenceSurface(:, 2), referenceSurface(:, 3));
% % scatter3(estimatedSurface(:,1), estimatedSurface(:,2), estimatedSurface(:,3)); 
% % hold off
% % title('Reference')
% % 
% % figure(11)
% % %mesh(estimatedSurfaceMesh);
% % scatter3(estimatedSurface(:,1), estimatedSurface(:,2), estimatedSurface(:,3)); 
% % title('Estimated') 

this.surfaceError = [this.surfaceError; sqrt( sum( power(refSurface(:,3) - estimatedSurface(:,3), 2) ) / numel( refSurface(:,3) ))];
figure(11)
plot([this.surfaceError_exp this.surfaceError]);
legend('EXp', 'CAD');

end


function evaluate_objectModelExp(this)

% 
x = this.objectModel_exp(:, 1);
y = this.objectModel_exp(:, 2);


xlin = linspace(min(x),max(x), this.nPoints);
ylin = linspace(min(y),max(y), this.nPoints);
fReference = scatteredInterpolant(x, y, this.objectModel_exp(:, 3), 'natural');
fEstimated = scatteredInterpolant([this.contactLocations(this.nPadding:end,1); this.exp_additionalContacts(:,1)],...
    [this.contactLocations(this.nPadding:end,2); this.exp_additionalContacts(:,2)],...
    [this.contactLocations(this.nPadding:end,3); this.exp_additionalContacts(:,3)], 'natural');

[XT,YT] = meshgrid(xlin,ylin);
referenceSurfaceMesh = fReference(XT, YT);
estimatedSurfaceMesh = fEstimated(XT, YT);

refSurface = [reshape(XT, numel(XT), 1),...
    reshape(YT, numel(YT), 1),...
    reshape(referenceSurfaceMesh, numel(referenceSurfaceMesh), 1)];

estimatedSurface = [reshape(XT, numel(XT), 1), ...
    reshape(YT, numel(YT), 1), reshape(estimatedSurfaceMesh, numel(estimatedSurfaceMesh), 1)];



figure (10)
scatter3(refSurface(:,1), refSurface(:,2), refSurface(:,3));
hold on
scatter3(estimatedSurface(:,1), estimatedSurface(:,2), estimatedSurface(:,3)); 
hold off

% %this.surfaceError = [this.surfaceError; mse(referenceSurfaceMesh, estimatedSurfaceMesh)];
this.surfaceError_exp = [this.surfaceError_exp; sqrt( sum( power(refSurface(:,3) - estimatedSurface(:,3), 2) ) / numel( refSurface(:,3) ))];
% figure(12)
% plot(this.surfaceError_exp);

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

% % kdNSearcher = createns([xlin; ylin], 'NSMethod', 'kdtree', 'distance', 'euclidean');
% % [contactIndex, ~] = knnsearch(kdNSearcher, this.nextSamplingLocation, 'K', 1);
% % plotNextLocation(this, f(xlin(contactIndex), ylin(contactIndex)), figNum);

view([az, el]);
end
