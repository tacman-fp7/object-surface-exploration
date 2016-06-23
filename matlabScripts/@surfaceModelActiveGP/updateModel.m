function updateModel(this)

% Determine if we need to increase the number of bins
updateNBins(this);

% Get classification probability
surfaceProb = runGURLS(this);

% Convert the probability to surface uncertainty
surfaceUncertainty = getSurfaceUncertainty(this, surfaceProb);

% Get the spatial uncertainty
spatialUncertainty = getSpatialUncertainty(this);


% combine uncertainty from all bins
combinedUncertainty = zeros(length(surfaceUncertainty), 1);
for bin = 1:this.nBins
    combinedUncertainty = combinedUncertainty +  surfaceUncertainty(:,bin);
end
combinedUncertainty = combinedUncertainty/this.nBins;


% Weigh it again spatial uncertainty
combinedUncertainty = combinedUncertainty .* spatialUncertainty;

% Pick the next sampling location

[~, next_idx] = max(combinedUncertainty);
this.nextSamplingLocation = [this.inputTesting(next_idx, 1), this.inputTesting(next_idx, 2)];





%%%%% Display for debugging only
if(this.plotDebug)
    figNum = 4;
    
    
    plotMesh(this, [this.inputTesting spatialUncertainty], false, figNum, this.nPoints, 'Spatial Uncertainty');
    figNum = figNum + 1;
    plotNextLocation(this, max(spatialUncertainty), figNum -1);
    
    plotMesh(this, [this.inputTesting maxProbSurface(this, surfaceProb)], false, figNum, this.nPoints, 'Max prob surface');
    figNum = figNum + 1;
    plotNextLocation(this, max(maxProbSurface(this, surfaceProb)), figNum -1);
    
    
    
    plotMesh(this, [this.inputTesting, combinedUncertainty ], false, figNum, this.nPoints, 'Combined Uncertainty');
    figNum = figNum + 1;
    plotNextLocation(this, max(combinedUncertainty), figNum -1);
    
    figure(figNum);
    hist(this.contactLocations((this.nPoints * 4 - 4):end,3), this.nBins);
    
end;

end

function surfaceUncertainty = getSurfaceUncertainty(this, surfaceProb)

surfaceUncertainty = abs(surfaceProb);


% Combine the surface uncertainty from all bins.
for bin = 1: this.nBins
    surfaceUncertainty(:, bin) = 1 - ((surfaceUncertainty(:, bin) - min(surfaceUncertainty(:, bin))) ./...
        (max(surfaceUncertainty(:, bin)) - min(surfaceUncertainty(:, bin))));
end
end

function updateNBins(this)

if(length(this.contactLocations) > (this.firstBinThreshold * 5 + (this.nPoints * 4 - 2)))
    this.nBins = 8;
elseif(length(this.contactLocations) > (this.firstBinThreshold * 4 + (this.nPoints * 4 - 2)))
    this.nBins = 6;
elseif(length(this.contactLocations) > (this.firstBinThreshold * 3 + (this.nPoints * 4 - 2)))
    this.nBins = 4;
elseif(length(this.contactLocations) > (this.firstBinThreshold * 2 + (this.nPoints * 4 - 2)))
    this.nBins = 2;
elseif(length(this.contactLocations) > (this.firstBinThreshold + (this.nPoints * 4 - 2)))
    this.nBins = 1;
end

end




function gpModel = getGPModel(this)

name = this.objectName;
gpModel = gurls_defopt(name);
gpModel.seq = {'split:ho', 'paramsel:siglamho', 'kernel:rbf',...
    'rls:dual', 'predkernel:traintest', 'pred:dual',...
    'perf:macroavg', 'perf:precrec', 'conf:boltzman'};

gpModel.process{1} = [2,2,2,2,0,0,0,0,0];
gpModel.process{2} = [3,3,3,3,2,2,2,2,2];
gpModel.epochs = this.epochs;
gpModel.hoperf = @perf_abserr;
gpModel.save = -1;
gpModel.nholdouts = this.nholdouts;
gpModel.hoproportion = 0.1;
gpModel.verbose = 0;
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




function surfaceProb = runGURLS(this)

gpModel = getGPModel(this);
jobID = 1;
target = binContacts(this);

gurls(this.contactLocations( : , 1:2),...
    target, gpModel, jobID); % <-----------

%%% Testing
% Find a better place for a single one time generation
updateInputTesting(this);
outputTesting = zeros(size(this.inputTesting, 1),1); % dummy to help me evalate

gurls(this.inputTesting, repmat(outputTesting, 1, this.nBins) , gpModel, 2);
surfaceProb = gpModel.pred;

end



