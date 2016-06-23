function updateModel(this)
%fprintf('Called Passive GP update \n');
surfaceUncertainty = runGURLS(this);
surfaceUncertainty = normalise(surfaceUncertainty);
[~, maxUncertainty_idx] = max(surfaceUncertainty);
this.nextSamplingLocation = [this.inputTesting(maxUncertainty_idx,1), this.inputTesting(maxUncertainty_idx,2)];

%%% debugging display
if(this.plotDebug)
figNum = 4;
plotMesh(this, [this.inputTesting surfaceUncertainty], false, figNum, this.nPoints, 'Surface Uncertainty');
figNum = figNum + 1;
plotNextLocation(this, max(surfaceUncertainty), figNum -1);

end;


end

function normalised = normalise(data)
normalised = (data(:, end) - min(data(:,end)))/...
    (max(data(:,end)) - min(data(:, end)));
end

function gpModel = getGPModel(this)

name = this.objectName;
gpModel = gurls_defopt(name);
gpModel.seq = {'split:ho', 'paramsel:siglamhogpregr', 'kernel:rbf',...
    'rls:gpregr', 'predkernel:traintest', 'pred:gpregr'};
gpModel.process{1} = [2,2,2,2,1,1];
gpModel.process{2} = [3,3,3,3,2,2];
gpModel.epochs = this.epochs;
gpModel.hoperf = @perf_abserr;
gpModel.save = -1;
gpModel.nholdouts = this.nholdouts;
gpModel.hoproportion = 0.1;
gpModel.verbose = 0;
end

function surfaceUncertainty = runGURLS(this)

gpModel = getGPModel(this);
jobID = 1;
gurls(this.contactLocations( : , 1:2),...
      this.contactLocations( : , 3), gpModel, jobID);

%%% Testing
jobID = 2;

% Find a better place for a single one time generation
updateInputTesting(this);
outputTesting = zeros(size(this.inputTesting, 1),1); % dummy to help me evalate

gurls(this.inputTesting, outputTesting , gpModel, jobID);
surfaceUncertainty = gpModel.pred.vars;
%surfaceModel = gpModel.pred.means;

end










