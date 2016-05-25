classdef surfaceModelGP < handle
    
    properties
        contactLocations;
        gpModel;
        nPoints = 10;
        objectName;
        nextSamplingLocation = zeros(1,2);
    end
    
    methods
        function nextSamplingLocation = getNextSamplingPoint(this)
            nextSamplingLocation = this.nextSamplingLocation;
        end
        
        function addContactLocation(this, contactLocation)
            this.contactLocations = [this.contactLocations; contactLocation];
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
        
        function plotMesh(this)
            plotMesh(this);
        end
    end
end


function plotMesh(objectSurface)

figure;
x = objectSurface.contactLocations(:, 1);
y = objectSurface.contactLocations(:, 2);
z = objectSurface.contactLocations(:, 3);


xlin = linspace(min(x),max(x),60);
ylin = linspace(min(y),max(y),60);
[XT,YT] = meshgrid(xlin,ylin);

f = scatteredInterpolant(x,y,z, 'natural');
ZT = f(XT,YT);


mesh(XT, YT, ZT);

hold on
scatter3(...
    objectSurface.contactLocations(:, 1),...
    objectSurface.contactLocations(:, 2),...
    objectSurface.contactLocations(:, 3),...
    'fill', 'markerFaceColor', 'blue', 'sizeData', [90]);
hold off
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%          update the model         %%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function opt = updateModel(this)


inputTraining = this.contactLocations( : , 1:2);
outputTraining = this.contactLocations( : , 3);

name = this.objectName;
opt = gurls_defopt(name);
opt.seq = {'split:ho', 'paramsel:siglamhogpregr', 'kernel:rbf',...
    'rls:gpregr', 'predkernel:traintest', 'pred:gpregr'};
opt.process{1} = [2,2,2,2,1,1];
opt.process{2} = [3,3,3,3,2,2];
opt.epochs = 100000;
opt.hoperf = @perf_abserr;
opt.save = -1;

opt.nholdouts = 4;
opt.hoproportion = 0.1;


%%% Run GP using GURLS
jobID = 1;
gurls(inputTraining, outputTraining, opt, jobID);

%%% Testing

xlin = linspace(min(inputTraining( : , 1)), max(inputTraining( : , 1)), this.nPoints);
ylin = linspace(min(inputTraining( : , 2)), max(inputTraining( : , 2)), this.nPoints);

inputTesting = [];
for i = 1:length(xlin)
    inputTesting =  [inputTesting;...
        repmat(xlin(i), length(ylin), 1) ylin'];
end

%%%
outputTesting = zeros(size(inputTesting, 1),1); % dummy to help me evalate

gurls(inputTesting, outputTesting , opt, 2);

x = inputTesting(:, 1);
y = inputTesting(:, 2);

z = opt.pred.vars;

[maxVar, maxVarIndex] = max(z);

this.nextSamplingLocation = [inputTesting(maxVarIndex, 1), inputTesting(maxVarIndex, 2)];

fprintf('Max variance (%d) at: (%d, %d)\n', maxVar, inputTesting(maxVarIndex, 1), inputTesting(maxVarIndex, 2));

xlin = linspace(min(x), max(x), this.nPoints);
ylin = linspace(min(y), max(y), this.nPoints);
[XT,YT] = meshgrid(xlin,ylin);

ZT = reshape(z, this.nPoints, this.nPoints);

figure(1);

mesh(XT, YT, ZT);

figure(2);
mesh(XT, YT, reshape(opt.pred.means, this.nPoints, this.nPoints))

hold on
scatter3(...
    this.contactLocations(:, 1),...
    this.contactLocations(:, 2),...
    this.contactLocations(:, 3),...
    'fill', 'markerFaceColor', 'blue', 'sizeData', [90]);
hold off

end
