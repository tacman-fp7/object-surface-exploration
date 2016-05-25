%% Surface reconstruction using the saved data.
clear
close all

nSet = 1;
objectName = {'circPrism', 'triangPrism', 'fish', 'fishSQ', 'hut', 'hutWave'};
explorationType = 'Grid';
trial = 1;
whichHand = 'left';
whichFinger = 'index';
objectIndex = 1;
objectType = objectName{objectIndex};

expDir = '/home/nawidj/tacman/gridSurfaceExplorationData';
cd(sprintf('%s/data/%s/set%02d/trial%02d/processedData', expDir, objectType, nSet, trial));

% load the data

contactPoints = dlmread('contactPoints.csv');

objectSurface = myObject;
objectSurface.initialise(objectName{objectIndex}, contactPoints);
%objectSurface.plotMesh();

surfaceModel = surfaceModelGP;
surfaceModel.initialise(objectSurface);
%surfaceModel.plotMesh();


%%
while(true)
cl = objectSurface.sampleObject(surfaceModel.getNextSamplingPoint());
surfaceModel.addContactLocation(cl);

pause(0.1)
end
