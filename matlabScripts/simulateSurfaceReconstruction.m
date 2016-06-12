%% Surface reconstruction using the saved data.
clear
%close all

nSet = 1;
objectName = {'circPrism', 'triangPrism', 'fish', 'fishSQ', 'hut', 'hutWave'};
explorationType = 'Grid';
trial = 1;
whichHand = 'left';
whichFinger = 'index';
objectIndex = 4;
objectType = objectName{objectIndex};

expDir = '/home/nawidj/tacman/gridSurfaceExplorationData';
cd(sprintf('%s/data/%s/set%02d/trial%02d/processedData', expDir, objectType, nSet, trial));

% load the data

contactPoints = dlmread('contactPoints.csv');
contactPoints = contactPoints(:,1:3);

objectSurface = myObject;
objectSurface.initialise(objectName{objectIndex}, contactPoints);
objectSurface.plotMesh(true, objectIndex);

%%
%surfaceModel = surfaceModelGP;
surfaceModel = activeSurfaceModelGP;
surfaceModel.initialise(objectSurface);



%surfaceModel.plotMesh(true, 2);



%%
while(true)
cl = objectSurface.sampleObject(surfaceModel.getNextSamplingPoint());
surfaceModel.addContactLocation(cl);
%surfaceModel.plotMesh(true, 2);
pause(0.01);
end
