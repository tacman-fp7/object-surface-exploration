%% Surface reconstruction using the saved data.
clear
%close all

nSet = 1;
objectName = {'circPrism', 'triangPrism', 'fish', 'fishSQ', 'hut', 'hutWave'};
explorationType = 'Grid';
trial = 1;
whichHand = 'left';
whichFinger = 'index';
objectIndex = 2;
objectType = objectName{objectIndex};

expDir = '/home/nawidj/tacman/gridSurfaceExplorationData';
cd(sprintf('%s/data/%s/set%02d/trial%02d/processedData', expDir, objectType, nSet, trial));

% load the data

contactPoints = dlmread('contactPoints.csv');
referenceSurface = dlmread(sprintf('%s/data/%s/objectModel.csv', expDir, objectType)); 
referenceSurface = [referenceSurface(:,3), referenceSurface(:,2), abs(referenceSurface(:,1))]; 
referenceSurface = referenceSurface(referenceSurface(:,3) ~= 0, :);
objectSurface = myObject;
objectSurface.initialise(objectName{objectIndex}, downsample(contactPoints,3));
objectSurface.plotMesh(true, objectIndex);

%%
%surfaceModel = surfaceModelGP;
surfaceModel = activeSurfaceModelGP;
surfaceModel.initialise(objectSurface);
surfaceModel.setReferenceSurface(referenceSurface);


%surfaceModel.plotMesh(true, 2);



%%
while(true)
cl = objectSurface.sampleObject(surfaceModel.getNextSamplingPoint());
surfaceModel.addContactLocation(cl);
%surfaceModel.plotMesh(true, 2);
pause(0.01);
end
