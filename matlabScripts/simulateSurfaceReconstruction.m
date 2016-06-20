%% Surface reconstruction using the saved data.
clear
%close all

nSet = 1;
objectName = {'circPrism', 'triangPrism', 'fish', 'fishSQ', 'hut', 'hutWave'};
explorationType = 'Grid';
trial = 1;
whichHand = 'left';
whichFinger = 'index';
objectIndex = 5;
objectType = objectName{objectIndex};

expDir = '/home/nawidj/tacman/gridSurfaceExplorationData';
cd(sprintf('%s/data/%s/set%02d/trial%02d/processedData', expDir, objectType, nSet, trial));

% load the data

contactPoints = dlmread('contactPoints.csv');
contactPoints = contactPoints(:,1:3);

%contactPoints = downsample(contactPoints, 3);
% % referenceSurface = dlmread(sprintf('%s/data/%s/objectModel.csv', expDir, objectType)); 
% % referenceSurface = [referenceSurface(:,3), referenceSurface(:,2), abs(referenceSurface(:,1))]; 
% % referenceSurface = referenceSurface(referenceSurface(:,3) ~= 0, :);
objectModel = myObject;
objectModel.initialise(objectName{objectIndex}, contactPoints);
objectModel.plotMesh(true, 1);

%%
clear surfModel;
surfaceModel = surfaceModelPassiveGP(objectModel, contactPoints);
surfaceModel.plotResults()
%%
while(true)
cl = objectModel.sampleObject(surfaceModel.getNextSamplingLocation());
surfaceModel.addContactLocation(cl);
surfaceModel.plotResults();
pause(0.01);
end

