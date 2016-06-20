%% Surface reconstruction using the saved data.
%clear
%close all
 warning('off', 'all');
for testRun = 1:10
    fprintf('Run: %02d\n', testRun);
    nSet = 1;
    objectName = {'circPrism', 'triangPrism', 'fish', 'fishSQ', 'hut', 'hutWave'};
    explorationType = 'Grid';
    trial = 1;
    whichHand = 'left';
    whichFinger = 'index';
    for objectIndex = 1:6
        fprintf('Object: %s, ', objectName{objectIndex}); 
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
        %objectModel.plotMesh(true, 1);
        
        %% Change to the new directory
        cd('/home/nawidj/tacman/gridSurfaceExplorationData/data/results');
        
        %%
        fprintf('Random...');
        clear surfaceModel;
        surfaceModel = surfaceModelRandom(objectModel, contactPoints);
        surfaceModel.setMaxSamplePoints(10);
        %surfaceModel.plotResults()
        %%%
        isDone = false;
        while(~isDone)
            cl = objectModel.sampleObject(surfaceModel.getNextSamplingLocation());
            isDone = surfaceModel.addContactLocation(cl);
        end
        %
        delete([objectModel.objectName '.mat']);
        save(sprintf('%s_random_%02d.mat', objectModel.objectName, testRun), 'surfaceModel', '-v7.3');
        
        %%%%
        fprintf('PassiveGP...');
        clear surfaceModel;
        surfaceModel = surfaceModelPassiveGP(objectModel, contactPoints);
        surfaceModel.setMaxSamplePoints(10);
        %surfaceModel.plotResults()
        %%%
        isDone = false;
        while(~isDone)
            cl = objectModel.sampleObject(surfaceModel.getNextSamplingLocation());
            isDone = surfaceModel.addContactLocation(cl);
        end
        %
        delete([objectModel.objectName '.mat']);
        save(sprintf('%s_passiveGP_%02d.mat', objectModel.objectName, testRun), 'surfaceModel', '-v7.3');
        
        %%%
        fprintf('ActiveGP\n');
        clear surfaceModel;
        surfaceModel = surfaceModelActiveGP(objectModel, contactPoints);
        surfaceModel.setMaxSamplePoints(10);
        %surfaceModel.plotResults()
        %%%
        isDone = false;
        while(~isDone)
            cl = objectModel.sampleObject(surfaceModel.getNextSamplingLocation());
            isDone = surfaceModel.addContactLocation(cl);
        end
        %
        delete([objectModel.objectName '.mat']);
        save(sprintf('%s_activeGP_%02d.mat', objectModel.objectName, testRun), 'surfaceModel', '-v7.3');
        
    end
end
%%

surfRMSE = [];
for nRun = 1
    
    load('circPrism_activeGP_01.mat');
    surfRMSE = [surfRMSE, surfaceModel.surfaceRMSE];
    load('circPrism_passiveGP_01.mat');
    surfRMSE = [surfRMSE, surfaceModel.surfaceRMSE];
    load('circPrism_random_01.mat');
    surfRMSE = [surfRMSE, surfaceModel.surfaceRMSE];
    
    plot(surfRMSE);
    legend('Active', 'Passive', 'Random');
    
end



















