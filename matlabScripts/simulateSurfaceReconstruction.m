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
        surfaceModel.setMaxSamplePoints(100);
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
        surfaceModel.setMaxSamplePoints(100);
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
        surfaceModel.setMaxSamplePoints(100);
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

fprintf('\n\nDone!\n\n');
%%

cd('/home/nawidj/tacman/gridSurfaceExplorationData/data/results');
objectName = {'circPrism', 'triangPrism', 'fish', 'fishSQ', 'hut', 'hutWave'};

for objectType =1:6
    surfRMSE_active = [];
    surfRMSE_passive = [];
    surfRMSE_random = [];
    
    for nRun = 1:5
        
        load(sprintf('%s_activeGP_%02d.mat', objectName{objectType}, nRun));
        surfRMSE_active = [surfRMSE_active, surfaceModel.surfaceRMSE];
        load(sprintf('%s_passiveGP_%02d.mat', objectName{objectType}, nRun));
        surfRMSE_passive = [surfRMSE_passive, surfaceModel.surfaceRMSE];
        load(sprintf('%s_random_%02d.mat',objectName{objectType}, nRun));
        surfRMSE_random = [surfRMSE_random, surfaceModel.surfaceRMSE];
        
        
        % %         plot(surfRMSE);
        % %         legend('Active', 'Passive', 'Random');
        % %         pause;
    end
    
    plot([mean(surfRMSE_active, 2), mean(surfRMSE_passive,2), mean(surfRMSE_random,2)]);
    hold on;
    x = 1:101;
    y = transpose(mean(surfRMSE_random, 2));
    err = std(surfRMSE_active');
    patch([x fliplr(x)],[y+err fliplr(y-err)], 'red', 'FaceAlpha', 0.3, 'LineStyle', 'none');
    
    y = transpose(mean(surfRMSE_passive, 2));
    err = std(surfRMSE_active');
    patch([x fliplr(x)],[y+err fliplr(y-err)], 'green', 'FaceAlpha', 0.3, 'LineStyle', 'none');
    
    y = transpose(mean(surfRMSE_active, 2));
    err = std(surfRMSE_active');
    patch([x fliplr(x)],[y+err fliplr(y-err)], 'blue', 'FaceAlpha', 0.3, 'LineStyle', 'none');
    
    plot([mean(surfRMSE_active, 2), mean(surfRMSE_passive,2), mean(surfRMSE_random,2)]);
    
    hold off;
    legend('Active', 'Passive', 'Random');
    pause;
end


%% Simulate the object sampling
cd('/home/nawidj/tacman/gridSurfaceExplorationData/data/results');
objectName = {'circPrism', 'triangPrism', 'fish', 'fishSQ', 'hut', 'hutWave'};

for objectType =1:6
    surfRMSE_active = [];
    surfRMSE_passive = [];
    surfRMSE_random = [];
    
    for nRun = 1:5
        
        load(sprintf('%s_activeGP_%02d.mat', objectName{objectType}, nRun));
        activeGP = surfaceModel;
        load(sprintf('%s_passiveGP_%02d.mat', objectName{objectType}, nRun));
        passiveGP = surfaceModel;
        load(sprintf('%s_random_%02d.mat',objectName{objectType}, nRun));
        randomS = surfaceModel;
        
        
        
        
        subplot(1,3,1);
        activeGP
        
    end
    
end






