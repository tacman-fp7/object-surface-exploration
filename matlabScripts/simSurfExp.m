function simSurfExp(objectIndex, testRunStart, testRunEnd)
warning('off', 'all');
maxContacts = 60;
for testRun = testRunStart:testRunEnd
    fprintf('Run: %02d\n', testRun);
    nSet = 1;
    objectName = {'circPrism', 'triangPrism', 'fish', 'fishSQ', 'hut', 'hutWave'};
    explorationType = 'Grid';
    trial = 1;
    whichHand = 'left';
    whichFinger = 'index';
    %for objectIndex = 1:6
        fprintf('Object: %s, ', objectName{objectIndex});
        objectType = objectName{objectIndex};
        
        expDir = '/home/iit.local/njamali/surfaceExpResults';
        cd(sprintf('%s/data/%s/set%02d/trial%02d/processedData', expDir, objectType, nSet, trial));
        
        % load the data
        
        contactPoints = dlmread('contactPoints.csv');
        contactPoints = contactPoints(:,1:3);
        
        %contactPoints = downsample(contactPoints, 3);
        % % referenceSurface = dlmread(sprintf('%s/data/%s/objectModel.csv', expDir, objectType));
        % % referenceSurface = [referenceSurface(:,3), referenceSurface(:,2), abs(referenceSurface(:,1))];
        % % referenceSurface = referenceSurface(referenceSurface(:,3) ~= 0, :);
        objectModel = myObject;
        objectModel.initialise(sprintf('%s_%02d', objectName{objectIndex}, testRun ), contactPoints);
        objectModel.expandObject();
        contactPoints = objectModel.objectSurface;
        %objectModel.plotMesh(true, 1);
        
        %% Change to the new directory
        cd('/home/iit.local/njamali/surfaceExpResults/results');
        
        %%
        fprintf('Random...');
        clear surfaceModel;
        surfaceModel = surfaceModelRandom(objectModel, contactPoints);
        surfaceModel.setMaxSamplePoints(maxContacts);
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
        surfaceModel.setMaxSamplePoints(maxContacts);
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
        surfaceModel.setMaxSamplePoints(maxContacts);
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
        
    %end
end

fprintf('\n\nDone!\n\n');

end