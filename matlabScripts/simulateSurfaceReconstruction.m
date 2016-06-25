%% Surface reconstruction using the saved data.
%clear
%close all

%% Test runs
nSet = 1;
objectName = {'circPrism', 'triangPrism', 'fish', 'fishSQ', 'hut', 'hutWave'};
explorationType = 'Grid';
trial = 1;
whichHand = 'left';
whichFinger = 'index';
objectIndex = 2;

fprintf('Object: %s, ', objectName{objectIndex});
objectType = objectName{objectIndex};

expDir = '/home/nawidj/tacman/gridSurfaceExplorationData';
cd(sprintf('%s/data/%s/set%02d/trial%02d/processedData', expDir, objectType, nSet, trial));

% load the data

contactPoints = dlmread('contactPoints.csv');
contactPoints = contactPoints(:,1:3);


objectModel = myObject;
objectModel.initialise(objectName{objectIndex}, contactPoints);


clear surfaceModel;
surfaceModel = surfaceModelActiveGP(objectModel, contactPoints);
surfaceModel.setExpochs(100);
surfaceModel.setNholdouts(1);
surfaceModel.setMaxSamplePoints(60);
surfaceModel.enableDebugPlot(true);

%%%
isDone = false;
while(~isDone)
    cl = objectModel.sampleObject(surfaceModel.getNextSamplingLocation());
    surfaceModel.plotResults();
    isDone = surfaceModel.addContactLocation(cl);
end

surfaceModel.plotResults();

%% batch run
warning('off', 'all');
maxContacts = 60;
for testRun = 17
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
        
    end
end

fprintf('\n\nDone!\n\n');
%%

cd('/home/nawidj/tacman/gridSurfaceExplorationData/results');
objectName = {'circPrism', 'triangPrism', 'fish', 'fishSQ', 'hut', 'hutWave'};

maxContacts = 150;
for objectType =1:6
    surfRMSE_active = [];
    surfRMSE_passive = [];
    surfRMSE_random = [];
    
    for nRun = 1:5
        
        
        load(sprintf('%s_activeGP_%02d.mat', objectName{objectType}, nRun));
        %surfaceModel.reEvaluateRMSE();
        surfRMSE_active = [surfRMSE_active, surfaceModel.surfaceRMSE(1:maxContacts,:)];
        load(sprintf('%s_passiveGP_%02d.mat', objectName{objectType}, nRun));
        %surfaceModel.reEvaluateRMSE();
        surfRMSE_passive = [surfRMSE_passive, surfaceModel.surfaceRMSE(1:maxContacts,:)];
        load(sprintf('%s_random_%02d.mat',objectName{objectType}, nRun));
        %surfaceModel.reEvaluateRMSE();
        surfRMSE_random = [surfRMSE_random, surfaceModel.surfaceRMSE(1:maxContacts,:)];
        
        
        % %         plot(surfRMSE);
        % %         legend('Active', 'Passive', 'Random');
        % %         pause;
    end
    fprintf('Done!\n');
   
    plot([mean(surfRMSE_active, 2), mean(surfRMSE_passive,2), mean(surfRMSE_random,2)]);
    hold on;
    x = 1:length(surfRMSE_active);
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

%% Simulate the object sampling NO movie
cd('/home/nawidj/tacman/gridSurfaceExplorationData/results');
objectName = {'circPrism', 'triangPrism', 'fish', 'fishSQ', 'hut', 'hutWave'};



for objectType =1:6
    
    figH = figure(1);
    set(figH, 'color', 'white');
    
    
    
    for nRun = 17
        
        load(sprintf('%s_activeGP_%02d.mat', objectName{objectType}, nRun));
        activeGP = surfaceModel;
        load(sprintf('%s_passiveGP_%02d.mat', objectName{objectType}, nRun));
        passiveGP = surfaceModel;
        load(sprintf('%s_random_%02d.mat',objectName{objectType}, nRun));
        randomS = surfaceModel;
        
        maxContacts = 60;
        
        for nContacts = 1:maxContacts
            viewPars =[50 42];
            subplot(2,3,1);
            activeGP.getMesh(nContacts, 'Active GP', viewPars);
            subplot(2,3,2);
            passiveGP.getMesh(nContacts, 'Passive GP', viewPars);
            subplot(2,3,3);
            randomS.getMesh(nContacts, 'Random Selection', viewPars);
            %activeGP
            subplot(2,3,4);
            imshow(sprintf('%s.png', objectName{objectType}));
            set(gca, 'fontname', 'Bitstream Charter','fontsize', 15);
            title(['The object: ' objectName{objectType}], 'fontsize', 20, 'interpreter', 'tex');
            
            subplot(2,3,5);
            activeGP.plotReferenceSurace('Grid Sampling', viewPars);
            
            
            subplot(2,3,6);
            plot([activeGP.surfaceRMSE(1:nContacts), passiveGP.surfaceRMSE(1:nContacts), randomS.surfaceRMSE(1:nContacts)]);
            set(gca, 'fontname', 'Bitstream Charter','fontsize', 15);
            xlim([1, maxContacts]);
            %             ylim([min(min([activeGP.surfaceRMSE, passiveGP.surfaceRMSE, randomS.surfaceRMSE])),...
            %                 max(max([activeGP.surfaceRMSE, passiveGP.surfaceRMSE, randomS.surfaceRMSE]))]);
            title({'RMSE between the Grid surface and', 'the sampled surface'}, 'fontsize', 20, 'interpreter', 'tex');
            
            xlabel('Number of locations sampled');
            ylabel('RMSE [m]');
            legend('Active GP', 'Passive GP', 'Random');
            %pause(0.1);
            drawnow;
            %break;
           
        end
        
        %fprintf('here');
        
        
        for i = 50:410
            subplot(2,3,1);
            view([i, 42]);
            subplot(2,3,2);
            view([i, 42]);
            subplot(2,3,3);
            view([i, 42]);
            subplot(2,3,5);
            view([i, 42]);
            drawnow;
            
        end
        
        for i = 42:90
            
            subplot(2,3,1);
            view([50, i]);
            subplot(2,3,2);
            view([50, i]);
            subplot(2,3,3);
            view([50, i]);
            subplot(2,3,5);
            view([50, i]);
            drawnow;
            
            
        end
    end
    
   
    
end





%% Simulate the object sampling movie
cd('/home/nawidj/tacman/gridSurfaceExplorationData/results');
objectName = {'circPrism', 'triangPrism', 'fish', 'fishSQ', 'hut', 'hutWave'};



for objectType =1:6
    myVideoObj = VideoWriter(sprintf('%s.avi', objectName{objectType}));
    myVideoObj.FrameRate = 8;
    open(myVideoObj);
    
    %Preallocate the struct array for the struct returned by getframe
    movieFrames(10000) = struct('cdata',[],'colormap',[]);
    nFrame = 1;
    figH = figure(1);
    set(figH, 'color', 'white');
    
    %     ax = gca;
    %     set(gca, 'Units', 'pixels');
    %     pos = get(gca, 'Position');
    %     ti = get(gca, 'LooseInset');
    %     rect = [-ti(1), -ti(2), pos(3)+ti(1)+ti(3), pos(4)+ti(2)+ti(4)];
    rect = [180, 30, 1600, 920];
    
    for nRun = 3
        
        load(sprintf('%s_activeGP_%02d.mat', objectName{objectType}, nRun));
        activeGP = surfaceModel;
        load(sprintf('%s_passiveGP_%02d.mat', objectName{objectType}, nRun));
        passiveGP = surfaceModel;
        load(sprintf('%s_random_%02d.mat',objectName{objectType}, nRun));
        randomS = surfaceModel;
        
        maxContacts = 100;
        
        for nContacts = 1:maxContacts
            viewPars =[50 42];
            subplot(2,3,1);
            activeGP.getMesh(nContacts, 'Active GP', viewPars);
            subplot(2,3,2);
            passiveGP.getMesh(nContacts, 'Passive GP', viewPars);
            subplot(2,3,3);
            randomS.getMesh(nContacts, 'Random Selection', viewPars);
            %activeGP
            subplot(2,3,4);
            imshow(sprintf('%s.png', objectName{objectType}));
            set(gca, 'fontname', 'Bitstream Charter','fontsize', 15);
            title(['The object: ' objectName{objectType}], 'fontsize', 20, 'interpreter', 'tex');
            
            subplot(2,3,5);
            activeGP.plotReferenceSurace('Grid Sampling', viewPars);
            
            
            subplot(2,3,6);
            plot([activeGP.surfaceRMSE(1:nContacts), passiveGP.surfaceRMSE(1:nContacts), randomS.surfaceRMSE(1:nContacts)]);
            set(gca, 'fontname', 'Bitstream Charter','fontsize', 15);
            xlim([1, maxContacts]);
            %             ylim([min(min([activeGP.surfaceRMSE, passiveGP.surfaceRMSE, randomS.surfaceRMSE])),...
            %                 max(max([activeGP.surfaceRMSE, passiveGP.surfaceRMSE, randomS.surfaceRMSE]))]);
            title({'RMSE between the Grid surface and', 'the sampled surface'}, 'fontsize', 20, 'interpreter', 'tex');
            
            xlabel('Number of locations sampled');
            ylabel('RMSE [m]');
            legend('Active GP', 'Passive GP', 'Random');
            %pause(0.1);
            
            %break;
            drawnow
            
            
            
            movieFrames(nFrame) = getframe(figH, rect);
            %writeVideo(myVideoObj, movieFrames(nFrame));
            nFrame = nFrame+1;
        end
        
        %fprintf('here');
        
        
        for i = 50:410
            subplot(2,3,1);
            view([i, 42]);
            subplot(2,3,2);
            view([i, 42]);
            subplot(2,3,3);
            view([i, 42]);
            subplot(2,3,5);
            view([i, 42]);
            
            drawnow;
            
            movieFrames(nFrame) = getframe(figH, rect);
            %writeVideo(myVideoObj, movieFrames(nFrame));
            nFrame = nFrame+1;
        end
        
        for i = 42:90
            
            subplot(2,3,1);
            view([50, i]);
            subplot(2,3,2);
            view([50, i]);
            subplot(2,3,3);
            view([50, i]);
            subplot(2,3,5);
            view([50, i]);
            
            drawnow;
            
            movieFrames(nFrame) = getframe(figH, rect);
            %writeVideo(myVideoObj, movieFrames(nFrame));
            nFrame = nFrame+1;
            
        end
    end
    
    
    for i = 1:nFrame -1;
        writeVideo(myVideoObj, movieFrames(i));
    end
    close(myVideoObj);
    
end





