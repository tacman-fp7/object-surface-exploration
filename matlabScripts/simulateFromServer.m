%% Simulate the object sampling NO movie
cd('/home/nawidj/tacman/gridSurfaceExplorationData/resultsServer');
objectName = {'circPrism', 'triangPrism', 'fish', 'fishSQ', 'hut', 'hutWave'};



for objectType =1:6
    
    figH = figure(1);
    set(figH, 'color', 'white');
    
    
    
    for nRun = 34
        
        
        load(sprintf('%s_%02d_activeGP_%02d.mat', objectName{objectType}, nRun, nRun));
        activeGP = surfaceModel;
        load(sprintf('%s_%02d_passiveGP_%02d.mat', objectName{objectType}, nRun, nRun));
        passiveGP = surfaceModel;
        load(sprintf('%s_%02d_random_%02d.mat',objectName{objectType}, nRun, nRun));
        randomS = surfaceModel;
        
        maxContacts = 80;
        
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

%% plot averages

cd('/home/nawidj/tacman/gridSurfaceExplorationData/resultsServer');
objectName = {'circPrism', 'triangPrism', 'fish', 'fishSQ', 'hut', 'hutWave'};
maxContacts = 80;

for objectType =1:6
    surfRMSE_active = [];
    surfRMSE_passive = [];
    surfRMSE_random = [];
    
    for nRun = 31:38
        
        load(sprintf('%s_%02d_activeGP_%02d.mat', objectName{objectType}, nRun, nRun));
        %surfaceModel.reEvaluateRMSE();
        surfRMSE_active = [surfRMSE_active, surfaceModel.surfaceRMSE(1:maxContacts,:)];
        load(sprintf('%s_%02d_passiveGP_%02d.mat', objectName{objectType}, nRun, nRun));
        %surfaceModel.reEvaluateRMSE()
        surfRMSE_passive = [surfRMSE_passive, surfaceModel.surfaceRMSE(1:maxContacts,:)];
        load(sprintf('%s_%02d_random_%02d.mat',objectName{objectType}, nRun, nRun));
        %surfaceModel.reEvaluateRMSE()
        surfRMSE_random = [surfRMSE_random, surfaceModel.surfaceRMSE(1:maxContacts,:)];
        
        
        % %         plot(surfRMSE);
        % %         legend('Active', 'Passive', 'Random');
        % %         pause;
    end
    
    plot([mean(surfRMSE_active, 2), mean(surfRMSE_passive,2), mean(surfRMSE_random,2)]);
    xlim([2 maxContacts]);
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


