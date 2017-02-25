%cd('/home/nawidj/software/src/object-surface-exploration/build');
%cd('/home/nawidj/gpdata')
%cd('/home/nawidj/gpDataTrial06')
%cd('/home/nawidj/tacman/GaussianSurfaceExplorationData/data/hut/set02/trial05/gpPoints');
cd('/home/nawidj/demoData');
objectName = 'test';
nextPointFileName = [objectName '_model_nextPoint.csv'];
modelInputFileName = [objectName '_model_input.csv'];
modelOutputRegressionFileName = [objectName '_model_output_GPRegression.csv'];
modelVarianceRegressionFileName = [objectName '_model_variance_GPRegression.csv'];
modelOutputClassificationFileName = [objectName '_model_output_GPClassification.csv'];
modelOutputSurfaceClassificationFileName =  [objectName '_model_output_GPSurfaceClassification.csv'];
modelVarianceClassificationFileName = [objectName '_model_variance_GPClassification.csv'];
modelVarianceCombinedFileName = [objectName '_model_variance_GPCombined.csv'];
trainingInputFileName = [objectName '_training_input_GP.csv'];
trainingTargetFileName = [objectName '_training_target_GP.csv'];
nextSamplePointFileName = [objectName '_nextPoint.csv'];

indexFingerFileName = [objectName '_finger_1_GP.csv'];
middleFingerFileName = [objectName '_finger_2_GP.csv'];


%%

viewVars = [50, 35];
clf('reset');
while(true)
    
    isLatentValid = false;
    isSurfaceModelValid = false;
    
    
    while(~exist(nextPointFileName, 'file'))
        pause(0.05);
    end
    
    
    
    mmFactor = 1000;
    
    maxVarPoint = dlmread(nextPointFileName);
    
    
    
    
    delete(nextPointFileName);
    
    if(exist('taxel.csv', 'file'))
        
        fileID = fopen('taxel.csv');
        if fseek(fileID, 1, 'bof') == 0
            
            % Check for more than one field
            testLatent = dlmread('taxel.csv');
            if(size(testLatent, 1) > 1)
                isLatentValid = true;
            end
        end
        fclose(fileID);
    end
    
    if(exist(modelOutputClassificationFileName, 'file'))
        isSurfaceModelValid = true;
    end
    
    
    if(isLatentValid)
        system('python vae.py --taxel-file=./taxel.csv --latent-file=latent.csv');
    end
    
    
    
    
    
    nPoints = 20;
    
    modelInput = dlmread(modelInputFileName);
    modelOutputRegression = dlmread(modelOutputRegressionFileName);
    modelVarianceCombined = dlmread(modelVarianceCombinedFileName);
    
    if(isSurfaceModelValid)
        modelOutputClassification = dlmread(modelOutputClassificationFileName);
        modelOutputSurfaceClassification = dlmread(modelOutputSurfaceClassificationFileName);
        %modelVarianceRegression = dlmread(modelVarianceRegressionFileName);
        %modelVarianceClassification = dlmread(modelVarianceClassificationFileName);
    end
    
    
    
    
    trainingInput = dlmread(trainingInputFileName);
    trainingTarget = dlmread(trainingTargetFileName);
    if(isLatentValid)
        latentVariables = dlmread('latent.csv');
    end
    
    if(exist(indexFingerFileName, 'file') == 2)
        indexFingerLocations = dlmread(indexFingerFileName, ',');
        indexFingerLocations(:, 1:2) = (indexFingerLocations(:, 1:2) - repmat(min(trainingInput), size(indexFingerLocations,1), 1)) * mmFactor;
        indexFingerLocations(:, 3) = (indexFingerLocations(:, 3) - repmat(min(trainingTarget), size(indexFingerLocations,1), 1)) * mmFactor;
    end
    
    if(exist(middleFingerFileName, 'file') == 2)
        middleFingerLocations = dlmread(middleFingerFileName, ',');
        middleFingerLocations(:, 1:2) = (middleFingerLocations(:, 1:2) - repmat(min(trainingInput), size(middleFingerLocations,1), 1)) * mmFactor;
        middleFingerLocations(:, 3) = (middleFingerLocations(:, 3) - repmat(min(trainingTarget), size(middleFingerLocations,1), 1)) * mmFactor;
        
    end
    
    maxVarPoint(1) = maxVarPoint(1) - min(modelInput(:,1));
    maxVarPoint(2) = maxVarPoint(2) - min(modelInput(:,2));
    maxVarPoint(3) = maxVarPoint(3) - min(modelOutputRegression);
    maxVarPoint = maxVarPoint * mmFactor;
    fprintf('Next sampling point is: (%0.4f, %0.4f. %0.4f)\n', maxVarPoint(1), maxVarPoint(2), maxVarPoint(3));
    
    
    modelInput = (modelInput - repmat(min(modelInput), length(modelInput), 1)) * mmFactor;
    modelOutputRegression = (modelOutputRegression - min(modelOutputRegression)) * mmFactor;
    
    
    
    
    trainingInput = (trainingInput - repmat(min(trainingInput), length(trainingInput), 1)) * mmFactor;
    trainingTarget = (trainingTarget - min(trainingTarget)) * mmFactor;
    
    
    %indexFingerLocation = (indexFingerLocation - repmat(min(trainingInput), length(indexFingerLocation), 1))
    
    x = modelInput(:, 1);
    y = modelInput(:, 2);
    
    fprintf('Xmin: %f, Xmax: %f, Ymin: %f, Ymax: %f\n',...
        min(x), max(x), min(y), max(y));
    
    xlin = linspace(min(x), max(x), nPoints);
    ylin = linspace(min(y), max(y), nPoints);
    [XT,YT] = meshgrid(xlin,ylin);
    %ZTRegression = reshape(modelOutputRegression, nPoints, nPoints);
    %ZVRegression = reshape(modelVarianceRegression, nPoints, nPoints);
    
    figH = figure(1);
    set(figH, 'color', 'white');
    
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if(isSurfaceModelValid)
        
        ZTClassification = reshape(modelOutputSurfaceClassification * 10, nPoints, nPoints);
        %ZVClassification = reshape(modelVarianceClassification, nPoints, nPoints);
        
        subplot(2,2,1)
        %figure(1)
        
        mesh(XT, YT, ZTClassification);
        set(gca, 'fontname', 'Bitstream Charter','fontsize', 15);
        xlabel('Width [mm]','fontsize', 15, 'interpreter', 'tex', 'verticalAlignment', 'bottom');
        ylabel('Length [mm]','fontsize', 15, 'interpreter', 'tex', 'verticalAlignment', 'top');
        zlabel('Height [mm]','fontsize', 15, 'interpreter', 'tex', 'verticalAlignment', 'bottom');
        title('Object Surface [GP Classification]', 'fontsize', 20, 'interpreter', 'tex');
        hold on
        scatter3(maxVarPoint(1), maxVarPoint(2),...
            max(max(ZTClassification)) + 0.05, 'fill', 'markerFaceColor', 'black', 'sizeData', [200]);
        
        %     scatter3(nextSamplePoint(1), nextSamplePoint(2), nextSamplePoint(3),...
        %         'fill', 'markerFaceColor', 'black', 'sizeData', [100]);
        hold off
        
        axis('equal');
        axis  tight
        view(viewVars);
        
    end
    
    
    ZVCombined = reshape(modelVarianceCombined, nPoints, nPoints);
    
    subplot(2,2,2)
    %figure(1)
    
    mesh(XT, YT, ZVCombined);
    set(gca, 'fontname', 'Bitstream Charter','fontsize', 15);
    xlabel('Width [mm]','fontsize', 15, 'interpreter', 'tex', 'verticalAlignment', 'bottom');
    ylabel('Length [mm]','fontsize', 15, 'interpreter', 'tex', 'verticalAlignment', 'top');
    zlabel('Height [mm]','fontsize', 15, 'interpreter', 'tex', 'verticalAlignment', 'bottom');
    title('Model Variance [GP Combined]', 'fontsize', 20, 'interpreter', 'tex');
    % axis('equal');
    axis  tight
    view(viewVars);
    
    hold on
    h_cp = scatter3(maxVarPoint(1), maxVarPoint(2),...
        max(max(ZVCombined)) + 0.05, 'fill', 'markerFaceColor', 'black', 'sizeData', [200]);
    
    %     scatter3(nextSamplePoint(1), nextSamplePoint(2), nextSamplePoint(3),...
    %         'fill', 'markerFaceColor', 'black', 'sizeData', [100]);
    hold off
    
    
    % %     hold on
    % %     h_cp = scatter3(trainingInput(:,1), trainingInput(:,2),...
    % %         trainingTarget(:,1), 'fill', 'markerFaceColor', 'blue', 'sizeData', [50]);
    % %     hold off
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %figure(3)
    subplot(2,2,3)
    x = trainingInput(1:end, 1);
    y = trainingInput(1:end, 2);
    z = trainingTarget(1:end, 1);
    
    
    xlin = linspace(min(x),max(x),40);
    ylin = linspace(min(y),max(y),40);
    [XT,YT] = meshgrid(xlin,ylin);
    
    f = scatteredInterpolant(x,y,z, 'natural');
    ZTNearestNeighbour = f(XT,YT);
    
    mesh(XT, YT, ZTNearestNeighbour);
    set(gca, 'fontname', 'Bitstream Charter','fontsize', 15);
    xlabel('Width [mm]','fontsize', 15, 'interpreter', 'tex', 'verticalAlignment', 'bottom');
    ylabel('Length [mm]','fontsize', 15, 'interpreter', 'tex', 'verticalAlignment', 'top');
    zlabel('Height [mm]','fontsize', 15, 'interpreter', 'tex', 'verticalAlignment', 'bottom');
    title('Object Surface [natural neighbour] ', 'fontsize', 20, 'interpreter', 'tex');
    axis('equal');
    axis tight;
    
    
    hold on
    if(exist(indexFingerFileName, 'file') ==2)
        scatter3(indexFingerLocations(:,1), indexFingerLocations(:,2), indexFingerLocations(:,3),...
            'fill', 'markerFaceColor', 'red', 'sizeData', [90]);
    end
    
    if(exist(middleFingerFileName, 'file') == 2)
        scatter3(middleFingerLocations(:,1), middleFingerLocations(:,2), middleFingerLocations(:,3),...
            'fill', 'markerFaceColor', 'blue', 'sizeData', [90]);
    end
    hold off
    view(viewVars);
    
    hold on
    scatter3(maxVarPoint(1), maxVarPoint(2),...
        max(max(ZTNearestNeighbour)), 'fill', 'markerFaceColor', 'black', 'sizeData', [100]);
    
    
    hold off
    
    
    if(isLatentValid)
        subplot(2,2,4)
        
        ngroups=5;
        x = trainingInput(81:end,1);
        y = trainingInput(81:end,2);
        z = latentVariables * 5;
        
        
        
        mesh(XT, YT, ZTNearestNeighbour);
        hold on;
        %z1=zeros(size(z,1),1);    % initial 'zldata'
        z1=indexFingerLocations(:,3);
        for i1=1:ngroups
            z2=z1;
            z1=z1+squeeze(z(:,i1));
            h(i1)=CREATESTACKEDMULTIBAR3d(x, y, z2, z1, i1.*ones(numel(z1(:)),1), 2, ngroups);
            
            hold on
        end
        hold off;
        set(gca, 'fontname', 'Bitstream Charter','fontsize', 15);
        xlabel('Width [mm]','fontsize', 15, 'interpreter', 'tex', 'verticalAlignment', 'bottom');
        ylabel('Length [mm]','fontsize', 15, 'interpreter', 'tex', 'verticalAlignment', 'top');
        title('Latent Variables', 'fontsize', 20, 'interpreter', 'tex');
        %legend(h, 'L 1','L 2','L 3','L 5','L 5');
        axis  tight equal;
        view([50, 35]);
        grid off; box off;
        
    end
    
    drawnow;
    
    %pause;
    %break;
end

