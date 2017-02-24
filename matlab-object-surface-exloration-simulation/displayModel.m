%cd('/home/nawidj/software/src/object-surface-exploration/build');
cd('/home/nawidj/gpdata')
%cd('/home/nawidj/gpDataTrial06')
%cd('/home/nawidj/tacman/GaussianSurfaceExplorationData/data/hut/set02/trial05/gpPoints');

objectName = 'hut';
nextPointFileName = [objectName '_model_nextPoint.csv'];
modelInputFileName = [objectName '_model_input.csv'];
modelOutputFileName = [objectName '_model_output_GP.csv'];
modelVarianceFileName = [objectName '_model_variance_GP.csv'];
trainingInputFileName = [objectName '_training_input_GP.csv'];
trainingTargetFileName = [objectName '_training_target_GP.csv'];
nextSamplePointFileName = [objectName '_nextPoint.csv'];

%viewVars = [45, 55];
%viewVars = [90, 90];
viewVars = [50, 35];
clf('reset');
while(true)
    
    while(~exist(nextPointFileName))
        pause(0.05);
    end
    
    mmFactor = 1000;
    
    maxVarPoint = dlmread(nextPointFileName);
    
    
    
    delete(nextPointFileName);
    
    system('python taxel2latent.py');
    
    
    
    
    
    nPoints = 20;
    
    modelInput = dlmread(modelInputFileName);
    modelOutput = dlmread(modelOutputFileName);
    modelVariance = dlmread(modelVarianceFileName);
    trainingInput = dlmread(trainingInputFileName);
    trainingTarget = dlmread(trainingTargetFileName);
    latentVariables = dlmread('latent.csv');
    
    maxVarPoint(1) = maxVarPoint(1) - min(modelInput(:,1));
    maxVarPoint(2) = maxVarPoint(2) - min(modelInput(:,2));
    maxVarPoint(3) = maxVarPoint(3) - min(modelOutput);
    maxVarPoint = maxVarPoint * mmFactor;
    fprintf('Next sampling point is: (%0.4f, %0.4f. %0.4f)\n', maxVarPoint(1), maxVarPoint(2), maxVarPoint(3));
    
    %nextSamplePoint = dlmread(nextSamplePointFileName);
    
    % in mms
    %%%%maxVarPoint(3) = maxVarPoint(3) - min(modelOutput);
    %nextSamplePoint(3) = nextSamplePoint(3) - min(modelOutput);
    
    modelInput = (modelInput - repmat(min(modelInput), length(modelInput), 1)) * mmFactor;
    modelOutput = (modelOutput - min(modelOutput)) * mmFactor;
    modelVariance = modelVariance * mmFactor;
    trainingInput = (trainingInput - repmat(min(trainingInput), length(trainingInput), 1)) * mmFactor;
    trainingTarget = (trainingTarget - min(trainingTarget)) * mmFactor;
    
   
    
    x = modelInput(:, 1);
    y = modelInput(:, 2);
    
    fprintf('Xmin: %f, Xmax: %f, Ymin: %f, Ymax: %f\n',...
        min(x), max(x), min(y), max(y));
    
    xlin = linspace(min(x), max(x), nPoints);
    ylin = linspace(min(y), max(y), nPoints);
    [XT,YT] = meshgrid(xlin,ylin);
    ZT = reshape(modelOutput, nPoints, nPoints);
    ZV = reshape(modelVariance, nPoints, nPoints);
    
    figH = figure(1);
    %clf('reset');
    set(figH, 'color', 'white');
    
    subplot(2,2,1)
    %figure(1)
    
    mesh(XT, YT, ZT);
    set(gca, 'fontname', 'Bitstream Charter','fontsize', 15);
    xlabel('Width [mm]','fontsize', 15, 'interpreter', 'tex', 'verticalAlignment', 'bottom');
    ylabel('Length [mm]','fontsize', 15, 'interpreter', 'tex', 'verticalAlignment', 'top');
    zlabel('Height [mm]','fontsize', 15, 'interpreter', 'tex', 'verticalAlignment', 'bottom');
    title('Object Surface [Gaussian Process]', 'fontsize', 20, 'interpreter', 'tex');
    axis('equal');
    axis  tight
    view(viewVars);
    
    hold on
       h_cp = scatter3(maxVarPoint(1), maxVarPoint(2),...
           maxVarPoint(3), 'fill', 'markerFaceColor', 'black', 'sizeData', [100]);
    
    %     scatter3(nextSamplePoint(1), nextSamplePoint(2), nextSamplePoint(3),...
    %         'fill', 'markerFaceColor', 'black', 'sizeData', [100]);
    hold off
    
    
    hold on
    h_cp = scatter3(trainingInput(:,1), trainingInput(:,2),...
        trainingTarget(:,1), 'fill', 'markerFaceColor', 'blue', 'sizeData', [50]);
    hold off
    
    subplot(2,2,2)
    %figure(2)
    mesh(XT,YT, ZV);
    set(gca, 'fontname', 'Bitstream Charter','fontsize', 15);
    xlabel('Width [mm]','fontsize', 15, 'interpreter', 'tex', 'verticalAlignment', 'bottom');
    ylabel('Length [mm]','fontsize', 15, 'interpreter', 'tex', 'verticalAlignment', 'top');
    zlabel('Uncertainty','fontsize', 15, 'interpreter', 'tex', 'verticalAlignment', 'bottom');
    title('GP Variance', 'fontsize', 20, 'interpreter', 'tex');
    %zlim([0 5]);
    %axis('equal');
    axis tight;
    view(viewVars);
    
    hold on
       h_cp = scatter3(maxVarPoint(1), maxVarPoint(2),...
           maxVarPoint(4), 'fill', 'markerFaceColor', 'black', 'sizeData', [100]);
    
    %     scatter3(nextSamplePoint(1), nextSamplePoint(2), nextSamplePoint(3),...
    %         'fill', 'markerFaceColor', 'black', 'sizeData', [100]);
    hold off
    
    % %     hold on
    % %     h_cp = scatter3(maxVarPoint(1), maxVarPoint(2),...
    % %         maxVarPoint(4), 'fill', 'markerFaceColor', 'green', 'sizeData', [100]);
    % %     hold off
    
    
    %figure(3)
    subplot(2,2,3)
    x = trainingInput(1:end, 1);
    y = trainingInput(1:end, 2);
    z = trainingTarget(1:end, 1);
    
    
    xlin = linspace(min(x),max(x),40);
    ylin = linspace(min(y),max(y),40);
    [XT,YT] = meshgrid(xlin,ylin);
    
    f = scatteredInterpolant(x,y,z, 'natural');
    ZT = f(XT,YT);
    
    mesh(XT, YT, ZT);
    set(gca, 'fontname', 'Bitstream Charter','fontsize', 15);
    xlabel('Width [mm]','fontsize', 15, 'interpreter', 'tex', 'verticalAlignment', 'bottom');
    ylabel('Length [mm]','fontsize', 15, 'interpreter', 'tex', 'verticalAlignment', 'top');
    zlabel('Height [mm]','fontsize', 15, 'interpreter', 'tex', 'verticalAlignment', 'bottom');
    title('Object Surface [natural neighbour] ', 'fontsize', 20, 'interpreter', 'tex');
    axis('equal');
    axis tight;
    hold on
    scatter3(x, y, z,...
        'fill', 'markerFaceColor', 'blue', 'sizeData', [90]);
    hold off
    view(viewVars);
    
    hold on
%        h_cp = scatter3(maxVarPoint(1), maxVarPoint(2),...
%            maxVarPoint(3), 'fill', 'markerFaceColor', 'black', 'sizeData', [100]);
    
    %     scatter3(nextSamplePoint(1), nextSamplePoint(2), nextSamplePoint(3),...
    %         'fill', 'markerFaceColor', 'black', 'sizeData', [100]);
    hold off
    
    
%     subplot(2,2,4)
%     
%     ngroups=5;
%     x = trainingInput(2:end,1);
%     y = trainingInput(2:end,2);
%     z = latentVariables* 10;
%     
%     
%     
%     z1=zeros(size(z,1),1);    % initial 'zldata'
%     for i1=1:ngroups
%         z2=z1;
%         z1=z1+squeeze(z(:,i1));
%         h(i1)=CREATESTACKEDMULTIBAR3d(x, y, z2, z1, i1.*ones(numel(z1(:)),1), 2, ngroups);
%         hold on
%     end
%     hold off;
%     set(gca, 'fontname', 'Bitstream Charter','fontsize', 15);
%     xlabel('Width [mm]','fontsize', 15, 'interpreter', 'tex', 'verticalAlignment', 'bottom');
%     ylabel('Length [mm]','fontsize', 15, 'interpreter', 'tex', 'verticalAlignment', 'top');
%     title('Latent Variables', 'fontsize', 20, 'interpreter', 'tex');
%     %legend(h, 'L 1','L 2','L 3','L 5','L 5');
%     axis  tight equal;
%     view([50, 35]);
%     grid off; box off;
    
    drawnow;
    
    
    
end

