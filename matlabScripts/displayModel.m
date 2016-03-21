cd('/home/nawidj/software/src/object-surface-exploration/build');
objectName = 'hut';
nextPointFileName = [objectName '_model_nextPoint.csv'];
modelInputFileName = [objectName '_model_input.csv'];
modelOutputFileName = [objectName '_model_output_GP.csv'];
modelVarianceFileName = [objectName '_model_variance_GP.csv'];
trainingInputFileName = [objectName '_training_input_GP.csv'];
trainingTargetFileName = [objectName '_training_target_GP.csv'];
nextSamplePointFileName = [objectName '_nextPoint.csv'];
viewVars = [45, 55];
%viewVars = [90, 90];

while(true)
    
    while(~exist(nextPointFileName))
        pause(0.05);
    end
    
    maxVarPoint = dlmread(nextPointFileName);
    fprintf('Next sampling point is: (%0.4f, %0.4f. %0.4f)\n', maxVarPoint(1), maxVarPoint(2), maxVarPoint(3));
    
    delete(nextPointFileName);
    
    
    
    nPoints = 20;
    
    modelInput = dlmread(modelInputFileName);
    modelOutput = dlmread(modelOutputFileName);
    modelVariance = dlmread(modelVarianceFileName);
    trainingInput = dlmread(trainingInputFileName);
    trainingTarget = dlmread(trainingTargetFileName);
    %nextSamplePoint = dlmread(nextSamplePointFileName);
    
    x = modelInput(:, 1);% - min(inputTesting(:, 1));
    y = modelInput(:, 2);% - min(inputTesting(:, 2));
    
    fprintf('Xmin: %f, Xmax: %f, Ymin: %f, Ymax: %f\n',...
        min(x), max(x), min(y), max(y));
    
    xlin = linspace(min(x), max(x), nPoints);
    ylin = linspace(min(y), max(y), nPoints);
    [XT,YT] = meshgrid(xlin,ylin);
    maxVarPoint(3) = maxVarPoint(3) - min(modelOutput);
    %nextSamplePoint(3) = nextSamplePoint(3) - min(modelOutput);
    modelOutput = modelOutput - min(modelOutput);
    
    ZT = reshape(modelOutput, nPoints, nPoints);
    ZV = reshape(modelVariance, nPoints, nPoints);
    
    figure(1)
    mesh(XT, YT, ZT);
%     set(gca, 'fontname', 'Bitstream Charter','fontsize', 35);
%     xlabel('Width [mm]','fontsize', 35, 'interpreter', 'tex', 'verticalAlignment', 'bottom');
%     ylabel('Length [mm]','fontsize', 35, 'interpreter', 'tex', 'verticalAlignment', 'top');
%     zlabel('Height [mm]','fontsize', 35, 'interpreter', 'tex', 'verticalAlignment', 'bottom');
%     title('Surface', 'fontsize', 35, 'interpreter', 'tex');
    view(viewVars);
    
    hold on
    h_cp = scatter3(maxVarPoint(1), maxVarPoint(2),...
        maxVarPoint(3), 'fill', 'markerFaceColor', 'black', 'sizeData', [100]);
    
%     scatter3(nextSamplePoint(1), nextSamplePoint(2), nextSamplePoint(3),...
%         'fill', 'markerFaceColor', 'black', 'sizeData', [100]);
    hold off
    
    trainingTarget = trainingTarget - min(trainingTarget);
     hold on
    h_cp = scatter3(trainingInput(:,1), trainingInput(:,2),...
        trainingTarget(:,1), 'fill', 'markerFaceColor', 'blue', 'sizeData', [50]);
    hold off
    
    
    figure(2)
    mesh(XT,YT, ZV);
%     set(gca, 'fontname', 'Bitstream Charter','fontsize', 35);
%     xlabel('Width [mm]','fontsize', 35, 'interpreter', 'tex', 'verticalAlignment', 'bottom');
%     ylabel('Length [mm]','fontsize', 35, 'interpreter', 'tex', 'verticalAlignment', 'top');
%     zlabel('Variance [mm]','fontsize', 35, 'interpreter', 'tex', 'verticalAlignment', 'bottom');
%     title('Variance', 'fontsize', 35, 'interpreter', 'tex');
    view(viewVars);
    
    
% %     hold on
% %     h_cp = scatter3(maxVarPoint(1), maxVarPoint(2),...
% %         maxVarPoint(4), 'fill', 'markerFaceColor', 'green', 'sizeData', [100]);
% %     hold off
    
    %regressGPML([trainingInput(1:end,:) trainingTarget(1:end, :)], viewVars);
    
    figure(3)
    x = trainingInput(1:end, 1);
y = trainingInput(1:end, 2);
z = trainingTarget(1:end, 1);


xlin = linspace(min(x),max(x),40);
ylin = linspace(min(y),max(y),40);
[XT,YT] = meshgrid(xlin,ylin);

f = scatteredInterpolant(x,y,z, 'natural');
ZT = f(XT,YT);

mesh(XT, YT, ZT);

hold on
scatter3(x, y, z,...
    'fill', 'markerFaceColor', 'blue', 'sizeData', [90]);
hold off
  view(viewVars);  
    drawnow;
    
end
