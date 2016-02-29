cd('/home/nawidj/software/src/object-surface-exploration/build');
objectName = 'hut';
nextPointFileName = [objectName '_model_nextPoint.csv'];
modelInputFileName = [objectName '_model_input.csv'];
modelOutputFileName = [objectName '_model_output_GP.csv'];
modelVarianceFileName = [objectName '_model_variance_GP.csv'];
viewVars = [140, 35];

while(true)
    
    while(~exist(nextPointFileName))
        pause(0.05);
    end
    
    maxVarPoint = dlmread(nextPointFileName);
    fprintf('Next sampling point is: (%0.4f, %0.4f. %0.4f)\n', maxVarPoint(1), maxVarPoint(2), maxVarPoint(3));
    
    delete(nextPointFileName);
    
    
    
    nPoints = 120;
    
    modelInput = dlmread(modelInputFileName);
    modelOutput = dlmread(modelOutputFileName);
    modelVariance = dlmread(modelVarianceFileName);
    
    x = modelInput(:, 1);% - min(inputTesting(:, 1));
    y = modelInput(:, 2);% - min(inputTesting(:, 2));
    xlin = linspace(min(x), max(x), nPoints);
    ylin = linspace(min(y), max(y), nPoints);
    [XT,YT] = meshgrid(xlin,ylin);
    
    ZT = reshape(modelOutput, nPoints, nPoints);
    ZV = reshape(modelVariance, nPoints, nPoints);
    
    figure(1)
    mesh(XT, YT, ZT);
    view(viewVars);
    
    hold on
    h_cp = scatter3(maxVarPoint(1), maxVarPoint(2),...
        maxVarPoint(3), 'fill', 'markerFaceColor', 'blue', 'sizeData', [100]);
    hold off
    
    figure(2)
    mesh(XT,YT, ZV);
    view(viewVars);
    
    
    hold on
    h_cp = scatter3(maxVarPoint(1), maxVarPoint(2),...
        maxVarPoint(4), 'fill', 'markerFaceColor', 'blue', 'sizeData', [100]);
    hold off
    
    drawnow;
    
end
