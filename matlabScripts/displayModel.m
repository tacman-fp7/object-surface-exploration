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
    
    fprintf('Xmin: %f, Xmax: %f, Ymin: %f, Ymax: %f\n',...
        min(x), max(x), min(y), max(y));
    
    xlin = linspace(min(x), max(x), nPoints);
    ylin = linspace(min(y), max(y), nPoints);
    [XT,YT] = meshgrid(xlin,ylin);
    maxVarPoint(3) = maxVarPoint(3) - min(modelOutput);
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
    %view(viewVars);
    
    hold on
    h_cp = scatter3(maxVarPoint(1), maxVarPoint(2),...
        maxVarPoint(3), 'fill', 'markerFaceColor', 'blue', 'sizeData', [100]);
    hold off
    
    figure(2)
    mesh(XT,YT, ZV);
%     set(gca, 'fontname', 'Bitstream Charter','fontsize', 35);
%     xlabel('Width [mm]','fontsize', 35, 'interpreter', 'tex', 'verticalAlignment', 'bottom');
%     ylabel('Length [mm]','fontsize', 35, 'interpreter', 'tex', 'verticalAlignment', 'top');
%     zlabel('Variance [mm]','fontsize', 35, 'interpreter', 'tex', 'verticalAlignment', 'bottom');
%     title('Variance', 'fontsize', 35, 'interpreter', 'tex');
    %view(viewVars);
    
    
    hold on
    h_cp = scatter3(maxVarPoint(1), maxVarPoint(2),...
        maxVarPoint(4), 'fill', 'markerFaceColor', 'blue', 'sizeData', [100]);
    hold off
    
    drawnow;
    
end
