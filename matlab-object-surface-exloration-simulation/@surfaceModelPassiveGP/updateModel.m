function updateModel(this)
%fprintf('Called Passive GP update \n');
surfaceUncertainty = getSurfaceUncertaintyGP(this);
%surfaceUncertainty = normalise(surfaceUncertainty);
[~, maxUncertainty_idx] = max(surfaceUncertainty);
this.nextSamplingLocation = [this.inputTesting(maxUncertainty_idx,1), this.inputTesting(maxUncertainty_idx,2)];

%%% debugging display
if(this.plotDebug)
figNum = 4;
plotMesh(this, [this.inputTesting surfaceUncertainty], false, figNum, this.nPoints, 'Surface Uncertainty');
figNum = figNum + 1;
plotNextLocation(this, max(surfaceUncertainty), figNum -1);

end;


end











