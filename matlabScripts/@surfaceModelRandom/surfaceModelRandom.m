classdef surfaceModelRandom < handle
    
    properties
        contactLocations; % Contact locations sampled so far
        nPoints = 60;
        objectName;
        nextSamplingLocation;
        referenceSurface;
        surfaceRMSE;
        nPadding;
        cornerPoints; 
        plotDebug = false;
    end
    
    methods (Access = public)
        function obj = surfaceModelRandom(objectSurface, referenceSurface)
            addPaddingPoints(obj, objectSurface);
            generateCornerPoints(obj, objectSurface);
            obj.referenceSurface = referenceSurface;
            evaluateRMSE(obj);
            updateModel(obj);
        end
        function nextSamplingLocation = getNextSamplingLocation(this)
            nextSamplingLocation = this.nextSamplingLocation;
        end
        function addContactLocation(this, contactLocation)
            this.contactLocations = [this.contactLocations; contactLocation];
            evaluateRMSE(this);
            updateModel(this);
        end
        function enableDebugPlot(this, flag)
            this.plotDebug = flag;
        end
        
        figNum = plotResults(this)
    end
    
    methods (Access = protected)
        updateModel(this)
        evaluateRMSE(this)
        addPaddingPoints(this, objectSurface)
        addPaddingPointsCorners(this, objectSurface)
        generateCornerPoints(this, objectSurface)
        plotMesh(this, contactLocations, plotPC, figNum, nPoints, plotTitle) 
        plotNextLocation(this, zPos, figNum)
    end
end










