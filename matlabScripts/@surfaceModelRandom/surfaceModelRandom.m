classdef surfaceModelRandom < handle
    
    properties
        contactLocations; % Contact locations sampled so far
        nPoints = 80;
        objectName;
        nextSamplingLocation;
        referenceSurface;
        surfaceRMSE;
        nPadding;
        cornerPoints; 
        plotDebug = false;
        maxSamplePoints = 100;
        epochs = 1000;
        nholdouts = 1;
    end
    
    methods (Access = public)
        function obj = surfaceModelRandom(objectSurface, referenceSurface)
            addPaddingPoints(obj, objectSurface);
            generateCornerPoints(obj, objectSurface);
            obj.referenceSurface = referenceSurface;
            evaluateRMSE(obj);
            updateModel(obj);
        end
        function setNholdouts(this, nholdouts)
           this.nholdouts = nholdouts; 
        end
        function setExpochs(this, epochs)
           this.epochs = epochs; 
        end
            
        function nextSamplingLocation = getNextSamplingLocation(this)
            nextSamplingLocation = this.nextSamplingLocation;
        end
        function isDone = addContactLocation(this, contactLocation)
            this.contactLocations = [this.contactLocations; contactLocation];
            evaluateRMSE(this);
            updateModel(this);
            if((length(this.contactLocations) - this.nPadding) >= this.maxSamplePoints)
                isDone = true;
            else
                isDone = false;
            end
        end
        function enableDebugPlot(this, flag)
            this.plotDebug = flag;
        end
        
        function setMaxSamplePoints(this, maxSamplePoints)
           this.maxSamplePoints = maxSamplePoints; 
        end
        
        figNum = plotResults(this)
        getMesh(this, nConctacts, plotTitle, viewPars)
        function plotReferenceSurace(this, plotTitle, viewPars)
           plotMeshAll(this, this.referenceSurface, plotTitle, viewPars); 
        end
        function reEvalRMSE(this)
            reEvaluateRMSE(this);
        end
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










