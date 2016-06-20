classdef surfaceModelActiveGP < surfaceModelPassiveGP
    
    properties
        nBins = 1;
        firstBinThreshold = 10;
    end
    
    methods (Access = public)
        function obj = surfaceModelActiveGP(objectSurface, referenceSurface)
           obj@surfaceModelPassiveGP(objectSurface, referenceSurface); 
        end
        figNum = plotResults(this)
        
    end
    methods (Access = protected)
        
        updateModel(this)
        
        function plotDecay(this)
            spatialCoverageUncertainty(this);
        end
    end
    
    methods (Access = private)
       contactBins = binContacts(this)
       dist = getSpatialUncertainty(this)
    end
end

