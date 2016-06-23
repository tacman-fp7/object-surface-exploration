classdef surfaceModelPassiveGP < surfaceModelRandom
    
    properties
        inputTesting;
    end
    methods (Access = public)
        function obj = surfaceModelPassiveGP(objectSurface, referenceSurface)
            obj@surfaceModelRandom(objectSurface, referenceSurface);
            
        end
    end
    
    methods (Access = protected)
        updateModel(this)
       updateInputTesting(this)
       surfaceUncertainty = getSurfaceUncertaintyGP(this)
    end
end



