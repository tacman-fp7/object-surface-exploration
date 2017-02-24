function updateModel(this)
%fprintf('Called Ramdom update\n');
maxX = max(this.contactLocations(:,1));
minX = min(this.contactLocations(:,1));
maxY = max(this.contactLocations(:,2));
minY = min(this.contactLocations(:,2));

this.nextSamplingLocation = (rand(1,2) .* [maxX - minX, maxY - minY]) +...
    [minX, minY];
end
