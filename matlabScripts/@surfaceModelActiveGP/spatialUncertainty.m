function dist = getSpatialUncertainty(this)

tau = -900;

% input testing has the testing values
dist = zeros(length(this.inputTesting), 1)+1;

for i = 1:this.nPadding -1
    dist = dist .* (1 - exp(tau * getDistance(this.inputTesting, this.contactLocations(i,1:2))));
end

tau = -500;
for i = this.nPadding: length(this.contactLocations)
    
    %Calculate the distance from the location
    dist = dist .* (1 - exp(tau * getDistance(this.inputTesting, this.contactLocations(i,1:2))));
end

dist = (dist - min(dist))/ (max(dist) - min(dist));

%plotMesh(this, [this.inputTesting, dist ], false, 2, this.nPoints, 'Distance');

end

function distance = getDistance(X, Y)
distance = sqrt((X(:,1) - Y(:,1)).^2 + (X(:,2) - Y(:,2)).^2);
end