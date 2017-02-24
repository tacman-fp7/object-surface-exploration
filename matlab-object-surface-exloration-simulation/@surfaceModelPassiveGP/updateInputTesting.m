function updateInputTesting(this)
xlin = linspace(min(this.contactLocations( : , 1)), max(this.contactLocations( : , 1)), this.nPoints);
ylin = linspace(min(this.contactLocations( : , 2)), max(this.contactLocations( : , 2)), this.nPoints);

this.inputTesting = [];
for i = 1:length(xlin)
    this.inputTesting =  [this.inputTesting; repmat(xlin(i), length(ylin), 1) ylin'];
end
end