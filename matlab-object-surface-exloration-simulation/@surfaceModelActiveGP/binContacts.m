function contactBins = binContacts(this)

nBins = this.nBins;

binSize = (max(this.contactLocations(:, 3)) - min(this.contactLocations(:, 3))) / (nBins + 1);
contactBins = zeros(length(this.contactLocations(:,3)), nBins) - 1;

for i = 1:length(this.contactLocations(:, 3))
    height = this.contactLocations(i, 3) - min(this.contactLocations(:, 3));
    
    for j = 1:nBins
        if(height < binSize * j)
            contactBins(i, j) = 1;
            break;
        end
    end
end

% Check that each bin has at least one positive and one negative value
% GURLS fails if there is none. They also have to be mutually exclusive

for j = 1:nBins
    
    if(sum(ismember(contactBins(:, j), 1)) == 0)
        contactBins(j,j) = 1; % Changing diagonally to make sure they are mutually exclusive
        
    end
end

if(nBins == 1)
    contactBins = contactBins * -1; % for asthetics
end

end



