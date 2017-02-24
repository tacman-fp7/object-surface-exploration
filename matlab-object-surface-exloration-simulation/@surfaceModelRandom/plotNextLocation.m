function plotNextLocation(this, zPos, figNum)

figure(figNum);
hold on;
scatter3(this.nextSamplingLocation(1), this.nextSamplingLocation(2), zPos, ...
    'fill', 'markerFaceColor', 'red', 'sizeData', [100]);
hold off;

end