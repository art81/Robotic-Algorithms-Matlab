function [] = makeDensityPlot(x)
    for i = 1:numel(x)
        line([x(i) x(i)], [0 1]);
    end
end