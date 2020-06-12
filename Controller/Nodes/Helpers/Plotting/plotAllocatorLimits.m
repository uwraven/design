function plotAllocatorLimits(fig, t, lims)
    figure(fig); hold on;
    for i = 1:length(lims(:,1))
        plot([0 t(end)], [lims(i,1) lims(i,1)])
        plot([0 t(end)], [lims(i,2) lims(i,2)])
    end
end