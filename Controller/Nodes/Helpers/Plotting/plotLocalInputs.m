function plotGlobalInputs(figs, t, u, ug, lim)
    figure(figs(1)); clf; hold on; grid on;
    plot(t, ug(:, 1:3));
    plot(t, u(:, 1:3), '--');
    colors = {'#0072BD', '#D95319', '#EDB120', '#7E2F8E', '#77AC30', '#4DBEEE'};
    for i = 1:length(lim(:,1))
        plot([0 t(end)], [lim(i, 1) lim(i, 1)], '-.', 'Color', colors{i})
        plot([0 t(end)], [lim(i, 2) lim(i, 2)], '-.', 'Color', colors{i})
    end
    legend('f_{x, true}', 'f_{y, true}', 'f_{z, true}', 'f_{x, req}', 'f_{y, req}', 'f_{z, req}', 'f_{ex, lim}', '', 'f_{ey, lim}', '', 'f_{ez, lim}', '', 'f_{r1, lim}', '', 'f_{r2, lim}', '', 'f_{r3, lim}', '');
    title('local requested and realized forces')

    figure(figs(2)); clf; hold on; grid on;
    plot(t, ug(:, 4:6));
    plot(t, u(:, 4:6), '--');
    legend('M_{x, true}', 'M_{y, true}', 'M_{z, true}', 'M_{x, req}', 'M_{y, req}', 'M_{z, req}');
    title('local requested and realized moments')
    
end