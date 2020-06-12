function plotGlobalInputs(fig, t, u, ug)
    figure(fig); clf; hold on; grid on;
    plot(t, ug(:, 1:3));
    plot(t, u(:, 1:3), '--');
    legend('f_{x, true}', 'f_{y, true}', 'f_{z, true}', 'f_{x, req}', 'f_{y, req}', 'f_{z, req}');
    title('global requested and realized forces')
end