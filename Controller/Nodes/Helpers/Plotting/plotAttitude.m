function plotAttitude(fig, t, q)
    figure(fig); clf; hold on; grid on;
    plot(t, q);
    legend('q_0', 'q_1', 'q_2', 'q_3');
    title('Attitude');
end