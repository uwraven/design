function plotPosition(fig, t, x, x_ref)
    figure(fig); clf; hold on; grid on;
    plot(t, x(:, 1:3));
    plot(t, x_ref(:,1:3), '--');
    ylabel('Position')
    legend('x_{true}', 'y_{true}', 'z_{true}', 'x_{req}', 'y_{req}', 'z_{req}');
    title('Position') 
end