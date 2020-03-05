function trace3D(x, u)
    % Assume r in R3, u in R6

    U = zeros(length(u(:,1)), 3);
    
    for i = 1:length(x(:,1))
        % Convert thrust vector to body frame
        U(i, 1:3) = reshape(Quaternion.rotateBy(u(i, 1:3), x(i, 7:10)), 1, 3);
    end

    quiver3(x(:,3), x(:,2), x(:,1), U(:,3), U(:,2), U(:,1), 0.5);

    % for i = 1:length(t)
    %     % Get state at current time
    %     r = reshape(x(i, 1:3), 1, 3);
    %     q = reshape(x(i, 7:10), 1, 4);

    %     % Create a cylinder mesh
    %     [XC, YC, ZC] = cylinder(0.1);

    %     % Get body input vectors
    %     thrust = u(1:3);
    %     rcs = u(4:6);



    % end

end