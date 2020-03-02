classdef Quaternion
% Instance of a quaternion
% Be careful using this class, it is slower to instantiate and allocate in memory than just an array
% Use the optimized, array based static methods to compute quaternion products and integrals during runtime

properties (Access = public)

end

properties (Access = private)
    arr
end

properties (Dependent)
    w
    x
    y
    z
    v % vector component of quaternion
    q % return the quaternion as a column vector
end

methods (Access = public)

    function self = Quaternion(arr)
        self.arr = reshape(arr, 4, 1);
    end

    function self = normalize(self)
        self.arr = self.arr / norm(self.arr);
    end

    function n = norm(self)
        n = norm(self.arr);
    end

end

methods
    % Operator overloads
    function q = plus(q1, q2)
        q = Quaternion([
            q1.w + q2.w
            q1.x + q2.x
            q1.y + q2.y
            q1.z + q2.z
        ]);
    end

    function q = times(q1, q2)
        % Element wise multiplication
        q = Quaternion([
            q1.w * q2.w
            q1.x * q2.x
            q1.y * q2.y
            q1.z * q2.z
        ]);
    end

    function q = mtimes(q1, q2)
        q = Quaternion(Quaternion.quatProduct(q1, q2));
    end

    function q = minus(q1, q2)
        q = q1 + (-1 * q2);
    end

    function q = rdivide(q1, a)
        q = Quaternion([
            q1.w / a
            q1.x / a
            q1.y / a
            q1.z / a
        ]);
    end


    % Property getters
    % These provide _slow_ access to member properties
    % consider accessing self.arr
    function q = get.q(self)
        q = self.arr;
    end

    function v = get.v(self)
        v = self.arr(2:4);
    end

    function w = get.w(self)
        w = self.arr(1);
    end

    function x = get.x(self)
        x = self.arr(2);
    end

    function y = get.y(self)
        y = self.arr(3);
    end

    function z = get.z(self)
        z = self.arr(4);
    end
end

methods (Static)

    function X = integrateAttitude(X0, a, dt)
        % from an initial state X0 (q and w) compute change in attitude / rates
        % from an applied angular acceleration over dt
        X = RK4(@(X, dt) [1 / 2 * Quaternion.skew(X(5:7)) * X(1:4); a], X0, dt);
        X(1:4) = X(1:4) / norm(X(1:4));
    end

    function qr = quatProductArr(q1, q2)
        qr = [
            q1(1) * q2(1) - q1(2) * q2(2) - q1(3) * q2(3) - q1(4) * q2(4)
            q1(1) * q2(2) + q1(2) * q2(1) + q1(3) * q2(4) - q1(4) * q2(3)
            q1(1) * q2(3) - q1(2) * q2(4) + q1(3) * q2(1) + q1(4) * q2(2)
            q1(1) * q2(4) + q1(2) * q2(3) - q1(3) * q2(2) + q1(4) * q2(1)
        ];
    end

    function wr = vecProductArr(q1, w)
        wr = [
            q1(1) * w(1) + q1(3) * w(3) - q1(4) * w(2)
            q1(1) * w(2) - q1(2) * w(3) + q1(4) * w(1)
            q1(1) * w(3) + q1(2) * w(2) - q1(3) * w(1)
        ];
    end

    function W = skew(w)
        % From an angular rate vector, return the skew symmetric rate matrix
        % This shouldn't be implemented in the Quaternion class, but Matlab is being dumb with paths, so here we are
        % TODO:: put this somewhere else
        W = [
            0       -w(3)   w(2)    w(1)
            w(3)    0       -w(1)   w(2)
            -w(2)   w(1)    0       w(3)
            -w(1)   -w(2)   -w(3)   0
        ];
    end

    % These methods utilize the member get properties and are generally slower
    function qr = quatProduct(q1, q2)
        qr = [
            q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z
            q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y
            q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x
            q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w
        ];
    end

    function wr = vecProduct(q1, v)
        qv = Quaternion([0; v]);
        wr = quatProduct(q1, qv);
    end

    
end


end