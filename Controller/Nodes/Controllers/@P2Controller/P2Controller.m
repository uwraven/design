classdef P2Controller < handle

properties
    Pq = diag(zeros(1, 3));
    Pw = diag(zeros(1, 3));
end

methods
    function self = P2Controller()
    end

    function U = inputs(self, qv, w)
        U = -Pq * qv - Pw * w;
    end
end

end