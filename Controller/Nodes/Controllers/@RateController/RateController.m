classdef RateController < handle

properties
    Pq = diag(zeros(1, 3));
    Pw = diag(zeros(1, 3));
end

methods
    function self = RateController()
    end

    function U = inputs(self, qv, w)
        U = -Pq * qv - Pw * w;
    end

    function setGains(self, Pq, Pw)
        self.Pq = Pq;
        self.Pw = Pw;
    end
end

end