classdef RateController < handle

properties
    Pq = diag(ones(3, 1))
    Pw = diag(ones(3, 1))
end

methods
    function self = RateController()
        
    end

    function U = inputs(self, qv, w)
        U = -self.Pq * qv - self.Pw * w;
    end

    function setGains(self, Pq, Pw)
        self.Pq = Pq;
        self.Pw = Pw;
    end
end

end