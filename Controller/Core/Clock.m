classdef Clock < handle

properties (Access = public)

end

properties (Access = private)
    tickRate uint64 % frequency
    elapsedTime uint64 = 0
    paused logical = 0
end

methods (Access = public)

    function self = Clock(varargin)
        % User can pass string or # to set clock frequency
        if ~isempty(varargin)
            if isstring(varargin{1})
                switch(varargin{1})
                case "us"
                    self.tickRate = 1e6;
                case "ms"
                    self.tickRate = 1e3;
                case "s"
                    self.tickRate = 1;
                end
            else
                self.tickRate = varargin{1};
            end
        else
            
        end
    end

    function tick(self)
        self.elapsedTime = self.elapsedTime + 1;
    end

    function time = getElapsedTime(self)
        time = self.elapsedTime;
    end
    
    function setTickRate(self, rate)
        self.tickRate = rate;
    end

    function rate = getTickRate(self)
        rate = self.tickRate;
    end

end

end
