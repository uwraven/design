classdef Sensor < Node

properties (Access = public)
    state = []
    bufferState = []
    variance = 0;
end

methods (Access = public)
    function self = Sensor(varargin)
        if length(varargin) >= 1
            self.variance = varargin{1};
        end
        if length(varargin) >= 2
            self.timer.interval = varargin{2};
        end
    end

    function setReadInterval(self, interval)
        self.timer.interval = interval;
    end

    function updatePhysicalState(self, newState)
        s = size(newState);
        self.bufferState = newState + wgn(s(1), s(2), self.variance);
    end

    function state = getState(self)
        state = self.state;
    end

    function onLoop(self, varargin)
        self.state = self.bufferState;
    end

end

methods (Access = private)

end

end