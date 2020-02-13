classdef Timer < handle

properties (Access = public)
    interval uint64 = 1
end

properties (Access = private)
    localTime uint64 = 0
    loopMethod
end

methods (Access = public)

    function self = Timer(varargin)
        if ~isempty(varargin)
            self.interval = varargin{1};
        end
    end

    % Advance forward in time by one interval
    function tick(self, varargin)
        self.localTime = self.localTime + 1;
        if self.localTime == self.interval
            % If enough time has passed, then call the super loop method
            % and reset the timer
            self.loopMethod(varargin);
            self.localTime = 0;
        end
    end

    function setLoopMethodHandle(self, methodHandle)
        self.loopMethod = methodHandle;
    end

    function setInterval(self, interval)
        self.interval = interval;
    end

    function resetLoop(self)
        self.localTime = 0;
    end

end

end
