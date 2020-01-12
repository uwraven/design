classdef Timer < handle

properties (Access = public)
    interval uint16 = 1
end

properties (Access = private)
    localTime uint16 = 0
    loopMethod
end

methods (Access = public)

    function self = Timer(varargin)
        if ~isempty(varargin)
            self.interval = varargin{1};
        end
    end

    function tick(self, varargin)
        self.localTime = self.localTime + 1;
        if self.localTime == self.interval
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
