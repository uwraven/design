classdef TrajectoryPlanner < handle
% Trajectory planning block
% Current model uses a simple low pass filter to limit the output trajectory
% based on a defined signal frequency wfc

properties (Access = public)
    targetCoordinate = [0 0 0]';
    targetCoordinateIndex = 0;
end

properties (Access = private)
    targets = [0 0 0 0];
    t = 0
end

properties (GetAccess = public, SetAccess = private)

end

methods
    
    function self = TrajectoryPlanner()
        % constructor
    end

    function update(self, dt)
        self.t = self.t + dt;
        if self.targetCoordinateIndex < length(self.targets(:,1))
            if (self.t > self.targets(self.targetCoordinateIndex + 1, 1))
                self.targetCoordinateIndex = self.targetCoordinateIndex + 1;
                self.targetCoordinate = self.targets(self.targetCoordinateIndex, 2:4)';
            end
        end
    end

    function setTargets(self, targets)
        self.targets = targets;
        self.targetCoordinate = targets(1, 2:4)'
    end

    function resetClock(self)
        self.t = 0;
    end

end

end