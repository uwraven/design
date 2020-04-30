classdef TrajectoryPlanner < handle
% Trajectory planning block
% Current model uses a simple low pass filter to limit the output trajectory
% based on a defined signal frequency wfc

properties (Access = public)
    targetCoordinate = [0 0 0]';
    filterCutoffFrequency = 0
end

properties (GetAccess = public, SetAccess = private)
    filteredCoordinate = [0 0 0]';
end

methods
    function self = TrajectoryPlanner()
        % constructor
    end

    function update(self, dt)
        a = 2 * pi * dt * self.filterCutoffFrequency
        self.filteredCoordinate = self.filteredCoordinate * (1 - a) + self.targetCoordinate * a;
    end


end

end