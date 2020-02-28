classdef Sensor < Node

% The sensor class provides a way to read state and store it on timed intervals
% simulating the sample rate found in real sensors. In addition, it can add 
% noise to the signal based on a fixed variance.

% TODO
% This should perform some sort of interpolation or averaging of the physical state
% during the measurement interval instead of returning the last recorded value 

properties (Access = public)
    state = []
    bufferState = []
    variance = 0;
end

methods (Access = public)
    % Initialize an instance of Sensor
    function self = Sensor(varargin)
        if length(varargin) >= 1
            self.variance = varargin{1};
        end
        if length(varargin) >= 2
            self.timer.interval = varargin{2};
        end
    end

    % Set the interval on which the sensor updates data
    function setReadInterval(self, interval)
        self.timer.interval = interval;
    end

    % Update the physical state recorded by the sensor and add variance to the signal
    function updatePhysicalState(self, newState)
        s = size(newState);
        self.bufferState = newState + wgn(s(1), s(2), self.variance);
    end

    % Get the most recently measured state
    function state = getState(self)
        state = self.state;
    end

    % Update the value of the state returned by the sensor if time interval is complete
    % This is the actual data 'sample' returned
    % Note that onLoop() is only called on the timing interval
    function onLoop(self)
        self.state = self.bufferState;
    end

end

methods (Access = private)

end

end