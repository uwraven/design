classdef PWMController < handle
% PWM Controller provides a helper class to convert analog command signals to digitized impulse bits
% This is important for on / off systems like a binary solenoid valve
% It works by sampling at discrete intervals and comparing to a sawtooth wave
% If the command signal is greater than the sawtooth, turn on
% If the command signal is less than the sawtooth, turn off
% This produces a pwm output that has the similar impulse as the command but with a small phase delay due to sampling

properties (Access = public)
    output = 0
    reference = 0
end

properties (Access = private)
    % t tracks the interval elapsed time of the controller
    % y tracks the magnitude of the pwm sawtooth
    t = 0
    y = 0
    onTime = 0
    slope
    bitLength
    gain
    tHold = 0
end

methods (Access = public)
    function self = PWMController(bitLength, pwmGain)
        self.bitLength = bitLength;
        self.gain = pwmGain;
        self.slope = pwmGain / bitLength;
    end

    function setCommand(self, u)
        self.reference = abs(u);
    end

    function setBitLength(self, bitLength)
        self.bitLength = bitLength;
    end

    function setGain(self, pwmGain)
        self.gain = pwmGain;
    end

    function y = getY(self)
        y = self.y;
    end

    function tick(self, dt)
        % Integrate the sawtooth
        self.y = self.y + self.slope * dt;
        % self.tHold = self.tHold + dt;

        % desiredOutput = 0;

        % Compare the sawtooth to input signal
        if (self.y > self.reference)
            % PWM off
            self.output = 0;
        else 
            % output is binary, parent can use this information to set control gains
            self.output = 1;
        end

        % if (self.tHold > self.bitLength)
        %     self.output = desiredOutput;
        %     self.tHold = 0;
        % end

        if (self.y > self.gain)
            % Reset if y exceeds gain
            self.y = 0;
        end
        % For more:
        % https://en.wikipedia.org/wiki/Pulse-width_modulation
    end
end

end