function [ OS_table ] = SincInit( bw, cbw, spb, ratio)
% OS_table = SincInit(bw,cbw,spb,ratio)
% This script generates an oversampled Sinc pulse
% Outputs :
%   OS_table, the oversampled sinc pulse
% Inputs :
%   bw, normalized bandwidth of the pulse
%   cbw, normalized carrier bandwidth of the pulse
%   spb, number of samples per buffer in the downsampled version(usually
%   1000)
%   ratio, ratio samples between oversampled and downsampled pulses
% Joseph Canfield, July 27, 2016

global STU;         % Stupid constant because MATLAB doesn't like 0
STU = 1;
global SCALAR;      % Largest possible value for signed 16 bit integers
SCALAR = 32767;

len = spb*ratio;   % length of the oversampled pulse

w0  = cbw*pi;   % w0 controls frequency shift
eta = bw*pi;    % eta controls BW of sinc pulse

% Init
OS_table = zeros(1,floor(len));

% Calculate Pulse
for j = 0:len-1
    i = j/ratio - spb/2;
    
    if (i ~= 0)
        OS_table(j+STU) = complex(SCALAR * sin(eta*i) / (eta*i) * cos(w0*i),SCALAR * sin(eta*i) / (eta*i) * sin(w0*i));
    else
        OS_table(j+STU) = complex(SCALAR, 0.0);
    end
end

end

