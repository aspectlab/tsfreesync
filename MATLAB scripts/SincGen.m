function [ table ] = SincGen(template, ampl, spb, delay )
% table = SincGen(template,ampl,spb,ratio,delay)
% This function takes the oversampled pulse from SincInit.m and
% undersamples it with a shift
% This function requires SincInit.m to be run first
% Outputs :
%   table, the undersampled shifted pulse
% Inputs :
%   template, the oversampled pulse from SincInit.m
%   spb, the length of the undersampled pulse
%   delay, number of samples to delay the undersampled pulse

global SCALAR;
global STU;

ratio = length(template)/spb;
scale = ampl/SCALAR;

if (delay < 0.0)
    delay = delay + spb;
end

    % keep delay below spb
delay = rem(delay,spb);

shift = delay*ratio;

j = 0;
i = 0;
table = zeros(1,spb);

OS_len = spb*ratio;


% Generate first part of delayed pulse
for i = shift:ratio:OS_len-1
    table(j+STU) = scale * template(floor(i+STU));
    j = j + 1;
end

remain = OS_len-i;

% Generate second part of delayed pulse
for i = ratio-remain:ratio:shift-ratio-1
    table(j+STU) = scale * template(floor(i+STU));
    j = j + 1;
end
% plot(real(table));axis([495 505 -ampl/2 ampl]);grid on;

end

