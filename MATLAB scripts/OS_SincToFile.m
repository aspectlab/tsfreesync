% OS_SincToFile
% Calls SincInit.m and then writes the oversampled pulse to file

fold = './';
file = 'OS_Sinc.dat';

% Sinc Pulse Parameters
bw  = .058;     % Normalized Bandwidth
cbw = .5;       % Normalized Carrier Bandwidth
spb = 1000;     % Samples per buffer(length of undersampled pulse)
ratio = 10000;  % Ratio of samples, length(OS_Sinc) = spb * ratio;

% Call and initialize oversampled pulse
OS_Sinc = SincInit(bw,cbw,spb,ratio);

% Split vector into [real, imag, real, imag...]
split = zeros(1,2*length(OS_Sinc));

for k = 1:length(OS_Sinc);
    split(k*2-1) = real(OS_Sinc(k));
    split(k*2)   = imag(OS_Sinc(k));
end

    % Typecast to integer
split = cast(split,'int16');

    % Write to file
fid = fopen(strcat(fold,file),'w');
count = fwrite(fid,split,'int16');
fclose(fid);

%% Open file and plot for verification

fid = fopen(strcat(fold,file),'r');
dat = fread(fid,Inf,'int16');
fclose(fid);

dat=dat(1:2:end)+1j*dat(2:2:end);

close all;
figure(1);
subplot(211);plot(real(dat));
subplot(212);plot(imag(dat));