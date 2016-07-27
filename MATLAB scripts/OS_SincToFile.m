% OS_SincToFile
% Calls SincInit.m and then writes the oversampled pulse to file

bw  = .058;
cbw = .5;
spb = 1000;
ratio = 10000;

table = SincInit(bw,cbw,spb,ratio);

split = zeros(1,2*length(table));

for k = 1:length(table);
    split(k*2-1) = real(table(k));
    split(k*2)   = imag(table(k));
end

    % Typecast to integer
split = cast(split,'int16');

    % Write to file
fid = fopen('OS_Sinc.dat','w');
count = fwrite(fid,split,'int16');
fclose(fid);

%% Open file and plot for verification

file = 'OS_Sinc.dat';

fid = fopen(file,'r');
dat = fread(fid,Inf,'int16');
fclose(fid);

dat=dat(1:2:end)+1j*dat(2:2:end);

close all;
figure(1);
subplot(211);plot(real(dat));
subplot(212);plot(imag(dat));