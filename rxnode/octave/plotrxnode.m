function plotrxnode(fs)
% Loads the binary files output by rxnode so that they may be compared.
%
% M.Overdick 15-Oct-2015
fname0 = 'RX_Ch0.dat';
fname1 = 'RX_Ch1.dat';
%% load data
fid=fopen(fname0,'r');
dat0=fread(fid,Inf,'int16');
fclose(fid);

fid=fopen(fname1,'r');
dat1=fread(fid,Inf,'int16');
fclose(fid);

%% extract I and Q, set up time vector
dat0=dat0(1:2:end)+1j*dat0(2:2:end);
dat1=dat1(1:2:end)+1j*dat1(2:2:end);
t=(0:length(dat0)-1)'/fs;

%% plot results
figure(1);
subplot(211);
plot(t,real(dat0),'LineWidth',2,t,imag(dat0),'LineWidth',2);
title('Channel 0');
legend('I channel','Q channel');
grid on;
xlabel('time (seconds)');

subplot(212);
plot(t,real(dat1),'LineWidth',2,t,imag(dat1),'LineWidth',2);
title('Channel 1');
legend('I channel','Q channel');
grid on;
xlabel('time (seconds)');

figure(2);
plot(t,real(dat0),'LineWidth',2,t,imag(dat0),'LineWidth',2,t,real(dat1),'LineWidth',2,t,imag(dat1),'LineWidth',2);

