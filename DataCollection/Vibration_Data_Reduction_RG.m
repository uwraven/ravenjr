% Roman Grinchuk
% Vibration Data Reduction

clear all; close all; clc
addpath('Vibration Testing')


%                           PROCESS VIBRATION DATA

vibration_data = ["v_test_70_2.mat","v_test_90_1.mat","v_test_90_2.mat","v_test_110_1.mat"];
fft_plot_name = ["70^o Test II","90^o Test I","90^o Test II", "110^o Test I"];

for i = 1:length(vibration_data)
    load(vibration_data(i))
    [mean_sig_vals(i,:),sig_var(i,:)] = data_reduction(xhatdata,fft_plot_name(i));
end


%                           PLOT MEAN SIGNAL VALUES
figure
for i = 1:4
    plot([2:7],mean_sig_vals(i,2:end),'x-'); hold on
end

hold off; title('Mean Signal Values'); grid on
set(gca,'xtick',2:7); set(gca,'xlim',[2,7])
xlabel('Channel'); ylabel('Mean Signal Values')
legend('70^o','90^o I','90^o II','110^o','location','best')


%                           PLOT SIGNAL VARIANCE
figure
for i = 1:4
    plot([2:7],sig_var(i,2:end),'x-'); hold on
end

hold off; title('Signal Varience'); grid on
set(gca,'xtick',2:7); set(gca,'xlim',[2,7])
xlabel('Channel'); ylabel('Signal Variance')
legend('70^o','90^o I','90^o II','110^o','location','best')




%{
cutoff_freqs = [1,10,20,30,50];
data_set = 2;
plot_LPF(xhatdata,cutoff_freqs,data_set)

for i = 2:7
    data_set = i;
    plot_LPF(xhatdata,cutoff_freqs,data_set)
end
%}



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [mean_sig_vals,sig_var]=data_reduction(xhatdata,fft_plot_name)

% MEAN SIGNAL VALUE
for i = 2:7
    mean_sig_vals(i) = mean(xhatdata(1:end,i));
end

% SIGNAL VARIENCE
for i = 2:7
    sig_var(i) = var(xhatdata(1:end,i));
end

% FAST FOURIER TRANSFORM
figure
for i = 2:7
    Y = fft(xhatdata(1:end,i));
    L = (xhatdata(end,1)-xhatdata(1,1));
    P2 = abs(Y/L);
    P1 = P2(1:L/2+1);
    P1(2:end-1) = 2*P1(2:end-1);
    Fs = length(xhatdata)/(L/1000);
    
    f = Fs*(0:(L/2))/L;
    subplot(3,2,i-1)
    plot(f,P1)
    title(['Channel ',num2str(i)])
    xlabel('f (Hz)')
    ylabel('|P1(f)|')
end
sgtitle(fft_plot_name)
end

function []=plot_LPF(xhatdata,cutoff_freqs,data_set)

run_time = (xhatdata(end,1)-xhatdata(1,1))/1000
sampling_freq = length(xhatdata)/run_time
t = (xhatdata(1:end,1)-xhatdata(1,1))/1000;

y_lim_u = max(xhatdata(1:end,data_set));
y_lim_l = min(xhatdata(1:end,data_set));


Fs = sampling_freq;
order = 2;
figure
sgtitle(['Data Set ', num2str(data_set)])
for i = 1:length(cutoff_freqs)
    [filtered_signal,filtb,filta] = lopass_butterworth([t,xhatdata(1:end,data_set)],cutoff_freqs(i),Fs,order);
    subplot(length(cutoff_freqs)+1,1,i)
    plot(filtered_signal(1:end,1),filtered_signal(1:end,2))
    title([num2str(cutoff_freqs(i)), ' Hz Cutoff Freq.'])
    ylim([y_lim_l y_lim_u])
    xlim([0 5])
    
end
subplot(length(cutoff_freqs)+1,1,i+1)
plot(t,xhatdata(1:end,data_set))
title('Raw Signal')

end

function [filtered_signal,filtb,filta]=lopass_butterworth(inputsignal,cutoff_freq,Fs,order)
% Low-pass Butterworth filter
% [filtered_signal,filtb,filta] = lopass_butterworth(inputsignal,cutoff_freq,Fs,order)
% 
% This is simply a set of built-in Matlab functions, repackaged for ease of
% use by Chad Greene, October 2012. 
%
% INPUTS: 
% inputsignal = input time series
% cutoff_freq = filter corner frequency
% Fs = data sampling frequency
% order = order of Butterworth filter
%  
% OUTPUTS: 
% filtered_signal = the filtered time series
% filtb, filta = filter numerator and denominator (optional)
% 
% EXAMPLE 1: 
% load train
% t = (1:length(y))/Fs;
% y_filt = lopass_butterworth(y,900,Fs,4); % cut off at 900 Hz
% figure
% plot(t,y,'b',t,y_filt,'r')
% xlabel('time in seconds')
% box off
% legend('unfiltered','filtered')
% sound(y,Fs)      % play original time series
% pause(2)         % pause two seconds
% sound(y_filt,Fs) % play filtered time series
% 
% 
% EXAMPLE 2: 
% load train
% t = (1:length(y))/Fs;
% [y_filt,filtb,filta] = lopass_butterworth(y,900,Fs,4); % cut off at 900 Hz
% [h1,f1] = freqz(filtb,filta,256,Fs);
% 
% figure
% subplot(3,1,1)
% plot(t,y,'b',t,y_filt,'r')
% xlabel('time in seconds')
% box off
% text(0,.1,' time series','units','normalized')
% 
% subplot(3,1,2)
% AX = plotyy(f1,10*log10(abs(h1)),f1,angle(h1),'semilogx');
% set(get(AX(1),'ylabel'),'string','gain (dB)')
% set(get(AX(2),'ylabel'),'string','phase (rad)')
% xlim(AX(1),[min(f1) max(f1)])
% xlim(AX(2),[min(f1) max(f1)])
% text(0,.1,' filter response','units','normalized')
% box off
% 
% [Pxx,f] = pwelch(y,512,256,[],Fs,'onesided');
% [Pxxf,f_f]= pwelch(y_filt,512,256,[],Fs,'onesided');
% subplot(3,1,3)
% semilogx(f,10*log10(Pxx))
% hold on
% semilogx(f_f,10*log10(Pxxf),'r')
% xlabel('frequency (Hz)')
% ylabel('PSD (dB)')
% xlim([min(f1) max(f1)])
% box off
% legend('unfiltered','filtered','location','northwest')
% legend boxoff

nyquist_freq = Fs/2;  % Nyquist frequency
Wn=cutoff_freq/nyquist_freq;    % non-dimensional frequency
[filtb,filta]=butter(order,Wn,'low'); % construct the filter
filtered_signal=filtfilt(filtb,filta,inputsignal); % filter the data with zero phase 
end