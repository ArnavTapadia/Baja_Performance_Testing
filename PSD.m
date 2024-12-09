function [Pxx, F] = PSD(time, signal, frequency)
% test parameter

    if ~exist('frequency', 'var')
        frequency = 1/0.001; %default frequency to upsample to (1000Hz)
    end
    linearTime = 0:1/frequency:max(time);
    linearSignal = interp1(time, signal, linearTime);
    % Test 
    % linearSignal = 5*cos(2*pi*3*linearTime) + 2*sin(2*pi*5*linearTime) + 1*randn(size(linearTime));
    nfft = 4096;
    noverlap = round(nfft*0.75);
    window = hann(nfft);

    [Pxx,F] = pwelch(linearSignal, window, noverlap, nfft, frequency);

    figure
    plot(F, Pxx)
    xlabel('Frequency (Hz)')
    title('Power Spectral Density')
    grid on
    xlim([0, 25])
end