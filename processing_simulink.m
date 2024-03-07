gyro_bs = out.gyro.data(1:3,1,:);
gyro_fr = out.frame_gyro.data(1:3,1,:);
% servo = out.servo1.data(400001:600001,:);
% err = out.err2.data(1:3,1,400001:600001);
% servo = out.servo1.data(:,:);
err = out.err2.data(1:3,1,:);

t = out.gyro.Time;
L = length(t);
% L2 = 600001-400000;
L2 = L;

gyro_bs = reshape(gyro_bs, L, 3);
gyro_fr = reshape(gyro_fr, L, 3);
% servo = reshape(servo, L2, 6);
err = reshape(err, L2, 3);

T = t(2);
Fs = 1/T;

Y = fft(gyro_fr(:,1));

f = (0:length(Y)-1)*Fs/length(Y);

gyro = gyro_fr;
Y = fft(gyro(:,1));
Y2 = fft(gyro(:,2));
Y3 = fft(gyro(:,3));
figure(1)
plot(f(1:50),abs(Y(1:50)),f(1:50),abs(Y2(1:50)),f(1:50),abs(Y3(1:50)))
title('Frame Gyros')


gyro = gyro_bs;
Y = fft(err(:,1));
Y2 = fft(err(:,2));
Y3 = fft(err(:,3));
figure(2)

f = (0:length(Y)-1)*Fs/length(Y);

plot(f(1:50),abs(Y(1:50)),f(1:50),abs(Y2(1:50)),f(1:50),abs(Y3(1:50)))
title('Err')
legend('roll', 'pitch', 'yaw')

% Ys1 = fft(servo(:,1));
% Ys2 = fft(servo(:,2));
% Ys3 = fft(servo(:,3));
% Ys4 = fft(servo(:,4));
% Ys5 = fft(servo(:,5));
% Ys6 = fft(servo(:,6));
% figure(3)

% plot(f(1:50),abs(Ys1(1:50)),f(1:50),abs(Ys2(1:50)),f(1:50),abs(Ys3(1:50)),f(1:50),abs(Ys4(1:50)))
% title('Servo States')
% legend('rdot', 'r', 'pdot', 'p', 'ydot', 'y')

% % Compute the two-sided spectrum
% X_fft_abs = abs(Ys2/L); % Normalize the amplitude
% 
% % Compute the single-sided spectrum
% X_fft_single = X_fft_abs(1:(floor(L/2)+1));
% X_fft_single(2:end-1) = 2*X_fft_single(2:end-1);
% 
% % Generate the frequency axis
% f = Fs*(0:(L/2))/L;
% 
% 
% % Adjust the axis as needed to focus on the main frequency components
% xlim([0, Fs/200]) % Limit x-axis up to Nyquist frequency
