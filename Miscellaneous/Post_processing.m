% Post Processing
% t = 0:dt:tf-dt;

% ARCSEC = 1;


figure(1)
tiledlayout(3,3)
nexttile
plot(t,y_all(16,:),t,y_all(17,:),t,y_all(18,:));
legend('OF','MF','IF')
title('Gondola Frames')

nexttile
plot(t,y_all(13,:),t,y_all(14,:),t,y_all(15,:));
legend('FT Roll','FT Pitch','FT Twist')
title('Flight Train')

nexttile
plot(t,y_all(10,:),t,y_all(11,:),t,y_all(12,:));
legend('BC Twist','BC Roll','BC Pitch')
title('Balloon')

nexttile
plot(t,y_all(21,:));
title('RW Angular Momentum')

nexttile
% plot(t,e_hist(1,:)/ARCSEC,t,e_hist(2,:)/ARCSEC,t,e_hist(3,:)/ARCSEC);
% legend('Roll','Pitch','Yaw')
% title('Error')
plot(t, e_hist(1,:)/ARCSEC,t,e_hist(3,:)/ARCSEC);
legend('Roll','Yaw')
title('Error')
ylim([-1 1])
err_sum = zeros(3,length(t));
for k = 2:length(t)
    C = compute_rotation_mat(z_n, y_all(10:18,k));
     
    %identity - (C_true' * C_des) and then extract euler angles 5.37
    err_sum(:,k) = err_sum(:,k-1) * err_decay + e_hist(:,k)*dt;
end

nexttile

plot(t, err_sum(1,:)/ARCSEC,t,err_sum(3,:)/ARCSEC);
legend('Roll','Yaw')
title('Error Sum')
ylim([-1 1])
% plot(t,err_sum(1,:)/ARCSEC,t,err_sum(2,:)/ARCSEC,t,err_sum(3,:)/ARCSEC);
% legend('Roll','Pitch','Yaw')
% title('Error Sum')

nexttile

plot(t,tau(7,:),t,tau(8,:),t,tau(9,:));
legend('Yaw','Roll','Pitch')
title('tau')

nexttile
% plot(t,e_hist(1,:)/ARCSEC,t,e_hist(2,:)/ARCSEC,t,e_hist(3,:)/ARCSEC);
% legend('Roll','Pitch','Yaw')
% title('Error')
plot(t,e_hist(3,:)/ARCSEC);
legend('Yaw')
title('Error')

nexttile

plot(t,err_sum(3,:)/ARCSEC);
legend('Yaw')
title('Error Sum')
% plot(t,err_sum(1,:)/ARCSEC,t,err_sum(2,:)/ARCSEC,t,err_sum(3,:)/ARCSEC);
% legend('Roll','Pitch','Yaw')
% title('Error Sum')


figure(2)
tiledlayout(3,3)
nexttile
plot(t,e_hist(1,:)/ARCSEC);
legend('Roll')
title('Error')

nexttile
plot(t,e_hist(2,:)/ARCSEC);
legend('Pitch')
title('Error')

nexttile
plot(t,e_hist(3,:)/ARCSEC);
legend('Yaw')
title('Error')

nexttile

plot(t,err_sum(1,:)/ARCSEC);
legend('Roll')
title('Error Sum')
nexttile

plot(t,err_sum(2,:)/ARCSEC);
legend('Pitch')
title('Error Sum')
nexttile

plot(t,err_sum(3,:)/ARCSEC);
legend('Yaw')
title('Error Sum')

nexttile

plot(t,tau(8,:));
legend('Roll')
title('tau')
nexttile

plot(t,tau(9,:));
legend('Pitch')
title('tau')
nexttile

plot(t,tau(7,:));
legend('Yaw')
title('tau')