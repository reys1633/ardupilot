function rrtplot
%time,gyrx,gyry,gyrz,rol,pit,yaw,accx,accy,accz,vCmds
R2D = 180/pi;
%rrtdata = csvread('rrt_01072019_1431.csv');
%rrtdata = csvread('rrt_01072019_1528.csv');
% rrtdata = csvread('rrt_01082018_1135.csv');
if 0
    doplot('rrt01072019_2229.csv',3); %2 deg
    doplot('rrt_01082018_1135.csv',4); %5 deg
    doplot('rrt_01082018_1438.csv',5); %10 deg commands
    doplot('rrt_01082018_1449.csv',6); %pitch -45, 45 deg motion
    doplot('rrt_01082018_1500.csv',7); %roll -45, 45 deg motion
    doplot('rrt01082018_1546.csv',8); %lab test #1 ?????
    doplot('rrt_01082018_1700pr.csv',9); %lab test #1 ?????
    doplot('rrt_01082018_1810pr_pitch.csv',10); %lab test #1 ?????
    doplot('rrt_01082018_1820pr_roll.csv',11); %lab test #1 ?????
    doplot('rrt_01082018_2034pr_roll.csv',12); %lab test #1 ?????
    doplot('rrt_01082018_2039pr_pit.csv',13); %lab test #1 ?????
    doplot('rrt_01092018_1355pr_pitrol.csv',14); %lab test #1 ?????
    doplot('rrtlog_01092019_1443pr_pitrol.csv',15); %lab test #1 ?????
    doplot('rrt_01092019_2200ph.csv',16); %lab test #1 ?????
    doplot('rrt_01102019_0810ph_stp.csv',1); %lab test #1 ?????
    doplot('rrt_01092019_0819ph_zero.csv',2); %lab test #1 ?????
    doplot('rrt_01102019_0842ph_zero_2hz.csv',3); %lab test #1 ?????
    doplot('rrt_01102019_0948ph_zero_2hz_imu0.csv',4); %lab test #1 ?????
end
%doplot('rrt_01102018_1356prStep.csv',4); %lab test #1 ?????
%doplot('rrt_01122019_0926.csv',5); %lab test #1 ?????
%doplot('rrt_01142019_1814.csv',4); %lab test #1 ?????
%doplot('rrt_01152019_1323.csv',4); %lab test #1 ?????
%doplot('rrt01152019_1438.csv',4); %lab test #1 ?????
%doplot('rrt_01152019_1530.csv',3); %lab test #1 ?????
%doplot('rrt_01162019_0902.csv',4); %lab test #1 ?????
%doplot('rrt_01162018_0950.csv',5); %lab test #1 ?????
%doplot('rrt01162018_1118good.csv',5); %lab test #1 ?????
% doplot('rrt_01172019_1509.csv',3); %lab test #1 ?????
% doplot('rrt_01172019_1522.csv',4); %lab test #1 ?????
% doplot('rrt_01172019_1536.csv',5); %lab test #1 ?????
doplot('rrt02052019_1307.csv',5); %lab test #1 ?????


end

function doplot(fn, fignum)

scaley = 0; rpyscl = 20;
scalex = 1; tmin = 5; tmax = 6;
figure(fignum);
rrtdata = csvread(fn);
for i = 1 : length( rrtdata )
    
%   de2bi(16)   =  0     0     0     0     1
%   de2bi(1+16) =  1     0     0     0     1
%   de2bi(2+16) =  0     1     0     0     1
%   de2bi(8+16) =  0     0     0     1     1
    
    vbin = de2bi( rrtdata(i,11)+16 );
    rrtdata(i,13) = vbin(1)+6;
    rrtdata(i,14) = vbin(2)+2;
    rrtdata(i,15) = vbin(3)+2;
    rrtdata(i,16) = vbin(4);
    %fprintf("%1d %1d %1d %1d\n", rrtdata(i,12:15) );
end
clf;
subplot(3,1,1);
% plot(rrtdata(:,1), rrtdata(:,13)); hold on; grid on;
plot(rrtdata(:,1), rrtdata(:,14)); hold on; grid on;
% plot(rrtdata(:,1), rrtdata(:,15));
plot(rrtdata(:,1), rrtdata(:,16));
if scalex; xlim([tmin tmax]); end
title('valve commands');

subplot(3,1,2);
%plot( rrtdata(:,1), rrtdata(:,2) ); hold on; grid on;
plot( rrtdata(:,1), rrtdata(:,3) );
%plot( rrtdata(:,1), rrtdata(:,4) );
if scalex; xlim([tmin tmax]); end
if scaley; ylim([-5 5]); end
title('roll pitch yaw rate');

% subplot(3,1,2);
% plot( rrtdata(:,1), rrtdata(:,8) ); hold on; grid on;
% plot( rrtdata(:,1), rrtdata(:,9) );
% plot( rrtdata(:,1), rrtdata(:,10) );
% if scalex; xlim([0 tmax]); end
% title('x y z accel');
% if scaley; ylim([-0.25 0.25]); end

subplot(3,1,3);
%plot( rrtdata(:,1), rrtdata(:,5) ); hold on; grid on;
plot( rrtdata(:,1), rrtdata(:,6), 'linewidth', 2 );
hold on; grid on;
plot( rrtdata(:,1), rrtdata(:,12)*180/pi, 'linewidth', 2 );

%plot( rrtdata(:,1), rrtdata(:,7) );
if scalex; xlim([tmin tmax]); end
title('roll pitch yaw');
% legend('location','northwest', 'roll', 'pitch', 'yaw');
if scaley; ylim([-rpyscl rpyscl]); end
hold on;
% k=5;
% t1 = 1*k;
% t2 = 2*k;
% t3 = 3*k;
% t4 = 4*k;
% t5 = 5*k;
% t6 = 6*k;
% t7 = 7*k;
% t8 = 8*k;
% t9 = 9*k;
% v = 5;
% plot( ...
%     [0 t1 t1 t2 t2 t3 t3 t4 t4 t5 t5 t6 t6 t7 t7 t8 t8 t9], ...
%     [0 0  -v -v 0  0  v  v  0  0  -v -v 0  0  v  v  0  0]);
end
