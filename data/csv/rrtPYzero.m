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
end
doplot('rrt_01092019_2200ph.csv',16); %lab test #1 ?????
end

function doplot(fn, fignum)
figure(fignum);
rrtdata = csvread(fn);  
for i = 1 : length( rrtdata )
    vbin = de2bi( rrtdata(i,11)+16 )
    rrtdata(i,12) = vbin(1)+6;
    rrtdata(i,13) = vbin(2)+4;
    rrtdata(i,14) = vbin(3)+2;
    rrtdata(i,15) = vbin(4);
    %fprintf("%1d %1d %1d %1d\n", rrtdata(i,12:15) );
end
clf; tmax = 10;
subplot(2,2,1);
plot(rrtdata(:,1), rrtdata(:,12)); hold on; grid on;
plot(rrtdata(:,1), rrtdata(:,13));
plot(rrtdata(:,1), rrtdata(:,14));
plot(rrtdata(:,1), rrtdata(:,15));
xlim([0 tmax]);
title('valve commands');

subplot(2,2,2);
plot( rrtdata(:,1), rrtdata(:,2) ); hold on; grid on;
plot( rrtdata(:,1), rrtdata(:,3) );
plot( rrtdata(:,1), rrtdata(:,4) );
xlim([0 tmax]);
ylim([-5 5]);
title('roll pitch yaw rate');

subplot(2,2,3);
plot( rrtdata(:,1), rrtdata(:,8) ); hold on; grid on;
plot( rrtdata(:,1), rrtdata(:,9) );
plot( rrtdata(:,1), rrtdata(:,10) );
xlim([0 tmax]);
title('x y z accel');
ylim([-0.25 0.25]);

subplot(2,2,4);
plot( rrtdata(:,1), rrtdata(:,5) ); hold on; grid on;
plot( rrtdata(:,1), rrtdata(:,6) );
plot( rrtdata(:,1), rrtdata(:,7) );
xlim([0 tmax]);
title('roll pitch yaw');
legend('location','northwest', 'roll', 'pitch', 'yaw');
% ylim([-2 2]);
end
