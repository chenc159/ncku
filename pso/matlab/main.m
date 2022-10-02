%%2無人機 無障礙物
clc;
clear;
close all;

%%起始點&目標點&速度&安全距離設定
xs1 = 0; ys1 = -30; xg1 = 0; yg1 =  30;
xs2 = 0; ys2 =  30; xg2 = 0; yg2 = -30; 
v0 = 5; r = 5;
 
%%初始目前位置設定
xc1(1) = xs1; yc1(1) = ys1; theta1 = rand;
xc2(1) = xs2; yc2(1) = ys2; theta2 = rand;

%%PSO算出下一航點，飛往下一航點，判斷是否抵達，繼續計算下一航點直到抵達目標點
for i = 1:1000

    [theta1(i), theta2(i)] = PSO(xc1(i),yc1(i),xg1,yg1,xc2(i),yc2(i),xg2,yg2,v0,r);
    xc1(i+1) = xc1(i) + v0*cos(theta1(i));
    yc1(i+1) = yc1(i) + v0*sin(theta1(i));
    xc2(i+1) = xc2(i) + v0*cos(theta2(i));
    yc2(i+1) = yc2(i) + v0*sin(theta2(i));
    disp(['xc1 = ' num2str(xc1(i+1)) ' yc1 = ' num2str(yc1(i+1)) ' xc2 = ' num2str(xc2(i+1)) ' yc2 = ' num2str(yc2(i+1))]);
    
    if ((xg1 - xc1(i+1))^2 + (yg1 - yc1(i+1))^2)^(0.5) <= 4 &&... 
       ((xg2 - xc2(i+1))^2 + (yg2 - yc2(i+1))^2)^(0.5) <= 4
        fprintf('mission completed');
        break; 
    end

end

%%畫出路徑
plot(xc1,yc1,'LineStyle','-','marker','d','color','#4DBEEE','MarkerSize',4); hold on;
plot(xc2,yc2,'LineStyle','-','marker','d','color','#EDB120','MarkerSize',4); hold on;
plot(xs1,ys1,'marker','o','color','#4DBEEE','MarkerFaceColor','#4DBEEE','MarkerSize',8); hold on;
plot(xs2,ys2,'marker','o','color','#EDB120','MarkerFaceColor','#EDB120','MarkerSize',8); hold on;
plot(xg1,yg1,'marker','^','color','#4DBEEE','MarkerFaceColor','#4DBEEE','MarkerSize',8); hold on;
plot(xg2,yg2,'marker','^','color','#EDB120','MarkerFaceColor','#EDB120','MarkerSize',8); hold on;
grid on; axis equal; axis ([-50 50 -50 50]);