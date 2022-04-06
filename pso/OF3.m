%路徑平滑
function F3 = OF3(xc1,yc1,xn1,yn1,xg1,yg1,xc2,yc2,xn2,yn2,xg2,yg2)

        F3 = acos((xc1-xg1)*(xn1-xg1)+(yc1-yg1)*(yn1-yg1))/((xc1 - xn1)^2 + ((yc1 - yn1)^2)^(0.5))*(((xn1 - xg1)^2 + (yn1 - yg1)^2)^(0.5))+...
             acos((xc2-xg2)*(xn2-xg2)+(yc2-yg2)*(yn2-yg2))/((xc2 - xn2)^2 + ((yc2 - yn2)^2)^(0.5))*(((xn2 - xg2)^2 + (yn2 - yg2)^2)^(0.5));

end