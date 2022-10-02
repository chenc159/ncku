%最短路徑
function F1 = OF1(xc1,yc1,xg1,yg1,xn1,yn1,xc2,yc2,xg2,yg2,xn2,yn2)

        F1 = ((xc1 - xn1)^2 + (yc1 - yn1)^2)^(0.5) + ((xn1 - xg1)^2 + (yn1 - yg1)^2)^(0.5)+...
             ((xc2 - xn2)^2 + (yc2 - yn2)^2)^(0.5) + ((xn2 - xg2)^2 + (yn2 - yg2)^2)^(0.5);

end