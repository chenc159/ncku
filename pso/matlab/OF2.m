%避免碰撞
function F2 = OF2(xn1,yn1,xn2,yn2,r)

    if     ((xn1 - xn2)^2 + (yn1 - yn2)^2)^(0.5) <= 2*r
        F2 = 100000;
    else
        F2 = 0;
    end
        
end