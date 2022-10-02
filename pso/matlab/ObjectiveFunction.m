%目標方程式
%解決:1.最短路徑 2.避免碰撞 3.路徑平滑

function  F = ObjectiveFunction(F1,F2,F3)

        a =   1;
        b =   1;
        c = 0.5;
        F = a*F1 + b*F2 + c*F3;
    
end