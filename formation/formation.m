clear;
clc;
addpath(genpath('uav_real_plot_function'))
%% 參數設置: 四旋翼加速度 ( x'', y'' )
% e =roll , f =pitch , g =yaw
global F m r1 p1 y1 r2 y2 p2 r3 y3 p3 dt theta d tm kp1p  kp1r  kp2p kp3p  kp2r kp3r kp3dp kp3dr kp2dr kp2dp
r1 = 0;
p1 = 0;
r2 = 0;
p2 = 0;
r3 = 0;
p3 = 0;
F = 6;  %  8N
m = 2; %  2kg
d = 5;
dt=0.1;
tm=20;
kp1p=0.2;
kp1r=8;
kp2p=1.4;
kp2r=8;
kp2dr =5;
kp2dp =7;
kp3p =1.4;
kp3r=8;
kp3dr =10.5;
kp3dp =5.5;
theta=0;
y1=0;
y2=0;
y3=0;
%% 初始條件 :  長機、僚機初始位置、初始速度
uav1Lp =[0 0];
uav1Lv =[0 0];
uav2Fp =[ 2.5 , (d*cosd(150) -5 ) ];  
uav3Fp =[ -2.5 , (d*cosd(150) -5) ];  
uav2Fv= [ 0 0 ];
uav3Fv =[ 0 0 ];
uav2dpv =[0 2];
uav3dpv =[0 2];
%% 導航點
wp1 = [0 0] ; wp2=[0 40] ;  wp3=[40 40] ; wp4=[40 0] ; 
% %% 真實領導無人機的位置動態、速度動態
% uav1Lv = [uav1Lv(:,1) + a1L * cosd(theta)*dt       ,      uav1Lv(:,2) + a1L *sind (theta)*dt  ]  ;
% uav1Lp= [uav1Lp(:,1)+uav1Lv(:,1)*dt + 1/2*a1L*cosd(theta)*dt^2  ,  uav1Lp(:,2)+uav1Lv(:,2)*dt + 1/2*a1L*sind(theta)*dt^2  ];
% %% 實際僚機 : 速度、位置動態
% uav2Fv = [uav2Fv(:,1) + a2F * cosd(theta)*dt       ,      uav2Fv(:,2) + a2F *sind (theta)*dt  ]  ;  %uavv = uavv + a * dt ;
% uav3Fv = [uav3Fv(:,1) + a3F * cosd(theta)*dt       ,      uav3Fv(:,2) + a3F * sind(theta)*dt  ]  ;  %uavp= uavp*dt + 1/2*v+a*dt^2;
% uav2Fp= [uav2Fp(:,1)+uav2Fv(:,1)*dt + 1/2*a2F*cosd(theta)*dt^2  ,  uav2Fp(:,2)+uav2Fv(:,2)*dt + 1/2*a2F*sind(theta)*dt^2  ];
% uav3Fp= [uav3Fp(:,1)+uav3Fv(:,1)*dt + 1/2*a3F*cosd(theta)*dt^2  ,  uav3Fp(:,2)+uav3Fv(:,2)*dt + 1/2*a3F*sind(theta)*dt^2  ];
% %% 黃點位置、速度(隨真實領導無人機變化)
%  uav2dp=[uav1Lp(:,1)+d* cosd(150+theta)                            uav1Lp(:,2)+d* sind(150+theta)];
%  uav3dp=[uav1Lp(:,1)+d* cosd(210+theta)                            uav1Lp(:,2)+d* sind(210+theta)];
%% 初始設置
time =0:dt:tm;
a1Lt=zeros(length(time),2);
a2Ft=zeros(length(time),2);
a3Ft=zeros(length(time),2);
uav1Lvt = zeros(length(time),2);
uav1Lpt =  zeros(length(time),2);
uav2dpt =  zeros(length(time),2);
uav3dpt =  zeros(length(time),2);
uav2Fvt=zeros(length(time),2);
uav3Fvt = zeros(length(time),2);
uav2Fpt= zeros(length(time),2);
uav3Fpt=zeros(length(time),2);
r1t=zeros(length(time),1);
p1t=zeros(length(time),1);
y1t=zeros(length(time),1);
r2t=zeros(length(time),1);
p2t=zeros(length(time),1);
y2t=zeros(length(time),1);
r3t=zeros(length(time),1);
p3t=zeros(length(time),1);
y3t=zeros(length(time),1);
 %% 動態圖形建置
for t =0:length(time)
    %wp2、wp3、wp4
    %% wp1->wp2  roll=0;
    if uav1Lp (:,1)==0 && ( 0<=uav1Lp (:,2))<30 %theta=0
    %  位置誤差判斷長機與目標點
           r1= (0-uav1Lp(:,1))*-kp1r;
           p1= (30-uav1Lp(:,2))*-kp1p;
           a1L= [   -F/m*(cosd(r1)*sind(p1)*sind(y1)+sind(r1)*cosd(y1) ) , -F/m*(cosd(r1)*sind(p1)*cosd(y1)+sind(r1)*sind(y1) )] ; % y'' , x''
           uav1Lv = [uav1Lv(:,1) + a1L(:,1) *dt             uav1Lv(:,2) + a1L(:,2)*dt  ]  ;
           uav1Lp= [uav1Lp(:,1)+uav1Lv(:,1)*dt + 0.5*a1L(:,1)*dt^2    uav1Lp(:,2)+uav1Lv(:,2)*dt + 0.5*a1L(:,2)*dt^2  ];
           uav2dp=[uav1Lp(:,1)+d* sind(150+theta)                            uav1Lp(:,2)+d* cosd(150+theta)];
           uav3dp=[uav1Lp(:,1)+d* sind(210+theta)                            uav1Lp(:,2)+d* cosd(210+theta)];
            if uav1Lv(:,2)>2 
                uav1Lv(:,2) = 2;
                a1L =0;
            end
    %  位置誤差判斷黃點與僚機
          r2=( uav2dp(:,1)-uav2Fp(:,1))*-kp2r ;
          p2 =( uav2dp(:,2)-uav2Fp(:,2))*-kp2p;
          r3 =(uav3dp(:,1)-uav3Fp(:,1))*-kp3r ;
          p3 =(uav3dp(:,2)-uav3Fp(:,2))*-kp3p;
         a2F= [ -F/m*(cosd(r2)*sind(p2)*sind(y2)+sind(r2)*cosd(y2) ) , -F/m*(cosd(r2)*sind(p2)*cosd(y2)+sind(r2)*sind(y2) )] ; 
         a3F= [-F/m*(cosd(r3)*sind(p3)*sind(y3)+sind(r3)*cosd(y3) ) , -F/m*(cosd(r3)*sind(p3)*cosd(y3)+sind(r3)*sind(y3) ) ] ; 
         uav2Fv = [uav2Fv(:,1) + a2F(:,1)*dt       ,      uav2Fv(:,2) + a2F(:,2)*dt  ]  ; 
         uav3Fv = [uav3Fv(:,1) + a3F(:,1)*dt       ,      uav3Fv(:,2) + a3F(:,2)*dt  ]  ; 
         uav2Fp= [uav2Fp(:,1)+uav2Fv(:,1)*dt + 0.5*a2F(:,1)*dt^2  ,  uav2Fp(:,2)+uav2Fv(:,2)*dt + 0.5*a2F(:,2)*dt^2  ];
         uav3Fp= [uav3Fp(:,1)+uav3Fv(:,1)*dt + 0.5*a3F(:,1)*dt^2  ,  uav3Fp(:,2)+uav3Fv(:,2)*dt + 0.5*a3F(:,2)*dt^2  ];
          if uav2Fv(:,2)>4 
                uav2Fv(:,2) = 4;
                a2F =0;
          end
          if uav3Fv(:,2)>4 
                uav3Fv(:,2) = 4;
               a3F =0;
          end
%            當到快達同一位置時,僚機的速度開始下降直到跟虛擬黃點速度一樣
          if  uav2dp(:,2)-uav2Fp(:,2) <0.05 || uav3dp(:,2)-uav3Fp(:,2) <0.05
              uav2Fv(:,2)=  uav1Lv(:,2);
              uav3Fv(:,2)=  uav1Lv(:,2);
          end
    end
