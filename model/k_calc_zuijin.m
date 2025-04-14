%LQR求解
clear;

syms theta_w_l dot_theta_w_l ddot_theta_w_l;
syms theta_w_r dot_theta_w_r ddot_theta_w_r;
syms theta_l_l dot_theta_l_l ddot_theta_l_l;
syms theta_l_r dot_theta_l_r ddot_theta_l_r;
syms theta_body dot_theta_body ddot_theta_body;



syms s dot_s ddot_s;
syms s_body dot_s_body ddot_s_body;
syms h_body dot_h_body ddot_h_body;
syms phi dot_phi ddot_phi;

syms s_l_l dot_s_l_l ddot_s_l_l;
syms s_l_r dot_s_l_r ddot_s_l_r;
syms h_l_l dot_h_l_l ddot_h_l_l;
syms h_l_r dot_h_l_r ddot_h_l_r;

syms T_wheel_l T_wheel_r T_body_l T_body_r;
syms F_wheel_s_l F_wheel_s_r F_wheel_h_l F_wheel_h_r;
syms F_body_s_l F_body_s_r F_body_h_l F_body_h_r;
syms R_wheel Width_body_half L_l L_r L_wheel_l L_wheel_r L_body_l L_body_r L_c M_wheel M_leg M_body I_wheel I_leg_l I_leg_r I_body I_z;
syms g;
R_wheel=0.091;
Width_body_half=0.2275;
L_c =0.02569;
M_wheel =1.62;
M_leg =0.92;
M_body=11.92;
I_wheel =0.005874;
I_body =0.287855;
I_z=0.5;
g=9.8;

%Q_cost=diag([50 150 50 120 10 10 10 10 2500 50]);
%Q_cost=diag([10 10 10 10 150 120 150 120 2000 50]);
Q_cost=diag([150 100 150 100 10 10 10 10 150 100]);
R_cost=diag([3 3 0.25 0.25]);

ddot_s=ddot_s_body+L_l/2*sin(theta_l_l)*dot_theta_l_l^2-L_l/2*cos(theta_l_l)*ddot_theta_l_l+L_r/2*sin(theta_l_r)*dot_theta_l_r^2-L_r/2*cos(theta_l_r)*ddot_theta_l_r-L_c*sin(theta_body)*dot_theta_body^2+L_c*cos(theta_body)*ddot_theta_body;
ddot_theta_w_r=+Width_body_half/R_wheel*ddot_phi+ddot_s/R_wheel+L_l*cos(theta_l_l)*ddot_theta_l_l/(2*R_wheel)-L_l*sin(theta_l_l)*dot_theta_l_l^2/(2*R_wheel)-L_r*cos(theta_l_r)*ddot_theta_l_r/(2*R_wheel)+L_r*sin(theta_l_r)*dot_theta_l_r^2/(2*R_wheel);
ddot_theta_w_l=-Width_body_half/R_wheel*ddot_phi+ddot_s/R_wheel-L_l*cos(theta_l_l)*ddot_theta_l_l/(2*R_wheel)+L_l*sin(theta_l_l)*dot_theta_l_l^2/(2*R_wheel)+L_r*cos(theta_l_r)*ddot_theta_l_r/(2*R_wheel)-L_r*sin(theta_l_r)*dot_theta_l_r^2/(2*R_wheel);
ddot_h_body=-L_l/2*sin(theta_l_l)*ddot_theta_l_l-L_l/2*cos(theta_l_l)*dot_theta_l_l^2-L_r/2*sin(theta_l_r)*ddot_theta_l_r-L_r/2*cos(theta_l_r)*dot_theta_l_r^2-L_c*cos(theta_body)*dot_theta_body^2-L_c*sin(theta_body)*ddot_theta_body;
ddot_h_l_l=ddot_h_body+L_c*cos(theta_body)*dot_theta_body^2+L_c*sin(theta_body)*ddot_theta_body+L_body_l*cos(theta_l_l)*dot_theta_l_l^2+L_body_l*sin(theta_l_l)*ddot_theta_l_l;
ddot_h_l_r=ddot_h_body+L_c*cos(theta_body)*dot_theta_body^2+L_c*sin(theta_body)*ddot_theta_body+L_body_r*cos(theta_l_r)*dot_theta_l_r^2+L_body_r*sin(theta_l_r)*ddot_theta_l_r;
ddot_s_l_l=R_wheel*ddot_theta_w_l+L_wheel_l*cos(theta_l_l)*ddot_theta_l_l-L_wheel_l*sin(theta_l_l)*dot_theta_l_l^2;
ddot_s_l_r=R_wheel*ddot_theta_w_r+L_wheel_r*cos(theta_l_r)*ddot_theta_l_r-L_wheel_r*sin(theta_l_r)*dot_theta_l_r^2;

F_body_h_l=(M_body*ddot_h_body+M_body*g+M_leg*ddot_h_l_r-M_leg*ddot_h_l_l)/2;
F_body_h_r=(M_body*ddot_h_body+M_body*g-M_leg*ddot_h_l_r+M_leg*ddot_h_l_l)/2;
F_wheel_h_l=F_body_h_l+M_leg*g+M_leg*ddot_h_l_l;
F_wheel_h_r=F_body_h_r+M_leg*g+M_leg*ddot_h_l_r;
F_wheel_s_l=(T_wheel_l-(I_wheel+M_wheel*R_wheel^2)*ddot_theta_w_l)/R_wheel;
F_wheel_s_r=(T_wheel_r-(I_wheel+M_wheel*R_wheel^2)*ddot_theta_w_r)/R_wheel;
F_body_s_l=F_wheel_s_l-M_leg*ddot_s_l_l;
F_body_s_r=F_wheel_s_r-M_leg*ddot_s_l_r;

equ1=F_body_s_l+F_body_s_r-M_body*ddot_s_body;
equ2=I_leg_l*ddot_theta_l_l-(F_wheel_h_l*L_wheel_l+F_body_h_l*L_body_l)*sin(theta_l_l)+(F_wheel_s_l*L_wheel_l+F_body_s_l*L_body_l)*cos(theta_l_l)+T_wheel_l-T_body_l;
equ3=I_leg_r*ddot_theta_l_r-(F_wheel_h_r*L_wheel_r+F_body_h_r*L_body_r)*sin(theta_l_r)+(F_wheel_s_r*L_wheel_r+F_body_s_r*L_body_r)*cos(theta_l_r)+T_wheel_r-T_body_r;
equ4=I_body*ddot_theta_body-T_body_l-T_body_r-(F_body_s_l+F_body_s_r)*L_c*cos(theta_body)-(F_body_h_l+F_body_h_r)*L_c*sin(theta_body);
equ5=I_z*ddot_phi+(M_wheel*R_wheel*ddot_theta_w_l+F_wheel_s_l-M_wheel*R_wheel*ddot_theta_w_r-F_wheel_s_r)*Width_body_half;


[ddot_s_body, ddot_phi, ddot_theta_l_l,ddot_theta_l_r,ddot_theta_body] = solve([equ1, equ2, equ3,equ4,equ5], [ddot_s_body, ddot_phi, ddot_theta_l_l,ddot_theta_l_r,ddot_theta_body]);
Ja = jacobian([dot_s_body,ddot_s_body, dot_phi, ddot_phi,  dot_theta_l_l,ddot_theta_l_l,dot_theta_l_r,ddot_theta_l_r,dot_theta_body,ddot_theta_body], [s_body,dot_s_body, phi, dot_phi, theta_l_l,dot_theta_l_l,theta_l_r,dot_theta_l_r,theta_body,dot_theta_body]);
Jb = jacobian([dot_s_body,ddot_s_body, dot_phi, ddot_phi,  dot_theta_l_l,ddot_theta_l_l,dot_theta_l_r,ddot_theta_l_r,dot_theta_body,ddot_theta_body], [T_wheel_l ,T_wheel_r ,T_body_l ,T_body_r]);
    
A = simplify(vpa(subs(Ja, [s_body,dot_s_body, phi, dot_phi, theta_l_l,dot_theta_l_l,theta_l_r,dot_theta_l_r,theta_body,dot_theta_body], [0, 0, 0, 0, 0, 0,0,0,0,0])));
B = simplify(vpa(subs(Jb, [s_body,dot_s_body, phi, dot_phi, theta_l_l,dot_theta_l_l,theta_l_r,dot_theta_l_r,theta_body,dot_theta_body], [0, 0, 0, 0, 0, 0,0,0,0,0])));
    
disp(A);
disp(B);
%LQR计算
leg_varl = 0.096;
K=zeros(900,40);
K_test=zeros(900,40);
x1=zeros(900,1);
x2=zeros(900,1);
x1sq=zeros(900,1);
x2sq=zeros(900,1);
x1x2=zeros(900,1);
for i=1:30
    leg_varl=leg_varl+0.01; % 10mm线性化一次
    lll=1.1519*leg_varl^2-0.2728*leg_varl+0.117;
    llml=leg_varl-lll;
    i_pl =-13.132* leg_varl^4 +13.37*leg_varl^3-5.0186*leg_varl^2+0.8074*leg_varl-0.0204;
    leg_varr = 0.096;
    for j=1:30
        leg_varr=leg_varr+0.01; % 10mm线性化一次
        llr=1.1519*leg_varr^2-0.2728*leg_varr+0.117;
        llmr=leg_varr-llr;
        i_pr =-13.132* leg_varr^4 +13.37*leg_varr^3-5.0186*leg_varr^2+0.8074*leg_varr-0.0204;
        trans_A=subs(A,[L_l L_r L_wheel_l L_wheel_r L_body_l L_body_r I_leg_l I_leg_r],[leg_varl leg_varr lll llr llml llmr i_pl i_pr]);
        trans_B=subs(B,[L_l L_r L_wheel_l L_wheel_r L_body_l L_body_r I_leg_l I_leg_r],[leg_varl leg_varr lll llr llml llmr i_pl i_pr]);
    %kk为不同时刻的反馈增益矩阵
        KK=lqrd(trans_A,trans_B,Q_cost,R_cost,0.001);
        KK_t=KK.';
        K((i-1)*30+j,:)=KK_t(:);
        x1((i-1)*30+j,1)=leg_varl;
        x2((i-1)*30+j,1)=leg_varr;
        x1sq((i-1)*30+j,1)=leg_varl^2;
        x2sq((i-1)*30+j,1)=leg_varr^2;
        x1x2((i-1)*30+j,1)=leg_varl*leg_varr;
    end
end
K_cons=zeros(40,6);  
%% 系数拟合
X = [x1, x2, x1sq, x2sq, x1x2];
%决定每个kij的拟合方法
for k=1:40
    mdl = fitlm(X, K(:,k));
    disp(mdl.Coefficients.Estimate');
    K_cons(k,:)=mdl.Coefficients.Estimate';
end
% for i=1:30
%     for j=1:30'
%         for k=1:40
%             K_test((i-1)*30+j,k)=K_cons(k,1)+K_cons(k,2)*x1((i-1)*30+j,1)+K_cons(k,3)*x2((i-1)*30+j,1)+K_cons(k,4)*x1sq((i-1)*30+j,1)+K_cons(k,5)*x2sq((i-1)*30+j,1)+K_cons(k,6)*x1x2((i-1)*30+j,1);
%         end
%     end
% end
% figure;  % 创建一个新的图形窗口
% for k = 31:40
%     subplot(5, 2, k-30);  % 设置子图布局为4行10列
%     scatter3(x1, x2, K(:,k), 'filled', 'DisplayName', '真实值');  % 绘制第一组三维散点图
%     hold on;  % 保持当前子图，以便绘制下一组数据
%     scatter3(x1, x2, K_test(:,k), 'r', 'filled', 'DisplayName', '拟合值');  % 绘制第二组三维散点图，使用红色
%     xlabel('左侧腿长');  % 设置X轴标签
%     ylabel('右侧腿长');  % 设置Y轴标签
%     title(['反馈矩阵真实值与拟合值随左右腿长变化情况']);  % 设置图表标题，显示当前子图索引
%     legend show;
%     grid on;  % 显示网格
%     hold off;  % 释放当前子图
% end


