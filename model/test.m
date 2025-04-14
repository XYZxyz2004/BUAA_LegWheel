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

%Q_cost=diag([50 150 50 120 10 10 10 10 2000 50]);
Q_cost=diag([100 3000 100 150 10 10 10 10 10 10]);
R_cost=diag([2 2 0.25 0.25]);

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
B= simplify(vpa(subs(Jb, [s_body,dot_s_body, phi, dot_phi, theta_l_l,dot_theta_l_l,theta_l_r,dot_theta_l_r,theta_body,dot_theta_body], [0, 0, 0, 0, 0, 0,0,0,0,0])));
%LQR计算
trans_C=eye(10);
trans_D=zeros(10,4);

leg_varl= 0.096;
lll=1.1519*leg_varl^2-0.2728*leg_varl+0.117;
llml=leg_varl-lll;
i_pl =-13.132* leg_varl^4 +13.37*leg_varl^3-5.0186*leg_varl^2+0.8074*leg_varl-0.0204;
leg_varr= 0.096;
llr=1.1519*leg_varr^2-0.2728*leg_varr+0.117;
llmr=leg_varr-llr;
i_pr =-13.132* leg_varr^4 +13.37*leg_varr^3-5.0186*leg_varr^2+0.8074*leg_varr-0.0204;
trans_A=subs(A,[L_l L_r L_wheel_l L_wheel_r L_body_l L_body_r I_leg_l I_leg_r],[leg_varl leg_varr lll llr llml llmr i_pl i_pr]);
trans_B=subs(B,[L_l L_r L_wheel_l L_wheel_r L_body_l L_body_r I_leg_l I_leg_r],[leg_varl leg_varr lll llr llml llmr i_pl i_pr]);
KK=lqrd(trans_A,trans_B,Q_cost,R_cost,0.001);
A1=double(trans_A);
B1=double(trans_B);
K1=KK;

leg_varl= 0.196;
lll=1.1519*leg_varl^2-0.2728*leg_varl+0.117;
llml=leg_varl-lll;
i_pl =-13.132* leg_varl^4 +13.37*leg_varl^3-5.0186*leg_varl^2+0.8074*leg_varl-0.0204;
leg_varr= 0.196;
llr=1.1519*leg_varr^2-0.2728*leg_varr+0.117;
llmr=leg_varr-llr;
i_pr =-13.132* leg_varr^4 +13.37*leg_varr^3-5.0186*leg_varr^2+0.8074*leg_varr-0.0204;
trans_A=subs(A,[L_l L_r L_wheel_l L_wheel_r L_body_l L_body_r I_leg_l I_leg_r],[leg_varl leg_varr lll llr llml llmr i_pl i_pr]);
trans_B=subs(B,[L_l L_r L_wheel_l L_wheel_r L_body_l L_body_r I_leg_l I_leg_r],[leg_varl leg_varr lll llr llml llmr i_pl i_pr]);
KK=lqrd(trans_A,trans_B,Q_cost,R_cost,0.001);
A2=double(trans_A);
B2=double(trans_B);
K2=KK;

leg_varl= 0.296;
lll=1.1519*leg_varl^2-0.2728*leg_varl+0.117;
llml=leg_varl-lll;
i_pl =-13.132* leg_varl^4 +13.37*leg_varl^3-5.0186*leg_varl^2+0.8074*leg_varl-0.0204;
leg_varr= 0.296;
llr=1.1519*leg_varr^2-0.2728*leg_varr+0.117;
llmr=leg_varr-llr;
i_pr =-13.132* leg_varr^4 +13.37*leg_varr^3-5.0186*leg_varr^2+0.8074*leg_varr-0.0204;
trans_A=subs(A,[L_l L_r L_wheel_l L_wheel_r L_body_l L_body_r I_leg_l I_leg_r],[leg_varl leg_varr lll llr llml llmr i_pl i_pr]);
trans_B=subs(B,[L_l L_r L_wheel_l L_wheel_r L_body_l L_body_r I_leg_l I_leg_r],[leg_varl leg_varr lll llr llml llmr i_pl i_pr]);
KK=lqrd(trans_A,trans_B,Q_cost,R_cost,0.001);
A3=double(trans_A);
B3=double(trans_B);
K3=KK;

leg_varl= 0.096;
lll=1.1519*leg_varl^2-0.2728*leg_varl+0.117;
llml=leg_varl-lll;
i_pl =-13.132* leg_varl^4 +13.37*leg_varl^3-5.0186*leg_varl^2+0.8074*leg_varl-0.0204;
leg_varr= 0.296;
llr=1.1519*leg_varr^2-0.2728*leg_varr+0.117;
llmr=leg_varr-llr;
i_pr =-13.132* leg_varr^4 +13.37*leg_varr^3-5.0186*leg_varr^2+0.8074*leg_varr-0.0204;
trans_A=subs(A,[L_l L_r L_wheel_l L_wheel_r L_body_l L_body_r I_leg_l I_leg_r],[leg_varl leg_varr lll llr llml llmr i_pl i_pr]);
trans_B=subs(B,[L_l L_r L_wheel_l L_wheel_r L_body_l L_body_r I_leg_l I_leg_r],[leg_varl leg_varr lll llr llml llmr i_pl i_pr]);
KK=lqrd(trans_A,trans_B,Q_cost,R_cost,0.001);
A4=double(trans_A);
B4=double(trans_B);
K4=KK;

leg_varl= 0.296;
lll=1.1519*leg_varl^2-0.2728*leg_varl+0.117;
llml=leg_varl-lll;
i_pl =-13.132* leg_varl^4 +13.37*leg_varl^3-5.0186*leg_varl^2+0.8074*leg_varl-0.0204;
leg_varr= 0.096;
llr=1.1519*leg_varr^2-0.2728*leg_varr+0.117;
llmr=leg_varr-llr;
i_pr =-13.132* leg_varr^4 +13.37*leg_varr^3-5.0186*leg_varr^2+0.8074*leg_varr-0.0204;
trans_A=subs(A,[L_l L_r L_wheel_l L_wheel_r L_body_l L_body_r I_leg_l I_leg_r],[leg_varl leg_varr lll llr llml llmr i_pl i_pr]);
trans_B=subs(B,[L_l L_r L_wheel_l L_wheel_r L_body_l L_body_r I_leg_l I_leg_r],[leg_varl leg_varr lll llr llml llmr i_pl i_pr]);
KK=lqrd(trans_A,trans_B,Q_cost,R_cost,0.001);
A5=double(trans_A);
B5=double(trans_B);
K5=KK;




