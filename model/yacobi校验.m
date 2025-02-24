clear;
syms k_thigh_len k_calf_len k_joint_len phi1_  phi4_ 
x_b_ = k_thigh_len *cos(phi1_);
y_b_ = k_thigh_len *sin(phi1_);
x_d_ = k_joint_len + k_thigh_len * cos(phi4_);
y_d_ = k_thigh_len *sin(phi4_);
bd_ = (x_d_ - x_b_)^2 + (y_d_ - y_b_)^2;
a0_ = 2 * k_calf_len * (x_d_ - x_b_);
b0_ = 2 * k_calf_len * (y_d_ - y_b_);
phi2_ = 2 * atan((b0_ + sqrt(a0_^2 + b0_^2 - bd_^2))/(a0_ + bd_));
x_c_ = x_b_ + k_calf_len * cos(phi2_);
y_c_ = y_b_ + k_calf_len * sin(phi2_);
phi0_ = atan(y_c_/( x_c_ - k_joint_len / 2));
l0_ = sqrt((x_c_ - k_joint_len / 2)^2 + y_c_^2);
phi3_ = atan(y_c_ - y_d_, x_c_ - x_d_);
Ja = jacobian(l0_, phi4_);
disp(Ja);
f=simplify(vpa(subs(Ja, [k_thigh_len,k_calf_len,k_joint_len,phi1_,phi4_],[0.15,0.27,0.15,4.01,-0.84])));
disp(f);