clear;
syms k_thigh_len k_calf_len k_joint_len phi1_  phi4_ phi0_ dotphi1_ dotphi4_ dotphi0_ l0_ dotl0_ phi2_ phi3_
fl1=dotphi1_*k_thigh_len*(-sin(phi1_)*k_calf_len*cos(phi2_)+cos(phi1_)*k_calf_len*sin(phi2_));
fl2=dotl0_*(cos(phi0_)*k_calf_len*cos(phi2_)+sin(phi0_)*k_calf_len*sin(phi2_));
fl3=dotphi0_*l0_*(-sin(phi0_)*k_calf_len*cos(phi2_)+cos(phi0_)*k_calf_len*sin(phi2_));
eq1=fl1-fl2-fl3;
fr1=dotphi4_*k_thigh_len*(-sin(phi4_)*k_calf_len*cos(phi3_)+cos(phi4_)*k_calf_len*sin(phi3_));
fr2=dotl0_*(cos(phi0_)*k_calf_len*cos(phi3_)+sin(phi0_)*k_calf_len*sin(phi3_));
fr3=dotphi0_*l0_*(-sin(phi0_)*k_calf_len*cos(phi3_)+cos(phi0_)*k_calf_len*sin(phi3_));
eq2=fr1-fr2-fr3;
[dotl0_ ,dotphi0_] = solve([eq1,eq2], [dotl0_,dotphi0_]);
f=simplify(vpa(subs(dotphi0_, dotphi1_, 0)));
% [dotphi1_ ,dotphi4_] = solve([eq1,eq2], [dotphi1_,dotphi4_]);
% f=simplify(vpa(subs(dotphi4_, dotl0_, 0)));
disp(f);