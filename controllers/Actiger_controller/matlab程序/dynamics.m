clear;clc;

syms f00 f01 f02 f10 f11 f12;
m=26.0;
i=[0.2,0,0;
    0,0.3,0;
    0,0,0.4];
g=[0.0;0.0;-9.8];
stance_pos_lf=[0.53;0.14;0.0];
stance_pos_rr=[-0.13;-0.11;0.0];

body_pos=[0.0;0.0;0.5];

f0=[f00;f01;f02];
f1=[f10;f11;f12];

c1=cross(stance_pos_lf-body_pos,f0);
c2=cross(stance_pos_rr-body_pos,f1);

acc(1:3,:)=i*(c1+c2);
acc(4:6,:)=(f0+f1)/m+g;

[f00,f01,f02,f10,f11,f12]=solve(acc(2,1)==0.0,acc(3,1)==0.0,acc(4,1)==0.0,acc(5,1)==0.0,acc(6,1)==0.0)

