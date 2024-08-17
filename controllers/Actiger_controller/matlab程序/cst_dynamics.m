clear;
clc;
syms  t G m p0 p1 p2 p3 p4 p5 ...
      spatial_now_0 spatial_now_1 spatial_now_2 spatial_now_3 spatial_now_4 spatial_now_5 ...
      swing_now_0 swing_now_1 swing_now_2 swing_now_3 swing_now_4 swing_now_5...
      swing_dot_now_0 swing_dot_now_1 swing_dot_now_2 swing_dot_now_3 swing_dot_now_4 swing_dot_now_5...
      spatial_next_0 spatial_next_1 spatial_next_2 spatial_next_3 spatial_next_4 spatial_next_5 ...
      swing_next_0 swing_next_1 swing_next_2 swing_next_3 swing_next_4 swing_next_5...
      swing_dot_next_0 swing_dot_next_1 swing_dot_next_2 swing_dot_next_3 swing_dot_next_4 swing_dot_next_5...
      i0 i1 i2 ;
g=[0;0;G];
x=[swing_now_0; swing_now_1; swing_now_2; swing_now_3; swing_now_4; swing_now_5;...
   swing_dot_now_0; swing_dot_now_1; swing_dot_now_2; swing_dot_now_3; swing_dot_now_4; swing_dot_now_5;...
   swing_next_0; swing_next_1; swing_next_2; swing_next_3; swing_next_4; swing_next_5;...
   swing_dot_next_0; swing_dot_next_1; swing_dot_next_2; swing_dot_next_3; swing_dot_next_4; swing_dot_next_5];
     
spatial_pos_now=[spatial_now_0 ;spatial_now_1; spatial_now_2 ;spatial_now_3 ;spatial_now_4; spatial_now_5];
swing_pos_now=[swing_now_0; swing_now_1; swing_now_2; swing_now_3; swing_now_4; swing_now_5];
swing_vel_now=[swing_dot_now_0; swing_dot_now_1; swing_dot_now_2; swing_dot_now_3; swing_dot_now_4; swing_dot_now_5];

spatial_pos_next=[spatial_next_0; spatial_next_1; spatial_next_2; spatial_next_3; spatial_next_4; spatial_next_5];
swing_pos_next=[swing_next_0; swing_next_1; swing_next_2; swing_next_3; swing_next_4; swing_next_5];
swing_vel_next=[swing_dot_next_0; swing_dot_next_1; swing_dot_next_2; swing_dot_next_3; swing_dot_next_4; swing_dot_next_5];

cst(1:6,:)=(swing_pos_next-swing_pos_now)-0.5*t*(swing_vel_now+swing_vel_next);
cst(7:12,:)=(swing_vel_next-swing_vel_now)/t;
for cnt=1:12
    for cnt1=1:size(x)
        j(cnt,cnt1)=diff(cst(cnt,:),x(cnt1,:));
    end
end

% for cnt=1:size(x)
%     for cnt2=1:size(x)
%         h1(cnt,cnt2)=diff(j(1,cnt),x(cnt2,:));
%         h2(cnt,cnt2)=diff(j(2,cnt),x(cnt2,:));
%         h3(cnt,cnt2)=diff(j(3,cnt),x(cnt2,:));
%         h4(cnt,cnt2)=diff(j(4,cnt),x(cnt2,:));
%         h5(cnt,cnt2)=diff(j(5,cnt),x(cnt2,:));
%         h6(cnt,cnt2)=diff(j(6,cnt),x(cnt2,:));
%         h7(cnt,cnt2)=diff(j(7,cnt),x(cnt2,:));
%         h8(cnt,cnt2)=diff(j(8,cnt),x(cnt2,:));
%         h9(cnt,cnt2)=diff(j(9,cnt),x(cnt2,:));
%         h10(cnt,cnt2)=diff(j(10,cnt),x(cnt2,:));
%         h11(cnt,cnt2)=diff(j(11,cnt),x(cnt2,:));
%         h12(cnt,cnt2)=diff(j(12,cnt),x(cnt2,:));
%     end
% end

