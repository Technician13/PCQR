clear;
clc;

syms  Ts dw p0 p1 p2 v0 v1 v2 f0 f1 f2 r0 r1 r2 stf0 stf1 stf2 str0 str1 str2;

x= [ p0 ;p1; p2 ;v0 ;v1 ;v2 ;f0 ;f1 ;f2 ;r0 ;r1 ;r2];

euler =[0; 0 ;dw];
pos=[p0 ;p1 ;p2];

swing_front=[f0 ;f1 ;f2];
swing_rear=[r0 ;r1 ;r2];

stance_front=[stf0; stf1; stf2];
stance_rear=[str0; str1; str2];

rotation=RotationMatrix(euler);

swing_front_pos_hip = rotation * (swing_front - pos);
swing_rear_pos_hip = rotation * (swing_rear - pos);

stance_front_pos_hip = rotation * (stance_front - pos);
stance_rear_pos_hip = rotation * (stance_rear - pos);

cst(1,:)=swing_front_pos_hip(1,:);
cst(2,:)=swing_front_pos_hip(2,:) + 3.556 * swing_front_pos_hip(3,:); 
cst(3,:)=-swing_front_pos_hip(2,:) + 3.556 * swing_front_pos_hip(3,:); 
cst(4,:)=swing_front_pos_hip(3,:);

cst(5,:)=swing_rear_pos_hip(1,:);
cst(6,:)=swing_rear_pos_hip(2,:) + 3.556 * swing_rear_pos_hip(3,:); 
cst(7,:)=-swing_rear_pos_hip(2,:) + 3.556 * swing_rear_pos_hip(3,:); 
cst(8,:)=swing_rear_pos_hip(3,:);

cst(9,:)=swing_front_pos_hip(1) - swing_rear_pos_hip(1);

cst(10,:)=swing_front_pos_hip(2) - swing_rear_pos_hip(2);

cst(11,:)=stance_front_pos_hip(1,:);
cst(12,:)=stance_front_pos_hip(2,:) + 3.556 * stance_front_pos_hip(3,:); 
cst(13,:)=-stance_front_pos_hip(2,:) + 3.556 * stance_front_pos_hip(3,:); 
cst(14,:)=stance_front_pos_hip(3,:);

cst(15,:)=stance_rear_pos_hip(1,:);
cst(16,:)=stance_rear_pos_hip(2,:) + 3.556 * stance_rear_pos_hip(3,:); 
cst(17,:)=-stance_rear_pos_hip(2,:) + 3.556 * stance_rear_pos_hip(3,:); 
cst(18,:)=stance_rear_pos_hip(3,:);


for cnt=1:18
    for cnt1=1:size(x)
        j(cnt,cnt1)=diff(cst(cnt,:),x(cnt1,:));
    end
end


for cnt=1:size(x)
    for cnt1=1:size(x)
        h1(cnt,cnt1)=diff(j(1,cnt),x(cnt1,:));
        h2(cnt,cnt1)=diff(j(2,cnt),x(cnt1,:));
        h3(cnt,cnt1)=diff(j(3,cnt),x(cnt1,:));
        h4(cnt,cnt1)=diff(j(4,cnt),x(cnt1,:));
        h5(cnt,cnt1)=diff(j(5,cnt),x(cnt1,:));
        h6(cnt,cnt1)=diff(j(6,cnt),x(cnt1,:));
        h7(cnt,cnt1)=diff(j(7,cnt),x(cnt1,:));
        h8(cnt,cnt1)=diff(j(8,cnt),x(cnt1,:));
        h9(cnt,cnt1)=diff(j(9,cnt),x(cnt1,:));
        h10(cnt,cnt1)=diff(j(10,cnt),x(cnt1,:));
        h11(cnt,cnt1)=diff(j(11,cnt),x(cnt1,:));
        h12(cnt,cnt1)=diff(j(12,cnt),x(cnt1,:));
        h13(cnt,cnt1)=diff(j(13,cnt),x(cnt1,:));
        h14(cnt,cnt1)=diff(j(14,cnt),x(cnt1,:));
        h15(cnt,cnt1)=diff(j(15,cnt),x(cnt1,:));
        h16(cnt,cnt1)=diff(j(16,cnt),x(cnt1,:));
        h17(cnt,cnt1)=diff(j(17,cnt),x(cnt1,:));
        h18(cnt,cnt1)=diff(j(18,cnt),x(cnt1,:));
    end
end



function r = RotationMatrix(actul_body_angle)

	roll = actul_body_angle(1);
	pitch = actul_body_angle(2);
	yaw = actul_body_angle(3);

	r(1, 1) = cos(yaw);
	r(1, 2) = sin(yaw);
	r(1, 3) = 0.0;
	r(2, 1) = - sin(yaw);
	r(2, 2) = cos(yaw);
	r(2, 3) = 0.0;
	r(3, 1) = 0.0;
	r(3, 2) = 0.0;
	r(3, 3) = 1.0;


end
