clear;
clc;

A=textread('result1.txt');
x = A(:,1);
y = A(:,2);
z = A(:,3);

for i = 1:1:size(x)
	plot3(x(i,:),y(i,:),z(i,:),'*');
    
    hold on;
    
end