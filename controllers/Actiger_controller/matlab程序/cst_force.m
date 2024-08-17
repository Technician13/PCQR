clear;
clc;

syms nx ny nz f0 f1 f2;
f=[f0;f1;f2];
n=[nx; ny; nz];
m=f.'*n;
j_f(1)=diff(m,f0);
j_f(2)=diff(m,f1);
j_f(3)=diff(m,f2);


