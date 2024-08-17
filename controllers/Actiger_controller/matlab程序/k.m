clear;
clc;
i=0;
L0=96;
L1=340;
L2=340;
for theta_2=-40/180*3.14:-0.1:-150/180*3.14
    for theta_1=-30/180*3.14:0.1:150/180*3.14
        for  theta_0=-22/180*3.14:0.05:22/180*3.14
          i=i+1;
            x(i)= -(L1 * sin(theta_1) + L2 * sin(theta_1 + theta_2));
            y(i)= L0*cos(theta_0) + sin(theta_0) * ( L1 * cos(theta_1) + L2 * cos(theta_1 + theta_2));
            z(i)= L0*sin(theta_0) - cos(theta_0) * ( L1 * cos(theta_1) + L2 * cos(theta_1 + theta_2));

        end
    end

end

