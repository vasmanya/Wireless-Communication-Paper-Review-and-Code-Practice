clc; clear; close all

k = (2:10); %RA_attempts
e = 0.52e-6;
c = 3e8;
R = 2e3;
y = (e*c)/2;
iterations = 10000; % number of iterations

syms r

M = 20;

for i = k
     P_col_conv = 1-(1-(1/M)).^(k-1); % conventional collision probability

     f = (2/(R^2));

     f_1 = r*(1-((R^2)-((r-y).^2))/(M*(R^2))).^k;

     f_2 = r*(1-(((r+y).^2))/(M*(R^2))).^k;

     f_3 = r*(1-((4*r*y))/(M*(R^2))).^k;

     func_1 = vpa(int(f_1, r, [R-y R]));

     func_2 = vpa(int(f_2, r, [0 y]));

     func_3 = vpa(int(f_3, r, [y R-y]));


     P_col_prop = 1-(f*(func_1 + func_2 + func_3)); % proposed collision probability
end

 semilogy(k,P_col_conv, '-')

 grid on
 
 hold on 
 
 semilogy(k,P_col_prop, "--")
 
 ylim([10^-3 1])


 hold on


 M = 5;

 for i = k
    P_col_conv = 1-(1-(1/M)).^(k-1); % conventional collision probability

    f = (2/R^2);

    f_1 = r*(1-((R^2)-((r-y).^2))/(M*(R^2))).^k;

    f_2 = r*(1-(((r+y).^2))/(M*(R^2))).^k;

    f_3 = r*(1 -((4*r*y))/(M*(R^2))).^k;

    func_1 = vpa(int(f_1, r, [R-y R]));

    func_2 = vpa(int(f_2, r, [0 y]));

    func_3 = vpa(int(f_3, r, [y R-y]));


    P_col_prop = 1-(f*(func_1 + func_2 + func_3));  % proposed collision probability
   
end

 semilogy(k,P_col_conv, '-')

 hold on 
 
 semilogy(k,P_col_prop, LineStyle='-.')
 
 ylim([10^-3 1])


 
legend('M=20 conv(anal)', 'M=20 prop(anal)', 'M=5 conv(anal)', 'M=5 prop(anal)', Location = 'best')
xlabel('Number of RA attempts from machine devices on a single RA slot (k+1)')
ylabel('Collision Probability (Pc)')