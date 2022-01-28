clc; clear; close all

M = 20; % number of preambles
RA_attempts = (2:10);
k = linspace(2,10,9);
R = 2e3;
ep = 0.52e-6;
c = 3e8;
iterations = 10000;

syms r

for i = RA_attempts

    P_col_conv = 1-(1-(1/M)).^(k-1); % collision probability


    f_1 = r*(1-(R^2-(r-ep*c/2)^2)/(M*R^2)).^k;
    f_2 = r*(1-((r+(ep*c/2))^2)/(M*R^2)).^k;
    f_3 = r*(1-(4*r*ep*c/2)/(M*R^2)).^k;
 
    func_integration = 1-(2/R^2)*(int(f_1, r, [R-(ep*c)/2, R]) + int(f_2, r, [0, (ep*c)/2]) + int(f_3, r, [ep*c/2, R-(ep*c)/2]));
    
    P_col_prop = round(vpa(func_integration),4);  % proposed collision probability

end

semilogy(k, P_col_conv, '-', LineWidth=1)

grid on
hold on

semilogy(k,P_col_prop, '--', LineWidth=1)
ylim([10^-4 1])

hold on

preamble_selection = zeros();
prob_collision = zeros(1,9);
 
for j = 1:9
      RA = k(j);
      collision =  0;

      for n = 1:iterations
            M_preambles = randperm(M);
            tagged_device_selection = randsample(M_preambles,1,true);

          for i = 1:(RA-1)             
               preamble_selection(i) = randsample(M_preambles,1,true);
          end

          preamble_check = preamble_selection(:);
          preamble_comparison = sum(logical(preamble_check == tagged_device_selection));
              
          if (preamble_comparison >= 1)
               collision = collision + 1;
          end
 
     end  

            prob_collision(j) = collision/iterations;
end    
 
semilogy(k, prob_collision,'ks')
ylim([10^-4 1])


hold on


M = 5;
for i = RA_attempts
    P_col_conv = 1-(1-(1/M)).^(k-1); % collision probability


    f_1 = r*(1-(R^2-(r-ep*c/2)^2)/(M*R^2)).^k;
    f_2 = r*(1-((r+(ep*c/2))^2)/(M*R^2)).^k;
    f_3 = r*(1-(4*r*ep*c/2)/(M*R^2)).^k;
 
    func_integration = 1-(2/R^2)*(int(f_1, r, [R-(ep*c/2), R]) + int(f_2, r, [0, (ep*c)/2]) + int(f_3, r, [ep*c/2, R-(ep*c)/2]));
    
    P_col_prop = round(vpa(func_integration),4);  % proposed collision probability
    
end

semilogy(k, P_col_conv, '-', LineWidth=1)
ylim([10^-4 1])

hold on

semilogy(k,P_col_prop, '--', LineWidth=1)
ylim([10^-4 1])

hold on

preamble_selection = zeros();
prob_collision = zeros(1,9);
 
for j = 1:9
      RA = k(j);
      collision =  0;

      for n = 1:iterations
            M_preambles = randperm(M);
            tagged_device_selection = randsample(M_preambles,1,true);

          for i = 1:(RA-1)             
               preamble_selection(i) = randsample(M_preambles,1,true);
          end

          preamble_check = preamble_selection(:);
          preamble_comparison = sum(logical(preamble_check == tagged_device_selection));
              
          if (preamble_comparison >= 1)
               collision = collision + 1;
          end
 
     end  

            prob_collision(j) = collision/iterations;
end    
 
semilogy(k, prob_collision,'bo')
ylim([10^-4 1])


legend('M=20 conv(anal)', 'M=20 prop(anal)', 'M=20 conv(sim)', 'M=5 conv(anal)', 'M=5 prop(anal)', 'M=5 conv(sim)',  Location = 'best')
xlabel('Number of RA attempts from machine devices on a single RA slot (k+1)')
ylabel('Collision Probability (Pc)')
