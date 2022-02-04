clc; clear; close all

M = 20; % number of preambles
RA_attempts = (2:10);
k = linspace(2,10,9);
R = 2e3;
ep = 0.52e-6;
c = 3e8;
z = (ep*c)/2;
iterations = 10000;

syms r

for i = RA_attempts

    P_col_conv = 1-(1-(1/M)).^(k-1); % collision probability


    f_1 = r*(1-(R^2-(r-z)^2)/(M*R^2)).^k;
    f_2 = r*(1-((r+(z))^2)/(M*R^2)).^k;
    f_3 = r*(1-(4*r*z)/(M*R^2)).^k;
 
    func_integration = 1-(2/R^2)*(int(f_1, r, [R-z, R]) + int(f_2, r, [0, z]) + int(f_3, r, [z, R-z]));
    
    P_col_prop = round(vpa(func_integration),4);  % proposed collision probability

end

semilogy(k, P_col_conv, '-', LineWidth=1)

grid on
hold on

semilogy(k,P_col_prop, '--', LineWidth=1)
ylim([10^-3 1])

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

            prob_collision(j) = collision/iterations;  % simulated collision probability
end    
 
semilogy(k, prob_collision,'ks')
ylim([10^-3 1])

hold on


angles = linspace(0, 2*pi, 1000); % 1000 is the total number of points
xCenter = 0;
yCenter = 0;
x = R * cos(angles) + xCenter; 
y = R * sin(angles) + yCenter;
eNodeB = [xCenter,yCenter];
devices = 10;

prob_collision = zeros(1,9);

for j = 1:9
      RA = RA_attempts(j);

      theta_1 = rand*(2*pi);
      tagged_machine_device = randsample(devices,1,true);
      r_t = sqrt(rand)*R;
      x_tagged_machine_device = tagged_machine_device + r_t.*cos(theta_1);
      y_tagged_machine_device = tagged_machine_device + r_t.*sin(theta_1);
      r_o = sqrt(((x_tagged_machine_device-eNodeB(1))^2) + ((y_tagged_machine_device-eNodeB(2))^2));  %distance b/n tagged device and eNodeB

      collision =  0;
         
         

          for n = 1:iterations  

                M_preambles = randperm(M);

                theta = rand*2*pi;
                 r_m = (sqrt(rand(1,RA-1))*R);

                x_machine_devices = xCenter + r_m.*cos(theta);
                y_machine_devices = yCenter + r_m.*sin(theta);

                T_o = (2*r_o)/c; % TA value for tagged device

                x_region_tagged_device = r_o * cos(angles) + xCenter; 

                y_region_tagged_device = r_o * sin(angles) + yCenter;

                

                x_region_tagged_device_1 = (r_o-z) * cos(angles) + xCenter; 

                y_region_tagged_device_1 = (r_o-z) * sin(angles) + yCenter;


                x_region_tagged_device_2 = (r_o+z) * cos(angles) + xCenter; 

                y_region_tagged_device_2 = (r_o+z) * sin(angles) + yCenter;

                tagged_device_selection = randsample(M_preambles,1,true); 
            

                     for f = 1:RA-1
                         location_comparison = 0;

                        if (sqrt((x_machine_devices(f)-eNodeB(1))^2 + (y_machine_devices(f)-eNodeB(2))^2) >= (r_o-z)) && (sqrt((x_machine_devices(f)-eNodeB(1))^2 + (y_machine_devices(f)-eNodeB(2))^2) <= (r_o+z))
                           location_comparison = location_comparison + 1;
                           preamble_selection = randsample(M_preambles,1,true);
                           preamble_comparison = preamble_selection == tagged_device_selection;

                           if ((preamble_comparison == 1) && (location_comparison == 1))
                                collision = collision + 1;
                                break;
                          end
                        end

                     end

          end

                 prob_collision(j) = collision/iterations;

end     

semilogy(k, prob_collision,'b*', LineWidth = 2)
grid on
ylim([10^-3 1])


hold on

M = 5;
for i = RA_attempts
    P_col_conv = 1-(1-(1/M)).^(k-1); % collision probability


    f_1 = r*(1-(R^2-(r-z)^2)/(M*R^2)).^k;
    f_2 = r*(1-((r+(z))^2)/(M*R^2)).^k;
    f_3 = r*(1-(4*r*z)/(M*R^2)).^k;
 
    func_integration = 1-(2/R^2)*(int(f_1, r, [R-z, R]) + int(f_2, r, [0, z]) + int(f_3, r, [z, R-z]));
    
    P_col_prop = round(vpa(func_integration),4);  % proposed collision probability
    
end

semilogy(k, P_col_conv, '-', LineWidth=1)
ylim([10^-3 1])

hold on

semilogy(k,P_col_prop, '--', LineWidth=1)
ylim([10^-3 1])

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

            prob_collision(j) = collision/iterations;  % simulated collision probability
end    
 
semilogy(k, prob_collision,'bo')
ylim([10^-3 1])

hold on


prob_collision = zeros(1,9);

for j = 1:9
      RA = RA_attempts(j);

      theta_1 = rand*(2*pi);
      tagged_machine_device = randsample(devices,1,true);
      r_t = sqrt(rand)*R;
      x_tagged_machine_device = tagged_machine_device + r_t.*cos(theta_1);
      y_tagged_machine_device = tagged_machine_device + r_t.*sin(theta_1);
      r_o = sqrt(((x_tagged_machine_device-eNodeB(1))^2) + ((y_tagged_machine_device-eNodeB(2))^2));  %distance b/n tagged device and eNodeB

      collision =  0;
         
         

          for n = 1:iterations  

                M_preambles = randperm(M);

                theta = rand*2*pi;
                r_s = (sqrt(rand(1,RA-1))*R);
   
                x_machine_devices = xCenter + r_s.*cos(theta);
                y_machine_devices = yCenter + r_s.*sin(theta);

                T_o = (2*r_o)/c; % TA value for tagged device

                x_region_tagged_device = r_o * cos(angles) + xCenter; 

                y_region_tagged_device = r_o * sin(angles) + yCenter;


                x_region_tagged_device_1 = (r_o-z) * cos(angles) + xCenter; 

                y_region_tagged_device_1 = (r_o-z) * sin(angles) + yCenter;


                x_region_tagged_device_2 = (r_o+z) * cos(angles) + xCenter; 

                y_region_tagged_device_2 = (r_o+z) * sin(angles) + yCenter;

                tagged_device_selection = randsample(M_preambles,1,true);
                

                     for f = 1:RA-1
                         location_comparison = 0;

                        if ((x_machine_devices(f)-eNodeB(1))^2 + (y_machine_devices(f)-eNodeB(2))^2 >= (r_o-z)^2 && (x_machine_devices(f)-eNodeB(1))^2 + (y_machine_devices(f)-eNodeB(2))^2 <= (r_o+z)^2)
                           location_comparison = location_comparison + 1;
                           preamble_selection = randsample(M_preambles,1,true);
                           preamble_comparison = preamble_selection == tagged_device_selection;

                           if ((preamble_comparison == 1) && (location_comparison == 1))
                                collision = collision + 1;
                                break;
                          end
                        end

                     end

          end

                 prob_collision(j) = collision/iterations;

end     

semilogy(k, prob_collision,'r+', LineWidth = 2)
grid on
ylim([10^-3 1])



legend('M=20 conv(anal)', 'M=20 prop(anal)', 'M=20 conv(sim)','M=20, prop(sim)', 'M=5 conv(anal)', 'M=5 prop(anal)', 'M=5 conv(sim)','M=5 prop(sim)',  Location = 'best')
xlabel('Number of RA attempts from machine devices on a single RA slot (k+1)')
ylabel('Collision Probability (Pc)')
