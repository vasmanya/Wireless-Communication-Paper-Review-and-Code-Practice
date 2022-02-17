clc;clear;close all;

%Parameters
N_t = 10; %transmit antennas
N_r = 4; %receive antennas
SNR_dB = (1:1:20); %signal to noise ratio
SNR = 10.^(SNR_dB/10); % Absolute Value of SNR
I_N_R = eye(N_r); %identity of N_r x N_r matrix
iterations = 1000;
Capacity = zeros(1,20);

for j = 1:iterations
    H = randn(N_r,N_t) + 1i*randn(N_r,N_t); % channel gains
    H_H = H*ctranspose(H);


    %Water filling algorithm for power allocation
    for  i = 1:length(SNR_dB)
         P_t = 10.^(SNR_dB(i)/10);   % Assuming noise power to be equal to 1
         [U, S, V] = svd(H_H);
         C_k = diag(S); % channel state information given by |h_k|/noise power^2
         g_lambda_mu = 1e4; %number of points to compute lagrangian dual function
         mu = linspace(0.7,0.698,g_lambda_mu);
         allocated_power = max(1./mu-(1./C_k), 0); %x^+ = max(x,0)

         %Lagrangian function
         g_mu = sum(log(1+(allocated_power.*(repmat(C_k,[1,g_lambda_mu]))))) - mu.*((sum(allocated_power))-P_t);
         [min_g_mu, Ind] = find(g_mu == min(g_mu));
         optimal_mu = mu(Ind);
         optimal_power = max(1./optimal_mu - 1./C_k, 0);
         Q_opt = diag(optimal_power);
         Capacity(i) = Capacity(i) + log(real(det(I_N_R + (P_t/N_t)*Q_opt*H_H)));
    end
end

Capacity = Capacity/iterations;

plot(SNR_dB,Capacity,':d');
ylim([0 30]), xlim([5 20])

hold on


N_t = 20; %transmit antennas

Capacity = zeros(1,20);

for j = 1:iterations
    H = randn(N_r,N_t) + 1i*randn(N_r,N_t); % channel gains
    H_H = H*ctranspose(H);

    %Water filling algorithm for power allocation
    for  i = 1:length(SNR_dB)
         P_t = 10^(SNR_dB(i)/10);   % Assuming noise power to be equal to 1
         [U, S, V] = svd(H_H);
         C_k = diag(S); % channel state information given by |h_k|/noise power^2
         g_lambda_mu = 1e4; %number of points to compute lagrangian dual function
         mu = linspace(0.4,0.398,g_lambda_mu);
         allocated_power = max(1./mu-(1./C_k), 0); %x^+ = max(x,0)

         %Lagrangian function
         g_mu = sum(log(1+(allocated_power.*(repmat(C_k,[1,g_lambda_mu]))))) - mu.*((sum(allocated_power))-P_t);
         [min_g_mu, Ind] = find(g_mu == min(g_mu));
         optimal_mu = mu(Ind);
         optimal_power = max(1./optimal_mu - 1./C_k, 0);
         Q_opt = diag(optimal_power);
         Capacity(i) = Capacity(i) + log(real(det(I_N_R + (P_t/N_t)*Q_opt*H_H)));
    end
end

Capacity = Capacity/iterations;

plot(SNR_dB,Capacity,':*');
ylim([0 30]), xlim([5 20])

legend('E[Capacity](N_t = 10)','E[Capacity](N_t = 20)', Location='best')


