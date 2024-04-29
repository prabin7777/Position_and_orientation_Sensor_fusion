% Constants
frequency = 900;  % Frequency in MHz
distance = linspace(1, 100, 100);  % Distance between transmitter and receiver in km
height_transmitter = 50;  % Height of the transmitter in meters
height_receiver = 2;  % Height of the receiver in meters
c = 3e8;  % Speed of light in m/s
lambda = c / (frequency * 1e6);  % Wavelength in meters

% Constants for Urban Area
A_urban = 69.55;  % for frequency in GHz
B_urban = 26.16;  % for frequency in GHz

% Path loss calculation
C = zeros(size(distance));
for i = 1:length(distance)
    if distance(i) < 20
        C(i) = 0;  % correction factor for distance < 20 km
    else
        C(i) = 4.98 + 4.78 * (log10(distance(i)/10))^2 - 18.33 * log10(distance(i)/10); % in dB
    end
end

path_loss = A_urban + B_urban * log10(distance) + C - 13.82 * log10(height_transmitter) ...
            - (44.9 - 6.55 * log10(height_transmitter)) * log10(lambda/1e6) + (1.1 * log10(frequency) - 0.7) * height_receiver - (1.56 * log10(frequency) - 0.8);

% Plotting
figure;
subplot(1,2,1);
plot(distance, path_loss, 'b', 'LineWidth', 2);
xlabel('Distance (km)');
ylabel('Path Loss (dB)');
title('Path Loss vs Distance');
grid on;

subplot(1,2,2);
height_transmitter_values = 10:10:100; % Vary transmitter height from 10 to 100 meters
path_loss_height = zeros(size(height_transmitter_values));
for i = 1:length(height_transmitter_values)
    path_loss_height(i) = A_urban + B_urban * log10(distance(50)) + C(50) - 13.82 * log10(height_transmitter_values(i)) ...
            - (44.9 - 6.55 * log10(height_transmitter_values(i))) * log10(lambda/1e6) + (1.1 * log10(frequency) - 0.7) * height_receiver - (1.56 * log10(frequency) - 0.8);
end
plot(height_transmitter_values, path_loss_height, 'r', 'LineWidth', 2);
xlabel('Transmitter Height (m)');
ylabel('Path Loss (dB)');
title('Path Loss vs Transmitter Height');
grid on;
