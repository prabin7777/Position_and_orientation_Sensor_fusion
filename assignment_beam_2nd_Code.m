% Define the properties of the beam
length = 5; % Length of the beam (in meters)
num_elements = 4; % Number of elements used for discretization
element_length = length / num_elements; % Length of each element

% Define the material properties
E = 200e9; % Young's modulus of the beam material (in Pa)
I = 1e-6; % Second moment of area (in m^4)

% Define the applied load
load_per_unit_length = 1000; % Applied load per unit length (in N/m)

% Calculate the element stiffness matrix
k_element = (E * I / element_length^3) * [12, 6 * element_length, -12, 6 * element_length;
                                         6 * element_length, 4 * element_length^2, -6 * element_length, 2 * element_length^2;
                                         -12, -6 * element_length, 12, -6 * element_length;
                                         6 * element_length, 2 * element_length^2, -6 * element_length, 4 * element_length^2];

% Assemble the global stiffness matrix
k_global = zeros(num_elements + 1);
for i = 1:num_elements
    k_global(i:i+1, i:i+1) = k_global(i:i+1, i:i+1) + k_element;
end

% Apply boundary conditions (simply supported beam)
k_global(1, :) = [];
k_global(:, 1) = [];
F = zeros(num_elements, 1); % Applied loads vector
F(end) = -0.5 * load_per_unit_length * element_length; % Applied load at the last node

% Solve for displacements
displacements = k_global \ F;

% Calculate reactions
reactions = k_global * [0; displacements];

% Compute internal forces (bending moment and shear force)
moment = zeros(num_elements + 1, 1);
shear_force = zeros(num_elements + 1, 1);

for i = 1:num_elements
    moment(i+1) = moment(i) + (displacements(i+1) - displacements(i)) * (6 * load_per_unit_length * element_length^2) / (2 * E * I);
    shear_force(i+1) = shear_force(i) + (moment(i+1) - moment(i)) / element_length;
end

% Display the results
disp('Displacements:');
disp(displacements);
disp('Reactions:');
disp(reactions);
disp('Bending Moment:');
disp(moment);
disp('Shear Force:');
disp(shear_force);

% Plot the deflected shape of the beam
x = linspace(0, length, num_elements + 1); % x-coordinates of the nodes
y = [0; displacements]; % y-coordinates of the deflected shape
plot(x, y, 'b-', 'LineWidth', 2);
xlabel('Position (m)');
ylabel('Deflection (m)');
title('Deflected Shape of the Beam');
grid on;