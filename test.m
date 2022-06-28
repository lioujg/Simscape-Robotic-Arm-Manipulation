a = [0, 1; 2, 3];
b = [4, 5; 6, 7];
c = [a,              zeros(2, 2);
     zeros(2, 2),              b];
d = c * 2;
e = zeros(3, 1);

% c_sum = sum(c, 2)
% size(c, 2)


mass = 0.27;
center_of_mass = [0 0 0];
inertia_matrix = [0.000204525 0 0 4.05e-06 0 0.000204525];
r = -0.15;
theta_o = [mass mass*center_of_mass inertia_matrix r].';


