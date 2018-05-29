M_in  = 0.686; %Nm


Z_ring = zeros(1, 100);
Z_sun = zeros(1, 100);
Z_planet = zeros(1, 100);

for i = 1:100
    Z_ring(i) = i;
    for j = 1:100
        Z_sun(j) = j;
        M_out(i,j) = M_in*((Z_ring(i) + Z_sun(j))/Z_sun(j));       
    end
end

[X,Y] = meshgrid(Z_ring, Z_sun);
Z = M_out;

surfl(X,Y,Z);


