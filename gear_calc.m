function[M_out, Z_planet] = gear_calc(Z_sun, Z_ring)

M_in  = 0.686; %Nm
M_out = M_in*((Z_ring + Z_sun)/Z_sun);
Z_planet = (Z_ring - Z_sun)/2;

end


