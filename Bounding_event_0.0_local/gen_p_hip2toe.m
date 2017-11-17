function p_hip2toe = gen_p_hip2toe(s,control)
%% --- generate vector from hip to toe ---

    coeff_x = control.coeff_x;
    coeff_z = control.coeff_z;
    x = polyval_bz(coeff_x, s);
    z = polyval_bz(coeff_z, s);
    p_hip2toe = [x;z]; 
        
end