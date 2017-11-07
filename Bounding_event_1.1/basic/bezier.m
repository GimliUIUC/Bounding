function b = bezier(s,coeff1,coeff2)

    if(0 <= s && s <= 0.5)
        b = polyval_bz(coeff1, s*2);
    elseif(0.5 < s && s <= 1)
        b = polyval_bz(coeff2, s*2-1);
    else
        b = 0;
    end

end
