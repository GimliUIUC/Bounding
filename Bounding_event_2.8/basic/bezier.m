function b = bezier(s,coeff1,coeff2)

    b = zeros(size(s));

    idx1 = (s >= 0) & (s <= 0.5);
    idx2 = (s >= 0.5) & (s <= 1);
    
    b(idx1) = polyval_bz(coeff1, s(idx1)*2);
    b(idx2) = polyval_bz(coeff2, s(idx2)*2 - 1);
%     if(0 <= s && s <= 0.5)
%         b = polyval_bz(coeff1, s*2);
%     elseif(0.5 < s && s <= 1)
%         b = polyval_bz(coeff2, s*2-1);
%     else
%         b = 0;
%     end

end
