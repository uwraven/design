function W = skew4(w)
    % From an angular rate vector, return the skew symmetric rate matrix
    W = [
        0       -w(3)   w(2)    w(1)
        w(3)    0       -w(1)   w(2)
        -w(2)   w(1)    0       w(3)
        -w(1)   -w(2)   -w(3)   0
    ];
end
