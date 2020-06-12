function TF = polysym2tf(polysymFunc)
    [Num, Den] = numden(polysymFunc);
    TFN = sym2poly(Num);
    TFD = sym2poly(Den);
    TF = tf(TFN, TFD);
end