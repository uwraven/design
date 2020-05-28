function result = bryson(vec)
    result = zeros(length(vec), 1);
    for i = 1:length(vec)
        if (~vec(i) == 0)
            result(i) = 1 / vec(i)^2;
        end
    end
end