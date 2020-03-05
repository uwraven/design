function result = clamp(range, value)
    if (value < range(1))
        result = range(1);
    elseif (value > range(2))
        result = range(2);
    else
        result = value;
    end
end