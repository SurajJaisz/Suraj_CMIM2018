function result = Newton_Raphson(fun,funD,starting_value,tolerance,maxIter) 

x = starting_value;
i = 0;
while abs(fun(x)) > tolerance
    x = x - funD(x)\fun(x);
    i = i+1;
    if i >= maxIter
        disp('Maximum Number of Iterations Reached (Newton Raphson Method)');
        return
    end
    result = x;
end