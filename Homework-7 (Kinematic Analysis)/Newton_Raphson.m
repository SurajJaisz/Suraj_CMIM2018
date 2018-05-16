function x = Newton_Raphson(fun,funD,starting_value,tol,maxIter) 

x = starting_value;
i = 0;

while any(abs(fun(x)) > tol)
    x = x - funD(x)\fun(x);
    i = i+1;
    if i >= maxIter
        disp('Maximum Number of Iterations Reached (Newton Raphson Method)');
        return
    end
end