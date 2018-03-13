function result = Newton_Raphson(f,dfdx,starting_value,eps) 
%     tol = 1e-6;
%     maxIter = 100;
%     eps = tolerance

    x = starting_value;
    while abs(f(x)) > eps
        x = x - f(x)/dfdx(x);
    end
    result = x;
end