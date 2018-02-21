function [t,y]=NewtonRaphson_Implementing_BackwardEuler(f,df,y0,t0,tf,n)
    
    h=(tf-t0)/n;
    t=linspace(t0,tf,n+1);
    y=zeros(n+1,length(y0));
    y(1,:)=y0;
    for i=1:n
        x0=y(i,:)';
        x1=x0-inv(eye(length(y0))-h*feval(df,t(i),x0))*(x0-h*feval(f,t(i),x0)'-y(i,:)');
        while (norm(x1-x0)>0.0001)
            x0=x1;
            x1=x0-inv(eye(length(y0))-h*feval(df,t(i),x0))*(x0-h*feval(f,t(i),x0)'-y(i,:)');
        end
        y(i+1,:)=x1';
end
end