function f = polyFun(x,u,T)

    f(1,1) =  .320*x(1)^2 +u(1)+0.7*x(1) ;%+u(1)^2+u(2)^2;

    f(2,1) = .320*u(2)*x(1)+ 0.4*x(2)^2+0.09*x(1);

end

%------------- END OF CODE --------------