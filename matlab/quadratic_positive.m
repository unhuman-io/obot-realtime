function sol = quadratic_positive(a, b, c)
%QUADRATIC_POSITIVE Summary of this function goes here
%   Detailed explanation goes here
    sol = [];
    if (a == 0)
        sol = -c/b;
    else
        s = b^2 - 4*a*c;
        if (s > 0)
            for q = [-1,1] 
                sol1 = (-b + q*sqrt(s))/2/a;
                if (sol1 > 0)
                    sol = [sol;sol1];
                end
            end
        end
    end
end

