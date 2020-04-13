function [x] = rejection_sample_p3(n)
%Does Rejection sampling for the function below
maxFX =  1;
x = zeros(1,n);

i = 1;
while(i <= n)
    rand  = uniform_random(-2, 2);
    rand2 = uniform_random(0, maxFX);
    if rand2 <= f(rand)
        x(i) = rand;
        i = i + 1;
    end
end

end

function y = f(x)

if (x <= 1 && x >= -1)
    y = abs(x);
else
    y = 0;
end

end

