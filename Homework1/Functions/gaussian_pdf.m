function [pdf] = gaussian_pdf(mean, stdev)
syms x;
pdf = (1/(stdev*sqrt(2*pi)))*exp(-0.5*(((x - mean) / (stdev))^2));
end

