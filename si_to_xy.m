function p = si_to_xy(si, n)
% takes a position index and maps to a tuple p=(x, y). if si is a vector
% then then the result will be a matrix with each row corresponding to a
% tuple
x = mod(si, n);
x(x == 0) = n; % fix the 1 based wraparound
y = (si - x)/n + 1;
p = [x y];
end