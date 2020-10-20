function si = xy_to_si(p, n)
% takes a tuple p=(x, y), and returns the index of the position. if p is a
% matrix then it as parsed as each row corresponding to a tuple
x = p(:,1);
y = p(:,2);
si = n*(y-1) + x;
end