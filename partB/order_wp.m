function [order, cost] = order_wp(D)
n = size(D,1) - 1;
P = perms(1:n);
cost = inf;
for k = 1:size(P,1)
    idx = [1, P(k,:)+1];
    c = sum(D(sub2ind(size(D), idx(1:end-1), idx(2:end))));
    if c < cost, cost = c; order = P(k,:); end
end
end