function P = build_path(paths, order)
idx = [1, order+1];
W = paths{idx(1), idx(2)};
for k = 2:numel(idx)-1
    seg = paths{idx(k), idx(k+1)};
    W = [W; seg(2:end,:)];
end
P = W';
end