function starts = randomStarts(n, seed, leftOnly)
% Match the starter's order and distributions for the two-sided starts.
rng(seed, 'twister');
starts = zeros(n, 5);
for k = 1:n
    if leftOnly
        side = -1;
    else
        side = sign(rand - .5);
    end
    starts(k,:) = [side*(2.5+2*rand), 2.7+.6*rand, ...
        pi*(2*rand-1), .5*rand, .4*rand-.2];
end
end
