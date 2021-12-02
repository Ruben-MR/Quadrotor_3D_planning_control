function Q = getQ(n_seg, n_order, ts)
    Q = [];
    for k = 1:n_seg
        %#####################################################
        % STEP 1.1: calculate Q_k of the k-th segment 
        Q_k = zeros(n_order+1, n_order+1);
        t_k = ts(k);
        for m = 4:n_order
            for n = 4:n_order
                den = m+n-7;
                Q_k(m+1,n+1) = m*(m-1)*(m-2)*(m-3)*n*(n-1)*(n-2)*(n-3)/den*(t_k^den);
            end
        end
        Q = blkdiag(Q, Q_k);
    end
    size(Q)
end