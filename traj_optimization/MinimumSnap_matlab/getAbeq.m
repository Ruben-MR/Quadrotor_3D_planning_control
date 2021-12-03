function [Aeq beq]= getAbeq(n_seg, n_order, waypoints, ts, start_cond, end_cond)
    n_all_poly = n_seg*(n_order+1);
    %#####################################################
    % p,v,a,j constraint in start, 
    Aeq_start = zeros(4, n_all_poly);
    % STEP 2.1: write expression of Aeq_start and beq_start
    for n = 1:4
        Aeq_start(n, n) = factorial(n-1);
    end 
    beq_start = start_cond';  % transform the row to column
    
    %#####################################################
    % p,v,a constraint in end
    Aeq_end = zeros(4, n_all_poly);
    % STEP 2.2: write expression of Aeq_end and beq_end
    t_end = ts(end);
    for m =1:4
        for n=m:n_order+1
            Aeq_end(m,(n_seg-1)*(n_order+1)+n) = (t_end^(n-m))*factorial(n-1)/factorial(n-m);
        end
    end
    beq_end = end_cond';
    
    %#####################################################
    % position constrain in all middle waypoints
    Aeq_wp = zeros(n_seg-1, n_all_poly);
    beq_wp = zeros(n_seg-1, 1);
    % STEP 2.3: write expression of Aeq_wp and beq_wp
    for m = 1:n_seg-1
        t_m = ts(m);
        for n = 1:n_order+1
            Aeq_wp(m, (m-1)*(n_order+1)+n) = t_m^(n-1);
        end
        beq_wp(m,1) = waypoints(m+1);
    end
    
    %#####################################################
    % position continuity constrain between each 2 segments
    beq_con_p = zeros(n_seg-1, 1);
    % STEP 2.4: write expression of Aeq_con_p and beq_con_p
    function Aeq_con = get_Aeq_con(n_seg, n_order, ts, k)
        Aeq_con = zeros(n_seg-1, n_seg*(n_order+1));
        for i = 1:n_seg-1
            t_i = ts(i);
            id = (i-1)*(n_order+1);
            for j = k+1:n_order+1
                Aeq_con(i, id+j) = t_i^(j-1-k)*factorial(j-1)/factorial(j-k-1);
            end
            Aeq_con(i, i*(n_order+1)+k+1) = -factorial(k);
        end
    end
    Aeq_con_p = get_Aeq_con(n_seg, n_order, ts, 0);
    
    %#####################################################
    % velocity continuity constrain between each 2 segments
    beq_con_v = zeros(n_seg-1, 1);
    % STEP 2.5: write expression of Aeq_con_v and beq_con_v
    Aeq_con_v = get_Aeq_con(n_seg, n_order, ts, 1);

    %#####################################################
    % acceleration continuity constrain between each 2 segments
    beq_con_a = zeros(n_seg-1, 1);
    % STEP 2.6: write expression of Aeq_con_a and beq_con_a
    Aeq_con_a = get_Aeq_con(n_seg, n_order, ts, 2);
    
    %#####################################################
    % jerk continuity constrain between each 2 segments
    beq_con_j = zeros(n_seg-1, 1);
    % STEP 2.7: write expression of Aeq_con_j and beq_con_j
    Aeq_con_j = get_Aeq_con(n_seg, n_order, ts, 3);
    
    %#####################################################
    % combine all components to form Aeq and beq   
    Aeq_con = [Aeq_con_p; Aeq_con_v; Aeq_con_a; Aeq_con_j];
    beq_con = [beq_con_p; beq_con_v; beq_con_a; beq_con_j];
    Aeq = [Aeq_start; Aeq_end; Aeq_wp; Aeq_con];
    beq = [beq_start; beq_end; beq_wp; beq_con];
end