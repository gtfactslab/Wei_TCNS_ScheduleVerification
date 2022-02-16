function [safe_set, C_check_mult,Reroute, N_vec,edge_reroute,edge_undef] = verification_backup(G,R,Demand_R,S,v_0,w,Backup)
% safety verification of closing the node v_0.
% input:
% G -- Graph
% R -- routes
% S -- schedules (a set of departure times of UAVs)
% v_0 -- the node being closed
% output: 
% safe_set -- |G.V|x1 vector that indicates safety of each node
% C_check{v} -- [start, end, #rerouted2v, #unaffected2v, c_v >= #rerouted2v + #unaffected2v?]
N = length(S);
safe_set = zeros(length(G.V),1);
num_set = [];
% Compute occupation time for each UAV at each node according to current schedule
occupation = cell(length(G.V),1);
for i = 1:length(G.V)
    occupation{i} = zeros(N,3);
end
for i = 1:N
    dpT = S(i);
    route = Demand_R(i);    % find the required route
    route_v = R{route,5};   % nodes along the route
    for k_v = 2:length(route_v)
        v = route_v(k_v);
        occupation{v}(i,1) = i;
        occupation{v}(i,2) = dpT+R{route,4}(1,k_v-1);
        occupation{v}(i,3) = dpT+R{route,4}(2,k_v-1)+w;
    end
end

% Verify safty of schedule when all nodes are functioning (0-node close)
if isempty(v_0)
    % check the beginning of every section for the aggregated of UAVs at
    % that time
    num_set = zeros(N,length(G.V));     % number of UAVs at v after k'th UAV landing at the node
    for v = 1:length(G.V)
        if ismember(v,G.ori)
            continue;
        end
        for k = 1:N
            land_early = occupation{v}(k,2);  % earliest arrival time at v
            % count the number of UAVs whose staying time (M) include land_early
            land_inclusion = (occupation{v}(:,2) <= land_early) & (occupation{v}(:,3) > land_early);
            num_set(k,v) = sum(land_inclusion,1);   
        end
    end
    C_check_mult = [];
    N_vec = [];
    Reroute = [];
    edge_reroute=[];
    edge_undef=[];
    safe_set(:) = G.C>=max(num_set',[],2);
else    % v_0 is closed

    % compute G'.Delta with the node disabled
    v0_ind = find(v_0 == G.V,1);
    Delta_close = G.Delta;
    for k = 1:size(G.Delta,2)
        if Delta_close(v0_ind,k) == 1
            Delta_close(v0_ind,:) = -Delta_close(v0_ind,:);
        end
    end
    
    % Find rerouted UAVs at each node
    N_R = size(R,1);    % # of routes
    affected_R = zeros(N_R,1);  % affected routes
    affected_v_mat = zeros(N_R,length(G.V));    % affected nodes
    affected_e_mat = zeros(N_R,size(G.Delta,2));    % affected edges
    
    % reroute2v{v} = [start time, end time, # of UAVs rerouted to v (possibly)]
    reroute2v = cell(length(G.V),1);
    % reroute_uncertain = [edge, start time, end time, # of UAVs rerouting]
    reroute_uncertain = [];
    check_uncertain = zeros(N,4);

    
    check_reroute = cell(length(G.V),1); % Time [start, end] needs to check at v
    for v = 1:length(G.V)
        check_reroute{v} = zeros(N,2);
    end

    for r = 1:N_R
        demand_ind = (Demand_R ==r);    % UAVs through R
        route_v = R{r,5};   % nodes along the route
        % find affected routes, nodes and edges
        if ismember(v_0,route_v)    
            affected_R(r) = 1; 
            position_in_route = find(route_v == v_0,1);
            affected_v_mat(r,1:position_in_route) = route_v(1:position_in_route);
            affected_e_mat(r,1:position_in_route-1) = R{r,5}(1:position_in_route-1);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % consider cases for position_in_route = 1 or 2            
            if position_in_route == 2
                % the UAVs are drived back to origin
                continue;
            end
            % position_in_route > 2  
            for k_v = 2:position_in_route-1                
                tail = route_v(k_v-1);
                head = route_v(k_v);                
                if k_v ==2  % tail is the origin
                    check_reroute{head}(demand_ind,1) = S(demand_ind);                    
                else
                    check_reroute{head}(demand_ind,1) = occupation{tail}(demand_ind,2)+w; % inf{M}+w
                end
                check_reroute{head}(demand_ind,2) = occupation{head}(demand_ind,3); % sup{M}
            end
            % the head is v_0  
            edge = R{r,2}(position_in_route-1);                        
            if size(Backup(edge,2)) == 2 % the UAV has a fixed rerouting node B\{v_0} = tail
                % merge 2 consecutive intervals for nodes connected to v_0
                tail_1 = route_v(position_in_route-2);
                head_1 = route_v(position_in_route-1);
                check_reroute{head_1}(demand_ind,1) = occupation{tail_1}(demand_ind,2)+w;
                check_reroute{head_1}(demand_ind,2) = occupation{v_0}(demand_ind,3)-w;
            else
                % if the UAV doesn't have a fixed backup node on edge
                tail_2 = route_v(position_in_route-1);
                check_uncertain(demand_ind,1) = edge;
                check_uncertain(demand_ind,2) =  occupation{tail_2}(demand_ind,2)+w;
                check_uncertain(demand_ind,3) = occupation{v_0}(demand_ind,3)-w;              
            end            
        end
    end
    
    
    % compute reroute2v (the fixed one) both the unaffected one and the
    % ones with only one backup node left
    for v = 1:length(G.V)
        bounds = unique(check_reroute{v});
        bounds = bounds(bounds~=0);
        if isempty(bounds)==1
            continue;
        end
        reroute2v{v} = zeros(length(bounds),3);
        reroute2v{v}(1,:) = [0,bounds(1),0];
        for k = 1:length(bounds)-1
            inclusion_ind = (check_reroute{v}(:,1) <= bounds(k)) & (check_reroute{v}(:,2) > bounds(k));
            num_reroute = sum(inclusion_ind,1);
            reroute2v{v}(k+1,:) = [bounds(k),bounds(k+1),num_reroute];          
        end
    end
    
    % compute # of rerouting UAVs on each edge for reroute_uncertain
    bounds_uncertain = unique(check_uncertain(:,2:3));
    bounds_uncertain = bounds_uncertain(bounds_uncertain~=0);
    reroute_uncertain = zeros(length(bounds_uncertain),2); % [start, end]
    reroute_uncertain_e = zeros(length(bounds_uncertain),size(G.Delta,2)); % # of rerouting UAVs on each edge
    if ~isempty(bounds_uncertain)     
        reroute_uncertain(1,:) = [0,bounds_uncertain(1)];     % initialize with 1st line
        for k = 1:length(bounds_uncertain)-1
            for edge = 1:size(G.Delta,2)
                inclusion_ind = (check_uncertain(:,2) <= bounds_uncertain(k)) ...
                    & (check_uncertain(:,3) > bounds_uncertain(k)) & (check_uncertain(:,1) == edge);
                num_reroute = sum(inclusion_ind,1);
                reroute_uncertain(k+1,:) = [bounds_uncertain(k),bounds_uncertain(k+1)];
                reroute_uncertain_e(k+1,edge) = num_reroute;
            end            
        end
    end

    
    % time-varying index set (unaffected UAVs)
    % Remain = {[start, end], unaffected indices of UAVs}
    check_affected = unique(occupation{v_0}(:,2));  % sorted
    check_affected = check_affected(check_affected~=0);
    Remain = cell(length(check_affected),2);
    Remain{1,1} = [0, check_affected(1)];
    Remain{1,2} = ones(N,1);
    for k = 1:length(check_affected)-1
        checking = check_affected(k);
        affected = (occupation{v_0}(:,2) <= checking) & (occupation{v_0}(:,3) > checking);
        remaining = 1-affected;
        Remain{k+1,1} = [checking, check_affected(k+1)];
        Remain{k+1,2} = remaining;
    end    
    % Find max of unaffected UAVs at a time for each node v
    % unaffected2v{v} = [start, end, # of unaffected UAVs]
    unaffected2v = cell(length(G.V),1);
    for v = 1:length(G.V)
        bounds_M = unique(occupation{v}(:,2:3));
        bounds_M = bounds_M(bounds_M~=0);
        if isempty(bounds_M)
            continue;
        end
        bounds_all = unique([bounds_M;check_affected]);   
        unaffected2v{v} = zeros(length(bounds_all),3);
        unaffected2v{v}(1,:) = [0,bounds_all(1),0];
        for k = 1: length(bounds_all)-1
            checking = bounds_all(k);
            remain_find = find(check_affected > checking,1);
            if isempty(remain_find)
                RI = ones(N,1);
            else
                RI = Remain{remain_find,2}; % index vector of unaffected UAVs                
            end
            larger_ind = find(bounds_M>checking,1);
            inclusion_ind = (occupation{v}(:,2).*RI <= checking) & (occupation{v}(:,3).*RI > checking);
            temp_max = sum(inclusion_ind,1);
            for k2 = larger_ind:length(bounds_M)
                inclusion_ind = (occupation{v}(:,2).*RI <= bounds_M(k2)) & (occupation{v}(:,3).*RI > bounds_M(k2));
                temp_max = max(temp_max, sum(inclusion_ind,1));
            end
            unaffected2v{v}(k+1,:) = [bounds_all(k),bounds_all(k+1),temp_max];
        end
    end
    
    % check: c_v >= #rerouted2v + #unaffected2v
    % C_check{v} = [start, end, #rerouted2v, #unaffected2v, c_v >= #rerouted2v + #unaffected2v?]
    C_check = cell(length(G.V),1);
    safe_set = zeros(length(G.V),1);
    for v = 1: length(G.V)
        if isempty(unaffected2v{v}) & isempty(reroute2v{v})
            safe_set(v) = 1;
            C_check{v} = [];
            continue;
        elseif isempty(unaffected2v{v})
            bounds_t = reroute2v{v}(:,2);
        elseif isempty(reroute2v{v})
            bounds_t = unaffected2v{v}(:,2);
            safe_set(v) = 1;           
            C_check{v} = [unaffected2v{v}(:,1:2),zeros(size(unaffected2v{v},1),1),unaffected2v{v}(:,3),ones(size(unaffected2v{v},1),1)] ;
            continue;        
        else
            bounds_t = unique([unaffected2v{v}(:,2);reroute2v{v}(:,2)]);
        end
        C_check{v} = zeros(length(bounds_t),5);
        C_check{v}(1,:) = [0,bounds_t(1),0,max(unaffected2v{v}(:,3)),1];
        for k = 1:length(bounds_t)-1
            checking = bounds_t(k);
            reroute2v_ind = (reroute2v{v}(:,1) <= checking ) & (reroute2v{v}(:,2) > checking);
            if isempty(unaffected2v{v})
                unaffected2v_ind = 0;
            else
                unaffected2v_ind = (unaffected2v{v}(:,1) <= checking ) & (unaffected2v{v}(:,2) > checking); 
            end
                       
            % because of partition, both contain at most one "1"         
            if sum(reroute2v_ind)==0 & sum(unaffected2v_ind)==0
                reroute2v_end = 0;
                unaffected2v_end = 0;
            elseif sum(reroute2v_ind)==0
                reroute2v_end = 0;
                unaffected2v_end = unaffected2v{v}(unaffected2v_ind,3);
                if unaffected2v_end == 0
                    unaffected2v_end = max(unaffected2v{v}(find(unaffected2v_ind,1):end,3));
                end
            elseif sum(unaffected2v_ind)==0
                reroute2v_end =reroute2v{v}(reroute2v_ind,3);
                unaffected2v_end = 0;
            else
                reroute2v_end =reroute2v{v}(reroute2v_ind,3);
                unaffected2v_end = unaffected2v{v}(unaffected2v_ind,3);
                if unaffected2v_end == 0
                    unaffected2v_end = max(unaffected2v{v}(find(unaffected2v_ind,1):end,3));
                end
            end

            C_check{v}(k+1,1:4) = [bounds_t(k), bounds_t(k+1), reroute2v_end, unaffected2v_end];
            C_check{v}(k+1,5) = (sum(C_check{v}(k+1,3:4),2) <=G.C(v));
        end
        safe_set(v) = ~ismember(0,C_check{v}(:,5));
        Reroute = [];
    end
    % consider only the edges with head = v_0 and have more than 1
    % available backup nodes
    edge_affected = find((cell2mat(Backup(:,1))==v_0) & (cell2mat(Backup(:,3))>2));
    if ~isempty(edge_affected)
        %         % group all related nodes for each independent set of Backup sets
        %         backup_group = cell(1,2);   % each row: {[edges], [backup nodes]}
        %         count_group = 1;
        %         for k_e = 1:size(edge_affected,1)
        %             % start a new group or not
        %             edge = edge_affected(k_e);
        %             node_temp = Backup{edge,2};
        %             for k_g = 1:count_group
        %                 if (length(intersect(node_temp,backup_group{count_group,2}))>1 ) || (isempty(backup_group{count_group,1}))
        %                 % the backup node sets are intersecting, or the current
        %                 % group is empty
        %                     backup_group{count_group,1} = [backup_group{count_group,1},edge];
        %                     backup_group{count_group,2} = [backup_group{count_group,2},node_temp];
        %
        %                 end
        %             end
        %
        %             %start a new group
        %             count_group  = count_group+1;
        %         end
        
        
        % find real affecting time intervals
        % reroute_uncertain = [start, end]
        % reroute_uncertain_e =  # of rerouting UAVs on each edge
        changing_list = zeros(size(reroute_uncertain,1),1);
        changing_list(1) = 1;
        for k_t = 2: size(reroute_uncertain,1)
            unchanged = (reroute_uncertain_e(k_t,edge_affected)) == (reroute_uncertain_e(k_t-1,edge_affected));
            %if the affected edges are changing # in the new time interval
            if sum(unchanged)>0
                changing_list(k_t) = 1;
                
            end            
        end
        reroute_e2v_t = reroute_uncertain(changing_list==1,:);  % [start, end]
        reroute_e2v_t(1:end-1,2) = reroute_e2v_t(2:end,1);
        reroute_e2v_e = reroute_uncertain_e(changing_list==1,:);    %  # of rerouting UAVs on each edge
        
        
        
        L_N = sum(cell2mat(Backup(edge_affected,3))-1);   % number of nodes being rerouted to
        N_vec = zeros(L_N,2);   % record the pair [edge, node] of each rerouting relation
        start_count = 1;
        for k = 1:length(edge_affected)
            edge = edge_affected(k);
            backup_v2e = Backup{edge,2};
            backup_v2e = backup_v2e(backup_v2e~=v_0);
            N_vec(start_count:start_count+Backup{edge,3}-2,1) = edge_affected(k);
            N_vec(start_count:start_count+Backup{edge,3}-2,2) = backup_v2e;            
            start_count = start_count+Backup{edge,3}-1;
        end
        v_affected = unique(N_vec(:,2));       % nodes indefinitely be rerouted to
        % combine all related  time intervals into 1 array
        time_check = [];
        for k_v = 1:v_affected
            v_temp = v_affected(k_v);
            time_check = [time_check;C_check{v_temp}(:,1:2)];
        end
        time_check = [time_check;reroute_e2v_t];    

        % checking each time instance
        time_ins = unique(time_check);
        % C_check{v} = [start, end, #rerouted2v, #unaffected2v, c_v >= #rerouted2v + #unaffected2v?]
        % Initialize C_check_mult into {C_check{v}, e2v distribution}
        C_check_mult = cell(length(G.V),1); 
        Reroute = zeros(length(time_ins)-1,L_N);        
        for v= 1:length(G.V)
            if ismember(v,v_affected)
                C_check_mult{v} =  zeros(length(time_ins)-1,5);
                %L_N = sum(cell2mat(Backup(edge_affected,3))-1);   % number of nodes being rerouted to
            else
                C_check_mult{v} =  C_check{v};
            end           
        end
        % Initialize variables NL, NR, A1, A2
        NR = zeros(length(edge_affected),1);
        NL = zeros(length(v_affected),1);
        A1 = zeros(length(edge_affected), L_N);
        A2 = zeros(length(v_affected),L_N);        
        edge_reroute = zeros(length(time_ins)-1,length(edge_affected)); % initialize #of UAVs on each link needs to be rerouted undefinitely
        edge_undef = edge_affected;
        for k = 1:L_N
            A1(edge_affected == N_vec(k,1),k)=1; % edge constraint
            A2(v_affected == N_vec(k,2),k)=1; % node constraint
        end
        for k_t = 1:length(time_ins)-1
            t_check = time_ins(k_t);
            for k_v = 1:length(v_affected)
                v =  v_affected(k_v);
                C_check_mult{v}(k_t,1:2) = [time_ins(k_t),time_ins(k_t+1)];

                % check NL (remaining landing spots) (C_check starts from 0)
                if isempty(C_check{v})
                    ind_v = 0;
                elseif t_check>=C_check{v}(end,2)
                    ind_v = 0;
                else
                    ind_v = (C_check{v}(:,1) <= t_check ) & (C_check{v}(:,2) > t_check);    
                    C_check_mult{v}(k_t,3:5) = C_check{v}(ind_v,3:5);
                end
                if ind_v == 0
                    NL_temp = G.C(v);
                    NL(k_v) = NL_temp;
                else 
                    NL_temp = G.C(v)-sum(C_check{v}(ind_v,3:4),2);
                    NL(k_v) = NL_temp;
                end
            end
            for k_e = 1:length(edge_affected)
                edge =  edge_affected(k_e);
                % check NR (# rerouting from e) (C_check starts from 0)
                if (t_check>reroute_e2v_t(end,2)) || (t_check<reroute_e2v_t(1,1))
                    ind_e = 0;
                else
                    ind_e = (reroute_e2v_t(:,1)<= t_check ) & (reroute_e2v_t(:,2) > t_check);    
                end
                if ind_e == 0
                    NR_temp = 0;
                    NR(k_e) = NR_temp;
                else 
                    NR_temp = reroute_e2v_e(ind_e,edge);
                    NR(k_e) = NR_temp;
                end
                edge_reroute(k_t,k_e) = NR(k_e);
            end
            %compute linear program A1*N = NR, A2*N <= NL, N>=0 with
            %objective fcn f(N)=0
            LB = zeros(L_N,1);
            UB = inf(L_N,1);
            options = optimoptions('linprog','Display','none');
            N_value = linprog(zeros(L_N,1),A2,NL,A1,NR,LB,UB,options);
            if ~isempty(N_value)
                Reroute(k_t,:) = N_value';
            else
                Reroute(k_t,:) = -1*ones(1,L_N);
            end
        end
        safe_set = [];    
        
    end


end










end