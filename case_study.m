%% Network parameters
% Graph
G.V = [1;2;3;4;5;6;7];    % vertices
%G.C = [100;5;4;2;4;3;5];    %capacity of vertices
%G.C = [100;50;40;20;40;30;50];
G.C = [100;8;6;4;5;3;5];       %1
version = 1;
% G.C = [100;8;6;8;5;3;5];       %2
% version = 2;

G.ori = [1];
G.dest = [6,7];

G.Delta = [-1,0,0,0,0,0,0;...
           1,-1,-1,0,0,0,0;...
           0,1,0,-1,-1,0,0;...
           0,0,1,0,0,-1,0;...
           0,0,0,0,1,1,-1;...
           0,0,0,1,0,0,0;...
           0,0,0,0,0,0,1];      % Incidence matrix

X_l = [8;3;2;4;5;1;3];
X_u = [10;5;6;8;6;5;6];

G.W_l = diag(X_l);          % Edge weights: upperbound
G.W_u = diag(X_u);          % Edge weights: lowerbound


% Routes
R = cell(3,4);

R{1,1} = 1;             % path number
R{1,2} = [1,2,4];       % edges of the path 
R{1,3} = zeros(1,length(R{1,2}));       % aggregated uncertainty at each dest
R{1,4} = zeros(2,length(R{1,2}));       % aggregated travel time at each dest

R{2,1} = 2;         
R{2,2} = [1,2,5,7];
R{2,3} = zeros(1,length(R{2,2}));
R{2,4} = zeros(2,length(R{2,2}));

R{3,1} = 3;
R{3,2} = [1,3,6,7];
R{3,3} = zeros(1,length(R{3,2}));
R{3,4} = zeros(2,length(R{3,2}));

% for each routes, the nodes along the route
R2V = cell(size(R,1),2);
for k_R = 1:size(R,1)
    R2V{k_R,1} = R{k_R,1};    
    route = R{k_R,2};
    R2V{k_R,2} = zeros(1,length(route)+1);
    R2V{k_R,2}(1) = find(G.Delta(:,route(1))<0,1);
    for k = 1:length(route)
         R2V{k_R,2}(k+1) = find(G.Delta(:,route(k))>0,1);
    end
end

R(:,5) = R2V(:,2);

V2R = cell(length(G.V),2);
for v = 1:length(G.V)
    V2R{v,1} = v;
    V2R{v,2} = zeros(1,size(R,1));
    for k_R = 1:size(R,1)
        if sum(ismember(R2V{k_R,2},v))
             V2R{v,2}(k_R) = R{k_R,1};
        end
    end
    V2R{v,2} = V2R{v,2}(V2R{v,2}~=0);
end

% waiting time
w = 1;

% find all converging nodes and corresponding routes
% [the node that has route passing through, corresponding route] = time
% needs to reserve 
for k_R = 1:size(R,1)
    route = R{k_R,2};
    a_agr = 0;
    b_agr = 0;
    for k_2 = 1:length(route)
        if k_2 == 1
            a_agr = a_agr + G.W_l(route(k_2),route(k_2));
            b_agr = b_agr + G.W_u(route(k_2),route(k_2));
        else
            a_agr = a_agr + G.W_l(route(k_2),route(k_2))+w;
            b_agr = b_agr + G.W_u(route(k_2),route(k_2))+w;
        end
        R{k_R,3}(k_2) = b_agr - a_agr;
        R{k_R,4}(1,k_2) =a_agr;     % earliest arrival of k_2'th dest(node)
        R{k_R,4}(2,k_2) =b_agr;     % latest arrival of k_2'th dest(node)     
    end
end

% assign backup nodes to edges: edge's row = [head, backup nodes,  size of backup nodes]
Backup = cell(length(X_l),3);
Backup{1,1} = 2; % head node
Backup{1,2} = [1,2];
Backup{2,1} = 3; % head node
Backup{2,2} = [2,3,4];
Backup{3,1} = 4; % head node
Backup{3,2} = [2,3,4];
Backup{4,1} = 6; % head node
Backup{4,2} = [3,5,6];
Backup{5,1} = 5; % head node
Backup{5,2} = [3,4,5];
Backup{6,1} = 5; % head node
Backup{6,2} = [3,4,5,7];
Backup{7,1} = 7; % head node
Backup{7,2} = [4,5,7];

for k = 1:length(X_l)
    Backup{k,3} = length(Backup{k,2});
end
%% Case study parameters
v_0 = 5;    % closing node
N = 100;     % number of demands
max_dpT = 500;   % maximum of departure time (when generating)

% S = zeros(N,1);     % schedules 
% Demand_R = zeros(N,1);  % routes for demands
% S = round(rand(N,1)*max_dpT,1);
% Demand_R = randi(size(R,1),N,1);   % choose routes randomly (uniform distr)

% save('sche_ex2.mat','S');
% save('demand_ex2.mat','Demand_R');

% 100-UAV example
% load('sche_ex1.mat','S');
% load('demand_ex1.mat','Demand_R');

% 20-UAV example
load('sche_ex0.mat','S');
load('demand_ex0.mat','Demand_R');

%% Computation/Verification (1 backup node)
% Verify that S satisfies the safety constraint when all nodes are
% available
% [safe_set, C_check] = verification_1_node(G,R,Demand_R,S,[],w);
 
% Verify that S satisfies the safety constraint when v_0 is disabled
% [safe_set, C_check] = verification_1_node(G,R,Demand_R,S,v_0,w);




%% Computation/Verification (multiple backup node)
% Verify that S satisfies the safety constraint when all nodes are
% available
% [safe_set, C_check] = verification_1_node(G,R,Demand_R,S,[],w);
 
% Verify that S satisfies the safety constraint when v_0 is disabled
tStart = tic; 
[safe_set, C_check,Reroute,N_vec,edge_reroute,edge_undef] = verification_backup(G,R,Demand_R,S,v_0,w,Backup);
tEnd = toc(tStart);


% % plot the figure
% color_unaffect = '#75bbfd';  % blue
% 
% color_reroute{1} = '#ffcfdc'; % pink
% color_reroute{2} = '#fcb001'; % orange (e5->v3)
% color_reroute{3} = '#89a203'; % green   (e6->v3)
% color_reroute{4} = '#9c6d57'; % brown
% 
% color_unable = '#7d7f7c'; %grey
% 
% 
% color_e2v{1,1}= '#fcb001'; % orange (e5->v3)
% color_e2v{2,1} = '#966ebd'; % purple (e5->v4)
% color_e2v{1,2} = '#89a203'; % green   (e6->v3)
% color_e2v{2,2} = '#fd4659'; % red (e6->v4)
% color_e2v{3,2} = '#6258c4'; % deep blue (e6->v7)
% 
% 
% % plot UAVs going to v
% %Reroute here represents [N(e5,v2), N(e5,v3),N(e6,v2),N(e6,v3),N(e6,v6)] (# or rerouting)
% % N_vec = [e,v] (affected e,v pairs)
% v_affected = unique(N_vec(:,2));
% v_check = 3;
% 
% if ismember(v_check,v_affected)
%     size_bound = size(C_check{v_check},1);
%     x = C_check{v_check}(:,1);  % x_start of bar
%     dx = C_check{v_check}(:,2)-C_check{v_check}(:,1);     % width of bar
%     
%     y1 = zeros(size_bound,1);  % y_start of unaffected bar
%     dy1 = C_check{v_check}(:,4);  % height of unaffected bar
%     y2 = dy1;  % y_start of rerouted bar (always 0)
%     dy2 = C_check{v_check}(:,3);  % height of rerouted bar
%     ind_edge = find(N_vec(:,2)==v_check);  %find all edges possible to rerouted to v_checked (undeterminedly)
%     
%     % initialize the indefinite reroutes
%     y_r = zeros(size_bound,length(ind_edge)+1);
%     dy_r = zeros(size_bound,length(ind_edge)+1);
%     y_r(:,1) = y2;
%     dy_r(:,1) = dy2;
%     for k = 1:length(ind_edge)
%         y_r(:,k+1) = y_r(:,k)+max(dy_r(:,k),0);
%         dy_r(:,k+1) = Reroute(:,ind_edge(k));
%     end
% 
%     % plot the v_check
%     figure
%     set(gcf,'Units','Inches')
%     set(gcf,'Position',[4 4 4 3])
%     set(gca,'units','inches')
%     set(gcf, 'PaperUnits','inches');
%     set(gcf, 'PaperSize', [4 3]);
%     set(gcf, 'PaperPositionMode', 'manual');
%     set(gcf, 'PaperPosition', [0 0 4 3]);
%     hold on
%     for ii=1:length(x)
%         R_unaffected = rectangle('position',[x(ii) y1(ii) dx(ii) dy1(ii)],'LineStyle','none'); % N_v
%         R_unaffected.FaceColor =  color_unaffect; %blue
%         hold on
%         if Reroute(ii,1)>=0    %if the nondeterministic rerouting UAV can be computed
%             for k = 1:length(ind_edge)+1
%                 Rec = rectangle('position',[x(ii) y_r(ii,k) dx(ii) max(dy_r(ii,k),0)],'LineStyle','none');
%                 Rec.FaceColor = color_reroute{k};
%                 %             Rec.EdgeColor = edgeList{1};
%                 hold on
%             end
%         else    %if the nondeterministic rerouting UAV not possible to be arranged
%             Rec = rectangle('position',[x(ii) dy1(ii) dx(ii) G.C(v_check)-dy1(ii) ],'LineStyle','none');   % make it up to the capacity
%             Rec.FaceColor = color_unable;  % grey bar
%         end
%     end
%     plot([0 max(C_check{v_check}(:,2))+1],[G.C(v_check),G.C(v_check)],'b-.','Linewidth',2)
%     axis([0 max(C_check{v_check}(:,2))+1 0 max([max(sum(C_check{v_check}(:,3:4),2)),G.C(v_check)+1])+3])
%     hold on
%     
%     l_gap = 2.5;
%     legend_x1 = 2;
%     legend_dx1 = 12;
%     legend_y1 = 9;
%     legend_dy1 = 0.5;
%     
%     text(legend_x1+1,8,'N_R(v_3)','Color','black','FontSize',12)
%     rectangle('position',[legend_x1 legend_y1 legend_dx1 legend_dy1],'LineStyle','none','FaceColor',color_unaffect)
% 
%     legend_x2 = l_gap+legend_x1+legend_dx1;
%     legend_dx2 = legend_dx1;
%     text(legend_x2+1,8,'N_{e_2,v_3}','Color','black','FontSize',12)
%     rectangle('position',[legend_x2 legend_y1 legend_dx2 legend_dy1],'LineStyle','none','FaceColor',color_reroute{1})
% 
%     legend_x3 = l_gap+legend_x2+legend_dx2;
%     legend_dx3 = legend_dx1;
%     text(legend_x3+1,8,'N_{e_5,v_3}','Color','black','FontSize',12)
%     rectangle('position',[legend_x3 legend_y1 legend_dx3 legend_dy1],'LineStyle','none','FaceColor',color_reroute{2})
% 
%     legend_x4 = l_gap+legend_x3+legend_dx3;
%     legend_dx4 = legend_dx1;
%     text(legend_x4+1,8,'N_{e_6,v_3}','Color','black','FontSize',12)
%     rectangle('position',[legend_x4 legend_y1 legend_dx4 legend_dy1],'LineStyle','none','FaceColor',color_reroute{3})
% 
%     legend_x5 = l_gap+legend_x4+legend_dx4;
%     legend_dx5 = legend_dx1;
%     text(legend_x5+1,8,'Failure','Color','black','FontSize',12)
%     rectangle('position',[legend_x5 legend_y1 legend_dx5 legend_dy1],'LineStyle','none','FaceColor',color_unable)
% 
%     text(65,G.C(3)-1,'C_{v_3}','Color','black','FontSize',12)
% %    plot([legend_x5 legend_x5+legend_dx5],[legend_y1,legend_y1],'b-.','Linewidth',2)
% 
%     hold off
% 
%     y_str = ['Number of UAVs at v_',num2str(v_check)];
%     ylabel(y_str,'FontSize',12)
%     xlabel('t_c (minute)','FontSize',12)
% 
%     if version == 1
%         name_fig = ['case_v',num2str(v_check),'_v1'];
%     elseif version == 2
%         name_fig = ['case_v',num2str(v_check),'_v2'];
%     end
%     
%     saveas(gcf,name_fig,'fig')
% 
% 
% 
% 
% 
%     % plot e (dividing e into parts rerouted to all possible v's)
%     % notice that v's that may be rerouted by e will have the same C_check
%     % dimension
%     for k_e = 1:length(ind_edge)
%         edge_check = N_vec(ind_edge(k_e),1);
%         ind_node = find(N_vec(:,1)==edge_check);  % indices of all (e,v) pair where v will be rerouted from e in 
% 
%         % initialize the indefinite reroutes
%         y_v = zeros(size_bound,length(ind_node));
%         dy_v = zeros(size_bound,length(ind_node));
%         for k_v = 1:length(ind_node)            
%             dy_v(:,k_v) = Reroute(:,ind_node(k_v));
%             if k_v > 1
%                 y_v(:,k_v) = y_v(:,k_v-1)+max(0,dy_v(:,k_v-1));   % starting of next node's bar
%             end
%         end
%         % the total # of UAVs on edge_check needs to be rerouted 
%         ind_new = find(edge_check==edge_undef);
%         y_t = zeros(size_bound,length(ind_node));
%         dy_t = edge_reroute(:,ind_new); 
% 
%         figure
%         set(gcf,'Units','Inches')
%         set(gcf,'Position',[4 4 4 3])
%         set(gca,'units','inches')
%         set(gcf, 'PaperUnits','inches');
%         set(gcf, 'PaperSize', [4 3]);
%         set(gcf, 'PaperPositionMode', 'manual');
%         set(gcf, 'PaperPosition', [0 0 4 3]);
%         hold on
%         for ii=1:length(x)
%             R_total = rectangle('position',[x(ii) y_t(ii) dx(ii) max(0,dy_t(ii))],'LineStyle','none'); % N_total
%             R_total.FaceColor = color_unable;
%             for k_v = 1:length(ind_node)
%                 if dy_v(ii,k_v)>0                
%                     R_unaffected = rectangle('position',[x(ii) y_v(ii,k_v) dx(ii) max(0,dy_v(ii,k_v))],'LineStyle','none'); % N_v
%                     R_unaffected.FaceColor = color_e2v{k_v,k_e};
%                     hold on                    
%                 end
%             end            
%         end
%         
%         axis([0 max(C_check{v_check}(:,2))+1 0 max(dy_t)+1.5]) 
% 
%         l_gap = 2.5;
%         h_gap = 0.3;
%         legend_x = 15;
%         legend_dx = 12;
%         legend_y1 = max(dy_t)+1;
%         legend_dy1 = 0.2;
% 
%         for k_v = 1:length(ind_node)     
%             node_check = N_vec(ind_node(k_v),2);
%             %str_temp = sprintf('I have %d dogs', x);
%             str_temp = ['N_{e_',num2str(edge_check),',v_',num2str(node_check),'}'];
%             text(legend_x+1,legend_y1-h_gap,str_temp,'Color','black','FontSize',12)
%             rectangle('position',[legend_x legend_y1 legend_dx legend_dy1],'LineStyle','none','FaceColor',color_e2v{k_v,k_e})
%             legend_x = l_gap+legend_x+legend_dx;
%         end
% 
%         text(legend_x+1,legend_y1-h_gap,'Failure','Color','black','FontSize',12)
%         rectangle('position',[legend_x legend_y1 legend_dx legend_dy1],'LineStyle','none','FaceColor',color_unable)
% 
% 
% %         text(legend_x1+1,8,'N_R(v_3)','Color','black','FontSize',12)
% %         rectangle('position',[legend_x1 legend_y1 legend_dx1 legend_dy1],'LineStyle','none','FaceColor',color_unaffect)
% % 
% %         legend_x2 = l_gap+legend_x1+legend_dx1;
% %         legend_dx2 = legend_dx1;
% %         text(legend_x2+1,8,'N_{e_2,v_3}','Color','black','FontSize',12)
% %         rectangle('position',[legend_x2 legend_y1 legend_dx2 legend_dy1],'LineStyle','none','FaceColor',color_reroute{1})
% % 
% %         legend_x3 = l_gap+legend_x2+legend_dx2;
% %         legend_dx3 = legend_dx1;
% %         text(legend_x3+1,8,'N_{e_5,v_3}','Color','black','FontSize',12)
% %         rectangle('position',[legend_x3 legend_y1 legend_dx3 legend_dy1],'LineStyle','none','FaceColor',color_reroute{2})
% % 
% %         legend_x4 = l_gap+legend_x3+legend_dx3;
% %         legend_dx4 = legend_dx1;
% %         text(legend_x4+1,8,'N_{e_6,v_3}','Color','black','FontSize',12)
% %         rectangle('position',[legend_x4 legend_y1 legend_dx4 legend_dy1],'LineStyle','none','FaceColor',color_reroute{3})
% % 
% 
% 
%         y_str = ['Number of UAVs at e_',num2str(edge_check)];
%         ylabel(y_str)
%         xlabel('t_c (minute)')
% 
% 
% 
%         if version == 1
%             name_fig = ['case_e',num2str(edge_check),'_v1'];
%         elseif version == 2
%             name_fig = ['case_e',num2str(edge_check),'_v2'];
%         end        
%         saveas(gcf,name_fig,'fig')
%     end
%     % endplot e (dividing e into parts rerouted to all possible v's)
%     
% end     %end plotting v_check
