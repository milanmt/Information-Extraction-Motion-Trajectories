clc;
clear all;

% Creating Directed Acyclic Graph
N = 5;
dag = zeros(5,5);
where_t = 1;
where_t1 = 2;
who = 3;
what_t = 4;
what_t1 = 5;
dag(where_t, [where_t1 who what_t]) = 1;
dag(where_t1, [who what_t1]) = 1;
dag(who, [what_t what_t1]) = 1;
discrete_nodes = 1:N;
node_sizes = [13 13 8 12 12 ];
% node_sizes = [ 8 13 13 12 12 ];

% Making Bayes Net 
onodes = [2];
bnet = mk_bnet(dag, node_sizes, 'discrete', discrete_nodes, 'observed', onodes,...
    'names', {'who', 'where_t', 'where_t1', 'what_t', 'what_t1'} );

% Forming Initial Probabilities
% AT A BUSY TIME DURING WORKING HOURS
% P(who) = Lecturers | PhD | Masters | Undergraduates | Technicians | Staff
% | Visitors | Janitor
p_who = [ 0.1 0.15 0.25 0.3 0.06 0.04 0.05 0.05 ];
% P(where) = Entries | Teaching Lab - 2| Hallway 1 - 3 | Lift - 6 | Hallway 2 - Hallway | Robot
% Lab | Vending Machines | Common Area - 10 | LG21 - 11| LG04 - 17| Hallway 3 - 14 | PhD
% Office - 4 | Nowhere on LGF
p_where = [0.045 0.1 0.065 0.04 0.065 0.1 0.045 0.25 0.015 0.15 0.065 0.05 0.01];
% P(what) Studying/Working on Laptops Individually | Eating | Group Discussions | Robot Work | Attending Lab| Chatting | Cleaning
% |Robot Evaluation | Demonstrating Lab | Walking/Waiting for someone | Other Activity | Getting Snack/Drink
p_what = [0.2 0.03 0.1 0.1 0.2 0.1 0.03 0.01 0.1 0.1 0.01 0.02];
% P(wheret1|wheret) = wheret | wheret+1
p_wheret1_wheret = [0 0.5225 0.375 0 0 0.0625 0 0 0 0 0 0 0.04;
                    0.625 0 0.375 0 0 0 0 0 0 0 0 0 0;
                    0.2012 0.0592 0  0.0355 0.2189 0.3195 0.0473 0 0 0 0 0.1184 0;
                    0 0 1 0 0 0 0 0 0 0 0 0 0;
                    0 0 0.0518 0 0 0 0.0817 0.7193 0.1091 0 0.0381 0 0; 
                    0.1764 0 0.6176 0 0 0 0 0 0 0 0.206 0 0;
                    0 0 0.2051 0 0.6667 0 0 0 0 0 0.1282 0 0;
                    0 0 0 0 0 0 0 1 0 0 0 0 0;
                    0 0 0 0 0 0 0 1 0 0 0 0 0;
                    0 0 0 0 0 0 0 0 0 0 1 0 0;
                    0.1714 0 0 0 0.2571 0.1428 0.0858 0 0 0.3429 0 0 0;
                    0.33 0 0.67 0 0 0 0 0 0 0 0 0 0;
                    0.9412 0 0 0.0588 0 0 0 0 0 0 0 0 0];                    
p_wheret1_wheret_b = reshape(p_wheret1_wheret, [169,1]);                   

% P(where|what)- what| where
p_where_what = [0 0.1 0 0 0 0.04 0 0.4 0 0.1 0 0.35 0.01;
                0.01 0.01 0.01 0.01 0.01 0.01 0.01 0.88 0.01 0.01 0.01 0.01 0.01;
                0 0.1 0 0 0 0.09 0 0.4 0 0.1 0 0.3 0.01;
                0.1 0.238 0.1 0 0.1 0.35 0 0.01  0 0.001 0.1 0.001 0;
                0 0.45 0 0 0 0 0 0 0 0.45 0 0 0.1;
                0.01 0.0001 0.1 0.01 0.1 0.0001 0.01 0.4 0.0001 0.0001 0.1 0.0001 0.2695;
                0.2 0.05 0.2 0 0.1 0 0 0.2 0 0.05 0.2 0 0;
                0 0.2 0.2 0 0.2 0 0 0.2 0 0 0.2 0 0;
                0 0.45 0 0 0 0 0 0 0 0.45 0 0 0.1;
                0.1893 0.0001 0.2 0.0001 0.2 0.0001 0.0001 0.01 0.0001 0.0001 0.2 0.0001 0.2;
                0.07 0.07 0.07 0.07 0.07 0.07 0.07 0.07 0.16 0.07 0.07 0.07 0.07; 
                0 0 0 0 0 0 0.9 0 0 0 0 0 0.1]; 
% P(what|where) - where|what 
p_what_where = p_where_what.*p_what';
p_what_where = (p_what_where./sum(p_what_where,1))';
             
% P(who|what) -what| who
p_who_what = [0.001 0.332981 0.332983 0.332983 0.00001 0.00001 0.00001 0.00001;
              0.125 0.125 0.125 0.125 0.125 0.125 0.125 0.125;
              0.001 0.01 0.494488 0.49448 0.00001 0.00001 0.00001 0.00001;
              0.001 0.33296 0.333 0.333 0.00001 0.00001 0.00001 0.00001;
              0.00001 0.00001 0.49997 0.49997 0.00001 0.00001 .00001 0.00001;
              0.01 0.1 0.33 0.33 0.01 0.01 0.2 0.01;
              0.00001 0.00001 0.00001 0.00001 0.00001 0.00001 0.00001 0.99993;
              0.49997 0.49997 0.00001 0.00001 0.00001 0.00001 0.00001 0.00001; 
              0.49997 0.49997 0.00001 0.00001 0.00001 0.00001 0.00001 0.00001;
              0.125 0.125 0.125 0.125 0.125 0.125 0.125 0.125;
              0.001 0.001 0.001 0.001 0.33137 0.3319 0.3317 0.001;
              0.125 0.125 0.125 0.125 0.125 0.125 0.125 0.125]; 
          
% P(what|who) - who|what
p_what_who = p_who_what.*p_what';
p_what_who = (p_what_who./sum(p_what_who,1))';

% P(who|where) - where| who
p_who_where = [0.125 0.125 0.125 0.125 0.125 0.125 0.125 0.125;
               0.05 0.1 0.0001 0.8495 0.0001 0.0001 0.0001 0.0001;
               0.125 0.125 0.125 0.125 0.125 0.125 0.125 0.125;
               0.125 0.125 0.125 0.125 0.125 0.125 0.125 0.125;
               0.125 0.125 0.125 0.125 0.125 0.125 0.125 0.125;
               0.05 0.8 0.1495 0.0001 0.0001 0.0001 0.0001 0.0001;
               0.125 0.125 0.125 0.125 0.125 0.125 0.125 0.125;
               0.02 0.05 0.3595 0.57 0.0001 0.0001 0.0002 0.0001;
               0.01 0.001 0.000001 0.000001 0.988995 0.000001 0.000001 0.000001; 
               0.05 0.1 0.4243 0.4244 0.0001 0.0001 0.0001 0.001;
               0.125 0.125 0.125 0.125 0.125 0.125 0.125 0.125;
               0.1 0.8994 0.0001 0.0001 0.0001 0.0001 0.0001 0.0001;
               0.125 0.125 0.125 0.125 0.125 0.125 0.125 0.125];

           
% P(who|wheret,wheret+1)
p_who_wherett1 = [];
for j  = 1:node_sizes(2)
    p_who_wherett1 = [p_who_wherett1 ; p_who_where.*p_who_where(j,:)];
end
nc = sum(p_who_wherett1,2);
p_who_wherett1 = p_who_wherett1./nc;
p_who_wherett1 = reshape(p_who_wherett1, [1352,1]);

% % P(who|wheret,wheret+1) - type2
% p_wheret_t1 = [];
% for j  = 1:node_sizes(2)
%     p_wheret_t1 = [p_wheret_t1 p_where_who.*p_where_who(:,j)];
% end
% pwtt1 = reshape(p_wheret1_wheret, [size(p_wheret_t1,2),1]);
% p_wherett1 = p_wheret_t1'.*pwtt1;
% p_who_giv_wherett1 = p_wherett1'.*p_who';
% nc = p_wherett1*p_who';
% for m = 1:size(p_wheret_t1, 2)
%     p_who_giv_wherett1(:,m) = p_who_giv_wherett1(:,m)/nc(m);
% end
% p_who_giv_wherett1_2 = p_who_giv_wherett1';
% p_who_giv_wherett1_2 = reshape(p_who_giv_wherett1_2, [1352,1]);
% % For finding the number of valid transitions
% % s = sum(p_who_giv_wherett1_2, 2);
% % count = 0;
% % for z = 1:size(s,1)
% %     if (s(z)-1) <= 0.00000000000001
% %         count = count + 1;
% %     end
% % end
% % count

% P(what_t|where_t,who) - who, where_t | what_t 
p_what_wherewho =[];
for i = 1:node_sizes(3)
    p_what_wherewho = [p_what_wherewho ; p_what_where.*p_what_who(i,:)];
end
p_what_wherewho = p_what_wherewho./sum(p_what_wherewho,2);
p_what_wherewho = reshape(p_what_wherewho, [1248,1]);

% %P(what_t|where_t,who) - type2
% p_whowhere_what = [];
% for i = 1:node_sizes(3)
%     p_whowhere_what = [p_whowhere_what p_where_what.*p_who_what(i)];
% end
% p_wherewho = reshape(p_who_where, [size(p_whowhere_what,2),1]);
% p_wherewho_what = p_whowhere_what'.*p_wherewho;
% p_what_wherewho = p_wherewho_what'.*p_what';
% nc = p_wherewho_what*p_what';
% for j = 1:size(p_wherewho_what,1)
%     p_what_wherewho(:,j) = p_what_wherewho(:,j)/nc(j);
% end
% p_what_wherewhot_2 = p_what_wherewho';
% p_what_wherewhot_2 = reshape(p_what_wherewhot_2, [1248,1]);

% Setting Conditional Probabilities 
bnet.CPD{where_t} = tabular_CPD(bnet, where_t, p_where');
bnet.CPD{where_t1} = tabular_CPD(bnet, where_t1, p_wheret1_wheret_b);
bnet.CPD{who} = tabular_CPD(bnet, who, p_who_wherett1);
bnet.CPD{what_t} = tabular_CPD(bnet, what_t, p_what_wherewho);
bnet.CPD{what_t1} = tabular_CPD(bnet, what_t1, p_what_wherewho);

% Inference
engine = jtree_inf_engine(bnet);
evidence = cell(1,N);
evidence{1} = 9;
[engine, loglik] = enter_evidence(engine, evidence);
marg = marginal_nodes(engine, who);
marg.T
marg = marginal_nodes(engine, where_t1);
marg.T
marg = marginal_nodes(engine, what_t);
marg.T
marg = marginal_nodes(engine, what_t1);
marg.T
