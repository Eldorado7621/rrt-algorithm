clear all
close all

seed=0

%rng(seed)

%%generate some points

nrows=1300;
ncols=1300;
obstacle=false(nrows,ncols);

[x,y]=meshgrid(1:ncols,1:nrows);

%%generate some obstacles

obstacle(y<50 | y>1250 | x<50  | x>1250)=true; %rectangular obstacle
obstacle(y<650 & y>600 & x<200)=true; %rectangular obstacle
obstacle(y<650 & y>600 & x>1000)=true; %rectangular obstacle
obstacle(y<350 & y>300 & x>1000)=true; %rectangular obstacle

obstacle(y>950 & x>650 & x<700)=true; %rectangular obstacle
obstacle(y>950 & y<1000 & x>400 & x<700)=true; %rectangular obstacle
obstacle(y>200 & y<650 & x>650 & x<700)=true; %rectangular obstacle

obstacle(y<650 & x>500 & x<550)=true; %rectangular obstacle

obstacle(y>600 & y<650 & x>500 & x<700)=true; %rectangular obstacle

obstacle(y<200 & x>350 & x<550)=true; %rectangular obstacle

figure;
imshow(~obstacle);

axis([0 ncols 0 nrows]);
axis xy; %flips the y-axis suxh that it is from bottom to top
axis on;

xlabel('x axis');
ylabel('y axis');

title('RRT created Map graph');
grid on;

%%RRT parameters
Max_connect_length=200;
Segments=100; %All these nodes must be away from obstacles
Goal_found=false; %variable to mark finding goal

%%RRT algorithm
Nodes=1; %counter to count the generated nodes- start from 1 because start_node is
map=zeros(size(obstacle));
map(obstacle)=1; % 1 in map means obstacle
Tree_connections=[]; %tree graph array representation

%locate start node and add it to the tree array
Start_node=[1200;1200];
Tree_connections=[sub2ind(size(map),Start_node(1),Start_node(2)) Inf];
map(Start_node(2),Start_node(1))=2;
hold on
plot(Start_node(2),Start_node(1),'b');
hold off

%locate goal node^
Goal_node=[200,200];
hold on
plot(Goal_node(2),Goal_node(1),'mv');
hold off

while(true)
  %%generate a node at a random location in the map
  Node_X=randi(ncols);
  Node_Y=randi(nrows);
  
  %check if not part of an obstacle
  if(map(Node_Y,Node_X)==1 || map(Node_Y,Node_X)==2)
    continue;
  end
  
  
  %%connect the new node to the closest node
  nodes_to_connect=[];
  distance=[]; %distance from neighbour
  for i=1: numel(Tree_connections(:,1))
    if(Tree_connections(i,1)==Inf)
      break
    end
    [row,col]=ind2sub(size(map),Tree_connections(i,1));
    
    %check if within range
    if(norm([Node_Y,Node_X]-[row,col])>Max_connect_length)
      continue
    end
    
    %check if obstacles is between them
    if(via_obstacle(map,Segments,[Node_Y,Node_X],[row,col]))
      continue
    end
    
    %add this node as a node to connect
    distances=[distances;[Tree_connections(i,1),norm([Node_Y,Node_X]-[row,col]
  end
  
  %%choose the closest to connect to
  if(size(distance>0))
    %Add the node to the map and set its parent as its closest node
    map(Node_Y,Node_X)=2 %2 means the node exist at that location
    Nodes=Nodes+1;
    distances_sorted=sortrows(distances,2);
    Tree_connections=[Tree_connections;sub2ind(size(map),Node_Y,Node_X) distances_sorted(1,1)];
    
    
    %plot the new added node
    hold on
    plot(Node_X,Node_Y,'r*');
    hold off;
    
    %plot edge between the new node and its parent
    [row,col]=ind2sub(size(map),distances_sorted(1,1));
    hold on
    line([Node_X,col],[Node_Y,row]);
    hold off
  else
    continue
  end
  
  %check if we are close to the goal
  if(norm([Node_Y,Node_X]-Goal_node)<Max_connect_length ...
    && ~via_obstacle(map,Segments,[Node_Y,Node_X],Goal_node));
    Nodes=Nodes+1;
    
    %add the goal node to the map
    Tree_connections(Nodes,1)=sub2ind(size(map),Goal_node(1),Goal_node(2));
    
    %set goal node parent to the new node
    Tree_connections(Nodes,2)=sub2ind(size(map),Node_Y,Node_X);
    map(Goal_node(1),Goal_node(2))=2; %2 means node exist at that location
    
    %plot edge between the new node and goal
    hold on
    line((Node_X,Goal_node(2)],[Node_Y,Goal_node(1)]);
    hold off
    
    Goal_found=true;
  end
  %%construct the path to the start_node
  if(Goal_found)
    route=[];
    route=construct_route(map,Nodes,Tree_connections);
    
    %plot the route
    for i=1:numel(route)-1
        [node1y,node1x]=ind2sub(size(map),route(i));
        [node2y,node2x]=ind2sub(size(map),route(i+1));
        hold on
        plot(node1x,node1y,'ro')
        plot(node2x,node2y,'ro')
        line([node1x,node2x],[node1y,node2y],'Color','green','LineWidth',2)
        hold off
        
     end
   end
   
   %%break if goal  found
   if(Goal_found)
      break
    end
  end
   
  
  ´
  
  
endwhile
