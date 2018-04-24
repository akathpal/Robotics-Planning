clc;
clear all;
close all;

tic

Nodes= [];
%%NodeInfo= [ Node #, Parent node #, CostToCome]
NodesInfo=[];
id = [];
Nodes(:,:,1)= [1 2 3; 4 5 6; 7 8 0];
id(1) = generate_unique_id(Nodes(:,:,1));
NodesInfo(:,:,1)= [1,0,0];
Nodes(:,:,2)= [1 2 3; 4 5 6; 7 0 8];
id(2) = generate_unique_id(Nodes(:,:,2));
NodesInfo(:,:,2)= [2,1,0];

%%Creating a structure named NodeSet which consists of Nodes and NodesInfo 
NodeSet.Nodes = Nodes;
NodeSet.NodesInfo = NodesInfo;

CurrentNode = NodeSet.Nodes(:,:,1);

NumberofNodes= size(NodeSet.Nodes);
a = NumberofNodes(3);ParentNode = 1;

while a <=100000
    b = a+1;
    [Status_l, NewNode_l] = ActionMoveLeft(CurrentNode);
    id_l = generate_unique_id(NewNode_l);
    visited = check_duplicate(id,id_l);
    if(Status_l==1 && ~visited)
        NodeSet.Nodes(:,:,b) = NewNode_l;
        NodeSet.NodesInfo(:,:,b) = [b,ParentNode,0];
        id(b) = generate_unique_id(NewNode_l);
        b = b+1;
    end
    
    [Status_r, NewNode_r] = ActionMoveRight(CurrentNode);
    id_r = generate_unique_id(NewNode_r);
    visited = check_duplicate(id,id_r);
    if(Status_r==1 && ~visited)
        NodeSet.Nodes(:,:,b) = NewNode_r;
        NodeSet.NodesInfo(:,:,b) = [b,ParentNode,0];
        id(b) = generate_unique_id(NewNode_r);
        b = b+1;
    end
    
    [Status_u, NewNode_u] = ActionMoveUp(CurrentNode);
    id_u = generate_unique_id(NewNode_u);
    visited = check_duplicate(id,id_u);
    if(Status_u==1 && ~visited)
        NodeSet.Nodes(:,:,b) = NewNode_u;
        NodeSet.NodesInfo(:,:,b) = [b,ParentNode,0];
        id(b) = generate_unique_id(NewNode_u);
        b = b+1;
    end
    
    [Status_d, NewNode_d] = ActionMoveDown(CurrentNode);
    id_d = generate_unique_id(NewNode_d);
    visited = check_duplicate(id,id_d);
    if(Status_d==1 && ~visited)
        NodeSet.Nodes(:,:,b) = NewNode_d;
        NodeSet.NodesInfo(:,:,b) = [b,ParentNode,0];
        id(b) = generate_unique_id(NewNode_d);
        b = b+1;
    end
    
    
    ParentNode = ParentNode + 1;
    CurrentNode = NodeSet.Nodes(:,:,ParentNode);
    NumberofNodes= size(NodeSet.Nodes);
    a = NumberofNodes(3);
end
NodesInfo = NodeSet.NodesInfo;
Nodes = NodeSet.Nodes;
toc