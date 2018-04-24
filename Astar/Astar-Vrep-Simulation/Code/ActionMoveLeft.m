function [Status_l, NewNode_l] = ActionMoveLeft(CurrentNode,res)
    NewNode_l = CurrentNode;
    if NewNode_l(1)>0.2
        Status_l = 1;
        NewNode_l(1)= NewNode_l(1) - res;
    else
        Status_l = 0;  
    end
    