function [Status_d, NewNode_d] = ActionMoveDown(CurrentNode,res)
    NewNode_d = CurrentNode;
    if NewNode_d(2)>0.2
        Status_d = 1;
        NewNode_d(2)= NewNode_d(2) - res;
    else
        Status_d = 0;
    end
    