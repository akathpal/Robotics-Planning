function [Status_d, NewNode_d] = ActionMoveDown(CurrentNode)
    NewNode_d = CurrentNode;
    if NewNode_d(2)>0
        Status_d = 1;
        NewNode_d(2)= NewNode_d(2) - 1;
    else
        Status_d = 0;
    end
    