function [Status_r, NewNode_r] = ActionMoveRight(CurrentNode)
    NewNode_r = CurrentNode;
    if NewNode_r(1)<250
        Status_r = 1;
        NewNode_r(1)= NewNode_r(1) + 1;
    else
        Status_r = 0;
    end
    