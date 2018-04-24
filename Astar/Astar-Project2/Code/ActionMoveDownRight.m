function [Status_dr, NewNode_dr] = ActionMoveDownRight(CurrentNode)
    NewNode_dr = CurrentNode;
    if NewNode_dr(2)>0 && NewNode_dr(1)<250
        Status_dr = 1;
        NewNode_dr(2)= NewNode_dr(2) - 1;
        NewNode_dr(1)= NewNode_dr(1) + 1;
    else
        Status_dr = 0;
    end
    