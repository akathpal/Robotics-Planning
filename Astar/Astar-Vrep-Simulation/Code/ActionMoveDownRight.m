function [Status_dr, NewNode_dr] = ActionMoveDownRight(CurrentNode,res)
    NewNode_dr = CurrentNode;
    if NewNode_dr(2)>0.2 && NewNode_dr(1)<14.8
        Status_dr = 1;
        NewNode_dr(2)= NewNode_dr(2) - res;
        NewNode_dr(1)= NewNode_dr(1) + res;
    else
        Status_dr = 0;
    end
    