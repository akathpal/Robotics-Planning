function [Status_dl, NewNode_dl] = ActionMoveDownLeft(CurrentNode,res)
    NewNode_dl = CurrentNode;
    if NewNode_dl(2)>0.2 && NewNode_dl(1)>0.2
        Status_dl = 1;
        NewNode_dl(2)= NewNode_dl(2) - res;
        NewNode_dl(1)= NewNode_dl(1) - res;
    else
        Status_dl = 0;
    end
    