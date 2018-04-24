function [Status_dl, NewNode_dl] = ActionMoveDownLeft(CurrentNode)
    NewNode_dl = CurrentNode;
    if NewNode_dl(2)>0 && NewNode_dl(1)>0
        Status_dl = 1;
        NewNode_dl(2)= NewNode_dl(2) - 1;
        NewNode_dl(1)= NewNode_dl(1) - 1;
    else
        Status_dl = 0;
    end
    