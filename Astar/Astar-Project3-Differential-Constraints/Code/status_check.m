function Status = status_check(NewNode)

 if NewNode(1)>0 && NewNode(2)>0 && NewNode(1)<15 && NewNode(2)<10
    Status = 1;
else
    Status = 0;  
 end

end
