function collision = obstacle(Space,Node)

%% Method 1: Just Checing Pixel Value at location
    x = Node(1) + 1;
    y = 151 - Node(2);
    if (Space(y,x) == 1)
        collision = 0;
    else
        collision = 1;
    end


%% Method2: Using Half Plane Concept
%     x = Node(1);
%     y = Node(2);
%     
%     %if node is within the region output will be true
%     circle = (((x-180)*(x-180)+ (y-120)*(y-120) - 15*15)<0);
%     square = (x>=55 && x<=105 && y>=69 && y<=114);
%     
%     function line = get_line(x1,y1,x2,y2)
%         line = (y-y1) - ((y2-y1)/(x2-x1))*(x-x1);
%     end
%     
%     %Dividing POlygon into 2 convex shapes and finding half planes for those
%     %shapes
%     l1 = get_line(145,14,120,55);
%     l2 = y-14;
%     l3 = get_line(158,51,120,55);
%     l4 = get_line(158,51,168,14);
%     l5 = get_line(165,89,158,51);
%     l6 = get_line(165,89,188,51);
%     l7 = get_line(188,51,168,14);
%     region1 = l1>0 && l2>0 && l3<0 && l4<=0; 
%     region2 = l4>=0 && l7>0 && l5<0 && l6<0;
%     polygon = region1 || region2;
%     
%     if (circle==1 || square==1 || polygon==1)
%         collision = 1;
%     else
%         collision = 0;
%     end
%     
end