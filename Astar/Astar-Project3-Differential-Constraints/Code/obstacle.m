function collision = obstacle(Node)

x = Node(1);
y = Node(2);
d = 0.2;
square1 = (x>=1.575-d && x<=2.375+d && y>=7.375-d && y<=9.375+d);
square2 = (x>=2.775-d && x<=3.575+d && y>=7.375-d && y<=9.375+d);
square3 = (x>=12.05-d && x<=13.65+d && y>=8.55-d && y<=9.65+d);
square4 = (x>=14.05-d && x<=14.85+d && y>=4.275-d && y<=6.275+d);
square5 = (x>=14.05-d && x<=14.85+d && y>=2.275-d && y<=4.275+d);
square6 = (x>=5.5525-d && x<=7.1525+d && y>=4.2-d && y<=5.8+d);
square7 = (x>=9.3-d && x<=10.9+d && y>=4.2-d && y<=5.8+d);
circle1 = (((x-6.3525)*(x-6.3525)+ (y-5.8)*(y-5.8) - (0.8+d)*(0.8+d))<0);
circle2 = (((x-6.3525)*(x-6.3525)+ (y-4.2)*(y-4.2) - (0.8+d)*(0.8+d))<0);
circle3 = (((x-10.1)*(x-10.1)+ (y-5.8)*(y-5.8) - (0.8+d)*(0.8+d))<0);
circle4 = (((x-10.1)*(x-10.1)+ (y-4.2)*(y-4.2) - (0.8+d)*(0.8+d))<0);
if (square1==1 || square2==1 || square3==1|| square4==1|| square5==1 || square6==1 || square7==1 ||...
        circle1==1 || circle2==1 || circle3==1 || circle4==1)
    collision = 1;
else
    collision = 0;
end
end