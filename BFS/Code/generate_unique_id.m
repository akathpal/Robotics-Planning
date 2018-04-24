function id = generate_unique_id(node)
    id = node(1,1)*10^8 + node(1,2)*10^7 + node(1,3)*10^6 +node(2,1)*10^5 + node(2,2)*10^4 + node(2,3)*10^3 + node(3,1)*10^2 + node(3,2)*10 + node(3,3);
    