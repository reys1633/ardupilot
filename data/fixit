d = csvread('rrt_01172019_1509.csv');
[r,c] = size(d);
for i = 1 : r
    temp = d(r,13:14);

    for j = 12 : -1 : 1 % move all over 2
        d(i, j+3) = d(i, j+1);
        d(i, j+2) = d(i, j);
    end
    d(i,1:2) = temp;

    for j = 1 : 14 % move all over 2
        fprintf("%9.1f", d(i,j) );
    end
    fprintf("\n");

end