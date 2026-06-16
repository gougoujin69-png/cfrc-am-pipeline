function hc = pos2hc(a)
for i = 1:3
    c(i) = cos(a(i+3));
    s(i) = sin(a(i+3));
end
hc(1,1) = c(2)*c(3);
hc(1,2) = -c(2)*s(3);
hc(1,3) = s(2);
hc(2,1) = c(1)*s(3) + c(3)*s(1)*s(2);
hc(2,2) = c(1)*c(3) - s(1)*s(2)*s(3);
hc(2,3) = -c(2)*s(1);
hc(3,1) = s(1)*s(3) - c(1)*c(3)*s(2);
hc(3,2) = c(3)*s(1) + c(1)*s(2)*s(3);
hc(3,3) = c(1)*c(2);
for i = 1:3
    hc(i,4) = a(i);
    hc(4,i) = 0;
end
hc(4,4) = 1;
