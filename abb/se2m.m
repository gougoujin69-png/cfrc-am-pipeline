function M = se2m(T)
for i =1:3
    M(i,1) = T.n(i);
    M(i,2) = T.o(i);
    M(i,3) = T.a(i);
    M(i,4) = T.t(i);
    M(4,i) = 0;
end
M(4,4) = 1;
