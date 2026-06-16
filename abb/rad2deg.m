%将pi相关角度表示为度数相关
function d = rad2deg(r)
for i = 1:6
    d(i) = r(i)*180/pi;
end