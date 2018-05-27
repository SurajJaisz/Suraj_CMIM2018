function bPosition = bodyPosition(bodies,initialPosition)
%Body position vector (2) from global position vector

if bodies(1) == 0
        bPosition = [[0;0;0], initialPosition(rangeCal(bodies(2)))];
elseif bodies(2) == 0
        bPosition = [initialPosition(rangeCal(bodies(1))), [0;0;0]];
else
        bPosition = [initialPosition(rangeCal(bodies(1))), initialPosition(rangeCal(bodies(2)))];
end

end

