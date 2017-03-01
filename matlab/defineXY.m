function [X, Y] = defineXY(Xc, Yc, MeanA, MeanB)
min = 10000;
RA = MeanA(1:3,1:3);
tA = MeanA(1:3,4);
RB = MeanB(1:3,1:3);
tB = MeanB(1:3,4);

for i = 1:1:size(Xc,3)
    for j = 1:1:size(Yc,3)
        
        RX = Xc(1:3,1:3,i);
        tX = Xc(1:3,4,i);
        RY = Yc(1:3,1:3,j);
        tY = Yc(1:3,4,j);
%         
%         temp = det(RA*RX)
               %temp =  norm(logm(MeanA(1:3,1:3) * Xc(1:3,1:3,i)) - logm(Yc(1:3,1:3,j) * MeanB(1:3,1:3)));
               temp = norm(RA*RX - RY*RB) + norm(RA*tX+tA-RY*tB-tY);
       if(temp < min)
           id_x = i;
           id_y = j;
           min = temp;       
       end
    end
    
end
X = Xc(:,:,id_x);
Y = Yc(:,:,id_y);

end