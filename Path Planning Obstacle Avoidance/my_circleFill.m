function my_circleFill(r,x_c,y_c,x_scaling,h_scaling,fColor)
%
% draws a disk with radius r, centered at (x_c,y_c), with fill color fColor
%

angle=-pi:.01:pi;
lP = length(angle);

x_circle = zeros(1,lP);
y_circle = zeros(1,lP);

for i = 1:length(angle)

R=[x_scaling*cos(angle(i)),-x_scaling*sin(angle(i));...
h_scaling*sin(angle(i)),h_scaling*cos(angle(i))];

temp=R*[r;0];

x_circle(i)=temp(1);
y_circle(i)=temp(2);

end

X_circle=x_c+x_circle;
Y_circle=y_c+y_circle;

fill(X_circle,Y_circle,fColor)
