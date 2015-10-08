function my_RectangleFill(x_o,y_o,dx,dy,x_scaling,h_scaling,fColor)
%
% draws a rectangel of size dx,dy with lower left corner x_o,y_o 

angle=-pi:.01:pi;
lP = length(angle);

x_circle = zeros(1,lP);
y_circle = zeros(1,lP);

for i = 1:length(angle)
a
R=[x_scaling*cos(angle(i)),-x_scaling*sin(angle(i));...
h_scaling*sin(angle(i)),h_scaling*cos(angle(i))];

temp=R*[r;0];

x_circle(i)=temp(1);
y_circle(i)=temp(2);

end

X_circle=x_c+x_circle;
Y_circle=y_c+y_circle;

fill(X_circle,Y_circle,fColor)
