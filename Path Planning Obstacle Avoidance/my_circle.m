function my_circle(r,x_c,y_c,x_scaling,h_scaling,lt,lw)
%
% draws a circle with radius r, centered at (x_c,y_c), linetype lt, and line
% width lw
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

plot(X_circle,Y_circle,lt,'LineWidth',lw)
