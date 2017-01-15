filename='ÊÓ²îÊı¾İ.txt';
data=importdata(filename);
r=data(1);
c=data(2);
disp = data(3:end);
vmin = min(disp);
vmax = max(disp);
disp = reshape(disp, [c,r]);

img = uint8(255 * (disp - vmin) /(vmax-vmin));
mesh(disp);
set(gca,'YDir','reverse');
axis tight;