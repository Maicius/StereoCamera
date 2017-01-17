
filename='视差数据.txt';
data = importdata(filename);
r = data(1);    % 行数
c = data(2);    % 列数
disp = data(3:end); % 视差
vmin = min(disp);
vmax = max(disp);
disp = reshape(disp, [c,r])'; % 将列向量形式的 disp 重构为 矩阵形式
%  OpenCV 是行扫描存储图像，Matlab 是列扫描存储图像
%  故对 disp 的重新排列是首先变成 c 行 r 列的矩阵，然后再转置回 r 行 c 列
img = uint8( 255 * ( disp - vmin ) / ( vmax - vmin ) );
mesh(disp);
set(gca,'YDir','reverse');  % 通过 mesh 方式绘图时，需倒置 Y 轴方向
axis tight; % 使坐标轴显示范围与数据范围相贴合，去除空白显示区
 