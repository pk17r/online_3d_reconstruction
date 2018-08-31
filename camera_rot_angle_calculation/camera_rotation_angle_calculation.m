lx = [214, 859];
ly = [401, 402];

px = 551;
py = 395;
cx = 1280/2;
cy=960/2;

aa = imread('rotation_angle.png');
imshow(aa)
hold on
line_width = 3;
point_width = 30;
text_offset = 20;
font_size = 25;

% general measurement line
plot(lx, ly, 'c-', 'LineWidth', line_width);
text(lx(1)+text_offset/2,ly(1)-text_offset,'L1','Color','c','FontSize',font_size)
text(lx(2)+text_offset/2,ly(2)-text_offset,'L2','Color','c','FontSize',font_size)
plot(lx(1), ly(1), 'c.', 'MarkerSize', point_width);
plot(lx(2), ly(2), 'c.', 'MarkerSize', point_width);

% triangle connecting center of image to center of pendulum
plot([cx,px], [cy,py], 'm-', 'LineWidth', line_width);
plot([cx,px], [py,py], 'm-', 'LineWidth', line_width);
plot([cx,cx], [cy,py], 'm-', 'LineWidth', line_width);
text(px-1.5*text_offset,py-text_offset,'P','Color','g','FontSize',font_size)
text(cx+text_offset,cy+text_offset,'C','Color','m','FontSize',font_size)
text(cx+text_offset,py-text_offset,'A','Color','m','FontSize',font_size)

% plotting points
plot(px, py, 'g.', 'MarkerSize', point_width);
plot(cx, cy, 'm.', 'MarkerSize', point_width);
plot(cx, py, 'm.', 'MarkerSize', point_width);

height = 900;   % mm
L1L2_d = 136; % mm
L1L2_px = sqrt((lx(1)-lx(2))^2+(ly(1)-ly(2))^2); % px
px2mm = L1L2_d/L1L2_px;
PA = (cx-px)*px2mm;
CA = (cy-py)*px2mm;
pitch = atand(-CA/height)
roll = atand(PA/height)

xaxis = '\rightarrow';
yaxis = '\downarrow';
text(1100,100,xaxis,'Color','c','FontSize',2*font_size)
text(1087,130,yaxis,'Color','c','FontSize',2*font_size)
text(1170,100,'X','Color','c','FontSize',font_size)
text(1090,185,'Y','Color','c','FontSize',font_size)

pitchtxt = strcat('pitch = ',num2str(pitch),' deg');
rolltxt = strcat('roll = ',num2str(roll),' deg');
text(950,330,pitchtxt,'Color','g','FontSize',font_size)
text(950,370,rolltxt,'Color','g','FontSize',font_size)

saveas(gcf,'rotation_angle_fig.png')

