camfolder='disparities/';
camfolder2 = 'disparities_color/';
images = dir(strcat(camfolder,'*.png'));
imCounts = double(string(strsplit(erase([images(:).name],'png'),'.')));
im_nums_sorted = sort(imCounts(1:length(imCounts)-1));

for i = 1:length(im_nums_sorted)
    I = imread(strcat(camfolder,num2str(im_nums_sorted(i)),'.png'));
    I2 = ind2rgb(I,jet);
    I3 = I2(:,65:640,:);
    imwrite(I3,strcat(camfolder2,num2str(im_nums_sorted(i)),'.png'))
end