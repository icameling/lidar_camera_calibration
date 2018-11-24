clear variables; dbstop error; close all;
disp('================================');

addpath('matching');

imagepath = 'E:\process_data\';
image_prefix = 'car_left';
image_extention = '.jpg';

%%% [image_start_num, image_end_num] °üÀ¨Á½¶Ë

image_start_num = 1;
image_end_num = 6;  

  for idx=image_start_num : image_end_num
        image_name =[imagepath, image_prefix, num2str(idx), image_extention];
        corner_save_name =[imagepath, image_prefix, num2str(idx), '.txt'];
        fprintf('reading %s \n', image_name);
        Image = imread( image_name);
        
        corners = findCorners(Image, 0.01, 1);
        chessboards = chessboardsFromCorners(corners);

        figure; imshow(uint8(Image)); hold on;
        resultXY = plotChessboards(corner_save_name, chessboards,corners);
  end
  

