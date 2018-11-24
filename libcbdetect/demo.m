clear variables; dbstop error; close all;
disp('================================');

addpath('matching');

imagepath = 'D:\pdl\front\front4.jpg';
saveCornertxt = 'front3.txt'; 


I = imread( imagepath );
corners = findCorners(I,0.01,1);

chessboards = chessboardsFromCorners(corners);

figure; imshow(uint8(I)); hold on;
resultXY = plotChessboards(saveCornertxt, chessboards,corners);
