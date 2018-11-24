% Copyright 2012. All rights reserved.
% Author: Andreas Geiger
%         Institute of Measurement and Control Systems (MRT)
%         Karlsruhe Institute of Technology (KIT), Germany

% This is free software; you can redistribute it and/or modify it under the
% terms of the GNU General Public License as published by the Free Software
% Foundation; either version 3 of the License, or any later version.

% This software is distributed in the hope that it will be useful, but WITHOUT ANY
% WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
% PARTICULAR PURPOSE. See the GNU General Public License for more details.

% You should have received a copy of the GNU General Public License along with
% this software; if not, write to the Free Software Foundation, Inc., 51 Franklin
% Street, Fifth Floor, Boston, MA 02110-1301, USA 

function h = plotChessboardMatching(I,chessboards,corners,BoardObservation,plot_corners)

% stack images vertically
I_side = uint8(I{1});
for c=2:length(I)
  I_side = [I_side; uint8(I{c})];
end

% find good width and height for figure
I_side = (I_side-0.5)/1.5+0.3;
width  = 640;
height = width*size(I_side,1)/size(I_side,2);

% create figure and show image
h = figure('Position',[100 100 width height]);axes('Position',[0 0 1 1]);
imshow(I_side); hold on;

% plot chessboards + corners
v0 = 0;
for cam=1:length(I)
  plotChessboards(chessboards{cam},corners{cam},v0);
  if nargin==5 && plot_corners
    plotCorners(corners{cam},v0);
  end
  v0 = v0+size(I{cam},1);
end

% plot correspondences
v0 = 0;
for cam=1:length(I)-1
  plotCornerCorrespondences(BoardObservation,cam,v0,v0+size(I{cam},1));
  v0 = v0+size(I{cam},1);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function plotCorners(corners,v0)

for i=1:size(corners.p,1)
  v1 = corners.v1(i,:);
  v2 = corners.v2(i,:);
  p  = [corners.p(i,:)-5*v1; ...
        corners.p(i,:)+5*v1; ...
        corners.p(i,:)-5*v2; ...
        corners.p(i,:)+5*v2];
  plot(p(1:2,1),v0+p(1:2,2),'-r','LineWidth',2);
  plot(p(3:4,1),v0+p(3:4,2),'-r','LineWidth',2);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function plotCornerCorrespondences(BoardObservation,cam,v0,v1)

% for all chessboards do
if ~isempty(BoardObservation)
  
  % for all boards do
  for board=1:length(BoardObservation{cam})

    % get color
    col = colorFromIndex(board);

    % for all corners of this board do
    for i=1:size(BoardObservation{cam}{board}.x,2)
      l = [BoardObservation{cam}{board}.x(:,i)'  + [0 v0];...
           BoardObservation{cam+1}{board}.x(:,i)' + [0 v1]];
      plot(l(:,1),l(:,2),'-','Color',col,'LineWidth',1);
    end
  end
end

