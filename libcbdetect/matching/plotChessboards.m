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

function resultXY = plotChessboards(savefilnename, chessboards,corners,v0)

if nargin<5
  v0 = 0;
end

% for all chessboards do
for i=1:length(chessboards)
    
  % extract chessboard
  cb = chessboards{i};
    
  % plot lines in black
  for j=1:size(cb,1)
    p = corners.p(cb(j,:),:)+1;
    plot(p(:,1),v0+p(:,2),'-','Color','r','LineWidth',9);
  end
  for j=1:size(cb,2)
    p = corners.p(cb(:,j),:)+1;
    plot(p(:,1),v0+p(:,2),'-','Color','r','LineWidth',9);
  end
  
  % plot lines in white
  for j=1:size(cb,1)
    p = corners.p(cb(j,:),:)+1;
    plot(p(:,1),v0+p(:,2),'-','Color','w','LineWidth',3);
  end
  for j=1:size(cb,2)
    p = corners.p(cb(:,j),:)+1;
    plot(p(:,1),v0+p(:,2),'-','Color','w','LineWidth',3);
  end
  
  % plot coordinate system
  if 0
    l1 = [corners.p(cb(1,1),:); corners.p(cb(1,2),:)]+1;
    l2 = [corners.p(cb(1,1),:); corners.p(cb(2,1),:)]+1;
    plot(l1(:,1),v0+l1(:,2),'-r','LineWidth',4);
    plot(l2(:,1),v0+l2(:,2),'-g','LineWidth',4);
  end
  
  % plot numbers
  if 0
    c = mean(corners.p(cb(:),:),1)+1;
    text(c(1),v0+c(2),num2str(i),'Color','r','FontSize',30);
    %text(c(1),c(2),num2str(chessboardEnergy(cb,corners)),'Color','r','FontSize',30);
  end
  
  resultX=zeros(size(cb,1), size(cb,2));
  resultY=zeros(size(cb,1), size(cb,2));
  resultXY = zeros(size(cb,1)*size(cb,2), 2);
  resultxxyy=zeros(size(cb,1)*2, size(cb,2));
  counter = 1;
  for i=1:size(cb,1)
      for j=1:size(cb,2)
          p = corners.p(cb(i,j),:)+1;
          resultX(i,j)=p(1);
          resultY(i,j)=p(2);
          resultXY(counter,1) = p(1);
          resultXY(counter,2) = p(2);
          resultxxyy(i,j) = p(1);
          resultxxyy(size(cb,1)+i,j) = p(2);
          counter = counter + 1;
      end
  end
  
%   dlmwrite(savefilnenameX,resultX,' ') 
%   dlmwrite(savefilnenameY,resultY,' ')  
  dlmwrite(savefilnename,resultxxyy,' ') 
  
end
