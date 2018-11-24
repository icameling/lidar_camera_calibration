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

function E = chessboardEnergy(chessboard,corners)

% energy: number of corners
E_corners = -size(chessboard,1)*size(chessboard,2);

% energy: structure
E_structure = 0;
  
% walk through rows
for j=1:size(chessboard,1)
  for k=1:size(chessboard,2)-2
    x           = corners.p(chessboard(j,k:k+2),:);
    E_structure = max(E_structure,norm(x(1,:)+x(3,:)-2*x(2,:))/norm(x(1,:)-x(3,:)));
  end
end

% walk through columns
for j=1:size(chessboard,2)
  for k=1:size(chessboard,1)-2
    x           = corners.p(chessboard(k:k+2,j),:);
    E_structure = max(E_structure,norm(x(1,:)+x(3,:)-2*x(2,:))/norm(x(1,:)-x(3,:)));
  end
end

% final energy
E = E_corners + 1*size(chessboard,1)*size(chessboard,2)*E_structure;
