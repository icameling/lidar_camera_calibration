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

function [Board,BoardObservation,BoardCorner] = observationsFromMatching(corners,chessboards,matching,cornerdist)

Board            = [];
BoardObservation = [];
BoardCorner      = [];

% only one camera
if length(chessboards)==1
  for b=1:length(chessboards{1})
    
    % generate 3d coordinates in board coordinate system
    % (columnwise to match MATLAB's memory access pattern)
    X = [];
    for x=0:size(chessboards{1}{b},2)-1
      for y=0:size(chessboards{1}{b},1)-1
        X = [X cornerdist*[x;y;0]];
      end
    end
    
    % set board
    Board{b}.X = X;
    Board{b}.r = zeros(3,1);
    Board{b}.T = zeros(3,1);
    
    % for reference camera do
    BoardObservation{1}{b}.x      = corners{1}.p(chessboards{1}{b},:)';
    BoardObservation{1}{b}.active = ones(1,size(BoardObservation{1}{b}.x,2));
    BoardObservation{1}{b}.size   = size(chessboards{1}{b});
  end
  
else
  
  % for all chessboards in reference image do
  b=1;
  for i=1:size(matching,2)

    % if this board has been matched to all other images/cameras => create a new "Board"
    if ~any(matching(1:2:end,i)==0)

      % generate 3d coordinates in board coordinate system
      % (columnwise to match MATLAB's memory access pattern)
      X = [];
      for x=0:size(chessboards{1}{i},2)-1
        for y=0:size(chessboards{1}{i},1)-1
          X = [X cornerdist*[x;y;0]];
        end
      end

      % set board
      Board{b}.X = X;
      Board{b}.r = zeros(3,1);
      Board{b}.T = zeros(3,1);

      % for reference camera do
      board_idx = i; % since this is the reference board
      BoardObservation{1}{b}.x      = corners{1}.p(chessboards{1}{board_idx},:)';
      BoardObservation{1}{b}.active = ones(1,size(BoardObservation{1}{b}.x,2));
      BoardObservation{1}{b}.size   = size(chessboards{1}{board_idx});

      % for all target cameras do
      for c=2:length(chessboards)
        board_idx  = matching((c-2)*2+1,i);
        rotation   = matching((c-2)*2+2,i);
        chessboard = rotateChessboard(chessboards{c}{board_idx},rotation);
        BoardObservation{c}{b}.x      = corners{c}.p(chessboard,:)';
        BoardObservation{c}{b}.active = ones(1,size(BoardObservation{c}{b}.x,2));
        BoardObservation{c}{b}.size   = BoardObservation{1}{b}.size;
      end

      % increment board counter
      b=b+1;
    end
  end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function chessboard = rotateChessboard(chessboard,rotation)

% rotate chessboard clockwise
for i=2:rotation
  chessboard = chessboard(end:-1:1,:)';
end
