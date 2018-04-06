function [ Xfree ] = LG_jump2free( X )
%LG_Z2N Summary of this function goes here
%   Detailed explanation goes here
% Xfree = [PosQ,AttQ,PosP]
Xfree = [X(1:6);X(11:16);X(17:22)];
end

