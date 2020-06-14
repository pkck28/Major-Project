% Script for trimimg the F18 Hornet Model.

% clc;
% clear;
% close all;

d2r = pi/180; 

x0 = [435; 0*d2r; 10*d2r; 0*d2r; 0; 0*d2r; 0; 10*d2r; 0; 0; 0; 25000];

u0 = [0;0;-1.26*d2r;5470.5];

IX = [1;2;3;4;5;6;7;8;9];
IU = [];

[xtrim,utrim,~,dx] = trim('F18_Trim',x0,u0,[],IX,IU,[]) ;
