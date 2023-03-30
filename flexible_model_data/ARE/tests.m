clear all
close all
clc

% For system xdot = Ax + Bu
%               y = Cx + Du

% x is nx1
% u is px1

% A is nxn
% B is nxp

% y is qx1

% C is qxn
% D is qxp

% Q is nxn
% R is pxp

A = [-1,2,3;4,5,-6;7,-8,9];
B = [5;6;-7];
Q = diag([1 2 3]);
R = diag([1]);

P = MyCARE(A,B,Q,R)
icare(A,B,Q,R)
% P = MyDARE(A,B,Q,R)
% idare(A,B,Q,R)
% P = MyLYAP(A,Q)