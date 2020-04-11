clear
clc

addpath 'C:\Users\ranuh\Desktop\JonW\dronempc\python'

x = datatest;

y=sqrt(x);

commandStr = sprintf('python C:\\Users\\ranuh\\Desktop\\JonW\\dronempc\\python\\IPpydatatest.py %d', y);
[status, commandOut] = system(commandStr)
