clc
clear

addpath 'C:\Users\ranuh\Desktop\JonW\dronempc\python'

 commandStr = 'python C:\Users\ranuh\Desktop\JonW\dronempc\python\test.py';
 [status, commandOut] = system(commandStr);
 if status==0
     fprintf('Python output is %s\n',commandOut);
 end