clc;
clear all;

%% Initialization
% Open a FIS
fis1 = mamfis('Name','Multirobot_static_dec');

% Define number of inputs and outputs
ni = 2;
nz = 1;

% Add Inputs and Outputs to FIS
in1 = [-1,1]; % Input 1 range m
in2 = [-pi/2,pi/2]; % Input 2 range rad
out1 = [-0.01,0.01]; % Output range m/s
fis1 = addInput(fis1,in1,'Name',"rho_DT1");
fis1 = addInput(fis1,in2,'Name',"phi_DT1");
fis1 = addOutput(fis1,out1,'Name',"del_p1");

%% Add the MFs
% Two boundaries for each Input and Output:
a = -0.4; b = 0.6; % Input 1
c = -pi/6; d = pi/6; % Input 2
e = -0.005; f = 0.005; % Output

% Input 1 MFs
fis1 = addMF(fis1,"rho_DT1",@trapmf,[in1(1) in1(1) a 0],"Name","Negative","VariableType","input");
fis1 = addMF(fis1,"rho_DT1",@trimf,[a 0 b],"Name","Zero","VariableType","input");
fis1 = addMF(fis1,"rho_DT1",@trapmf,[0 b in1(2) in1(2)],"Name","Positive","VariableType","input");

% Input 2 MFs
fis1 = addMF(fis1,"phi_DT1",@trapmf,[in2(1) in2(1) c 0],"Name","Negative","VariableType","input");
fis1 = addMF(fis1,"phi_DT1",@trimf,[c 0 d],"Name","Zero","VariableType","input");
fis1 = addMF(fis1,"phi_DT1",@trapmf,[0 d in2(2) in2(2)],"Name","Positive","VariableType","input");

% Output 1 MFs
fis1 = addMF(fis1,"del_p1",@trapmf,[out1(1) out1(1) e 0],"Name","Negative","VariableType","output");
fis1 = addMF(fis1,"del_p1",@trimf,[e 0 f],"Name","Zero","VariableType","output");
fis1 = addMF(fis1,"del_p1",@trapmf,[0 f out1(2) out1(2)],"Name","Positive","VariableType","output");

%% Define Rulebase
ruleList = [1 1 1 1 1;
            1 2 1 1 1;
            1 3 2 1 1;
            2 1 1 1 1;
            2 2 2 1 1;
            2 3 2 1 1;
            3 1 2 1 1;
            3 2 3 1 1;
            3 3 3 1 1;];
        
fis1 = addRule(fis1,ruleList);

%% Evaluation using FIS
clc;
% Method of Defuzzification
def_method = ["centroid","mom","lom"];
j = 1; % Index for def_method
fis1.defuzzMethod = def_method(j);

%% Saving the FIS to .fis file
writeFIS(fis1,'Multirobot_static_dec');