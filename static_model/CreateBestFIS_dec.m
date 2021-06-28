function fis1 = CreateBestFIS_dec(vec,fis1)  % This function computes the cost value for each individual.

% vec refers to the individual vector from GA's population defined by GA.
% This function is evaluated for each individual during every generation.

warning('off')

vec(:,7:15) = round(vec(:,7:15));
%%% Assign GA individual to the corresponding fis paramters
fis1.input(1).mf(1).params(3) = vec(1);
fis1.input(1).mf(2).params(1) = vec(1);
fis1.input(1).mf(2).params(3) = vec(2);
fis1.input(1).mf(3).params(2) = vec(2);

fis1.input(2).mf(1).params(3) = vec(3);
fis1.input(2).mf(2).params(1) = vec(3);
fis1.input(2).mf(2).params(3) = vec(4);
fis1.input(2).mf(3).params(2) = vec(4);

fis1.output(1).mf(1).params(3) = vec(5);
fis1.output(1).mf(2).params(1) = vec(5);
fis1.output(1).mf(2).params(3) = vec(6);
fis1.output(1).mf(3).params(2) = vec(6);

fis1.rule(1).consequent = vec(7);
fis1.rule(2).consequent = vec(8);
fis1.rule(3).consequent = vec(9);
fis1.rule(4).consequent = vec(10);
fis1.rule(5).consequent = vec(11);
fis1.rule(6).consequent = vec(12);
fis1.rule(7).consequent = vec(13);
fis1.rule(8).consequent = vec(14);
fis1.rule(9).consequent = vec(15);