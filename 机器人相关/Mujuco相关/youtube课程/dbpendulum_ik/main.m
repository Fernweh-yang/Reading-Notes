clear all
clc
close all
filename = 'data.csv';

T = readtable(filename); %check T.Properties
VariableNames = T.Properties.VariableNames;

Arr = table2array(T);
[m,n] = size(Arr);

% for i=1:n
%     figure(i)
%     yy = i;
%     plot(Arr(:,yy),'r');
%     ylabel(cell2mat(VariableNames(yy)))
% end

figure(1)
plot(Arr(:,2),Arr(:,3),'r');
ylabel(cell2mat(VariableNames(3)))
xlabel(cell2mat(VariableNames(2)))
axis('equal');
