clear
close all
clc

m=map();

n=input('Insert the number of obstacles that you want:');

figure('units','normalized','outerposition',[0 0 1 1]);
xlim([0 50])
ylim([0 50])
axis square
grid on
hold on
m=m.readMap(n);

filename=input('Insert the filename:','s');
m.saveMap(filename);
hold off