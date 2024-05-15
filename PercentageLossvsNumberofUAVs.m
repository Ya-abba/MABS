x = 10:10:90;

y_A = [1, 2, 3, 4, 5,6,];
%y_B = [96.5052, 69.3334, 48.8973, 37.5665, 27.4886, 22.0438, 19.0141, 14.5306, 11.7861];
%y_C = [99.4292, 72.5845, 50.4524, 37.7699, 28.8066, 23.1472, 17.9228, 15.3089, 12.4875];

figure(1);
plot(x, y_A');
hold on
%plot(x, y_B','*-','Color',[0	0.344827586206897	0]);
%hold on
%plot(x, y_C','+-','Color',[0.517241379310345	0.517241379310345	1]);
legend('UAV', 'FontSize', 12);
h1= xlabel('Percentage Loss %'); 
h2=ylabel('Number of UAVs');
title('Percentage Loss vs Number of UAVs')
%or h=get(gca,'xlabel')
set(h1, 'FontSize', 13)
set(h2, 'FontSize', 13)
grid on;
