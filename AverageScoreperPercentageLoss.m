x = 10:10:90;

%y_A = [0.872676637, 0.872676637, 0.887538591, 0.886381759, 0.878614952, 0.880704129, 0.886209402, 0.887999023];
y_A = [0.875634292	0.875634292	0.875634292	0.875634292	0.875634292	0.875634292	0.875634292	0.875634292	0.875634292];

y_A = flip(y_A);

figure(1);
plot(x, y_A');
hold on
%plot(x, y_B','*-','Color',[0	0.344827586206897	0]);
%hold on
%plot(x, y_C','+-','Color',[0.517241379310345	0.517241379310345	1]);
legend('UAV', 'FontSize', 12);
h1= xlabel('Percentage Loss %'); 
h2=ylabel('Average Score');
title('Average Score per Percentage Loss')
%or h=get(gca,'xlabel')
set(h1, 'FontSize', 13)
set(h2, 'FontSize', 13)
grid on;
