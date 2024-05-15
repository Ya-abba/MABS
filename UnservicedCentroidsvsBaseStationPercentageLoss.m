x = 10:10:90;

y_A = [18, 20, 20, 22, 24, 26, 28, 36, 39];
%y_B = [90.5472, 58.1212, 36.2312, 26.3665, 18.7739, 13.0418, 10.0733, 7.44539, 6.00467];


figure(1);
plot(x, y_A','x-','Color',[0	1	0]);
hold on
%plot(x, y_B','>-','Color',[0	0	0.172413793103448])
%hold on
%plot(x, y_C','<-','Color',[1	0.103448275862069	0.724137931034483]);
legend('Unserviced Centroids', 'FontSize', 10);
h1= xlabel('Base Percentage Loss'); h2=ylabel('Unserviced Centroids');
%or h=get(gca,'xlabel')
set(h1, 'FontSize', 13)
set(h2, 'FontSize', 13)
%title('Unserviced Centroids vs Base Station Percentage Loss')