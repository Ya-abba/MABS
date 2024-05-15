x = 1:5;
y_A = [30, 24, 33, 34, 31, 20, 14, 20, 13];
y_B = [26, 30, 25, 24, 19, 28, 19, 17, 23];
y_C = [34, 26, 33, 22, 29, 17, 8, 17, 16];
y_D = [31, 20, 22, 27, 4, 21, 10, 8, 8];
y_E = [36, 22, 27, 16, 17, 19, 15, 10, 16];

y_A = flip(y_A);
y_B = flip(y_B);
y_C = flip(y_C);
y_D = flip(y_D);
y_E = flip(y_E);

figure;
bar(x, [y_A', y_B', y_C', y_D', y_E'], 'grouped');
legend('90 % BS loss','80 % BS loss', '70 % BS loss','60 % BS loss', '50 % BS loss','40 % BS loss', '30 % BS loss','20 % BS loss','10 % BS loss' ,'FontSize', 10);xlabel('Allowed Drones')
ylabel('Number of Candidates')
%title('Allowed Drones vs Number of Candidates')
%or h=get(gca,'xlabel')
set(h1, 'FontSize', 13)
set(h2, 'FontSize', 13)
grid on;