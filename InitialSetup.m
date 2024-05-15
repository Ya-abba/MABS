x = 1;
vals = [611,1826,2438];
b = bar(x,vals);
legend('UEs without Service','UEs with Service', 'Total Number of UEs', 'FontSize', 10);
h1= xlabel('UE Categories'); h2=ylabel('Number of UEs');
set(h1, 'FontSize', 13)
set(h2, 'FontSize', 13)
grid on;