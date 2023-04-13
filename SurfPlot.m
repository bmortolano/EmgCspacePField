
figure
surf(intent.intent_field.field, 'EdgeColor', 'None')
hold on
plot3(0,0,intent.intent_field.field(1,1), 'k.', 'MarkerSize', 35)
plot3(0,360,intent.intent_field.field(360,1), 'k.', 'MarkerSize', 35)
plot3(360,0,intent.intent_field.field(1,360), 'k.', 'MarkerSize', 35)