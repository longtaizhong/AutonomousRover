a = arduino('COM3', 'Uno', 'Libraries',{'Servo', 'Ultrasonic'});
s = servo(a, 'D4');
u = ultrasonic(a, 'D2','D3');
table = zeros(180,2);
%% 
currentPosition = readPosition(s);
for angle = currentPosition:-1/180:0
    writePosition(s, angle);
    pause(1/100);
end;

%% 
i=1;
for angle = 0:1/180:1
    writePosition(s, angle);
    currentPosition = angle*pi;
    distance1 = readDistance(u);
    pause(0.005);
    distance2 = readDistance(u);
    distance = (distance1+distance2)/2;
    table(i,1) = currentPosition;
    table(i,2) = distance*100;
    i=i+1;
    pause(0.005);
end;
%%
polarplot(table(:,1), table(:,2), 'r-o');