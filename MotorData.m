clear;
clc;

load("DATA.TXT");
load("LoadData.TXT");

NoLoadtime = DATA(1:102,1);
NoLoadinput = DATA(1:102,2) + 300;
NoloadRPM = DATA(1:102,3);

Loadtime = LoadData(1:102,1);
Loadinput = LoadData(1:102,2) + 300;
LoadRPM = LoadData(1:102,3);

figure(1)
plot(time,NoLoadinput);
hold on;
plot(time,NoLoadRPM,'*');

figure(2)
plot(time,Loadinput);
hold on;
plot(time,LoadRPM,'*');

NoLoadtf = tf(1506.1,[1 56.55]);
Loadtf = tf(515.5,[1 44.22]);

figure(3)
opt = stepDataOptions('InputOffset',300,'StepAmplitude',200);
step(Loadtf,opt);