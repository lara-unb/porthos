

close all;
clear all;
clc;

%user defined parameters
r=0.05;
l=0.05;

aux=load('pathAndControls.txt');

v_control_aux=(aux(:,4))';
theta_control_aux=(aux(:,5))';
dur_control=(aux(:,6))';
x=(aux(:,1))';
y=(aux(:,2))';

%time inputs
tinc=.05;
tmax=sum(dur_control)/tinc;
v_control=zeros(tmax)(1,:);
theta_control=v_control;
xi=v_control;
yi=v_control;
theta=v_control;
%robot's initial pose
xi(1)=-2;
yi(1)=0;
theta(1)=0;
%goal input

xob=zeros(4);
xob(1)=xob(2)=5;
xob(3)=xob(4)=13;
yob=zeros(4);
yob(1)=yob(4)=-2;
yob(2)=yob(3)=8.5;

xot=zeros(4);
xot(1)=xot(2)=5;
xot(3)=xot(4)=13;
yot=zeros(4);
yot(1)=yot(4)=20;
yot(2)=yot(3)=9.5;
figure;
%simulation's plots
t0=plot(x,y);
set(t0,'Color','blue')
axis equal
xlabel('x')
ylabel('y')
hold on;

t1=plot(xob,yob);
set(t1,'Color','black');

hold on;

t2=plot(xot,yot);
set(t2,'Color','black')




