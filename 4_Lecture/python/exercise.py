from graphics import *
from math import sin,cos

win = GraphWin('Exercise', 300, 300)

robotArms = [100, 70, 20]
Q = [1.0, -2.0, 0.0]

base = Point(150,300)
robotPos = [base,Point(0,0),Point(0,0),Point(0,0)]

start = Point(base.x+100,base.y-100)
end = Point(base.x-100,base.y-100)

for i in range(len(robotArms)):
    angle = 0
    for j in range(i+1):
        angle += Q[j]
    robotPos[i+1] = Point(robotArms[i]*sin(angle)+robotPos[i].x,robotPos[i].y-(robotArms[i]*cos(angle)))

if win.isOpen:
        win.close
        
win = GraphWin('Exercise', 300, 300)

line = Line(start, end)
line.draw(win)
line.setOutline('red')
line.setFill('red')

for i in range(len(robotPos)):
    cir = Circle(robotPos[i], 10)
    cir.draw(win)
    cir.setOutline('black')
    cir.setFill('black')
    if i != 0:
        line = Line(robotPos[i], robotPos[i-1])
        line.draw(win)

input("Done")

Q[0] = 0

for i in range(len(robotArms)):
    angle = 0
    for j in range(i+1):
        angle += Q[j]
    robotPos[i+1] = Point(robotArms[i]*sin(angle)+robotPos[i].x,robotPos[i].y-(robotArms[i]*cos(angle)))

if win.isOpen:
        win.close
        
win = GraphWin('Exercise', 300, 300)

line = Line(start, end)
line.draw(win)
line.setOutline('red')
line.setFill('red')

for i in range(len(robotPos)):
    cir = Circle(robotPos[i], 10)
    cir.draw(win)
    cir.setOutline('black')
    cir.setFill('black')
    if i != 0:
        line = Line(robotPos[i], robotPos[i-1])
        line.draw(win)

input("Done")