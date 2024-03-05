from math import atan2, acos, sin, cos
from math import pi as PI

RADIAN_TO_DEGREE = 180 / PI
DEGREE_TO_RADIAN = PI / 180

l1 = 30
l2 = 30

HOME_THETA1 = -70.53 * DEGREE_TO_RADIAN
HOME_THETA2 = 141.06 * DEGREE_TO_RADIAN


def getXY(t1, t2):
    x = l1 * sin(t1) + l2 * sin(t1 + t2)
    y = l1 * cos(t1) + l2 * cos(t1 + t2)
    return x, y

def getThetas(x, y):
    d2 = x**2 + y**2
    phi = atan2(y, x)
    theta2 = acos((d2 - l1**2 - l2**2) / (2 * l1 * l2))
    theta1 = PI/2 - phi - theta2/2
    theta1 = theta1 * RADIAN_TO_DEGREE
    theta2 = theta2 * RADIAN_TO_DEGREE
    return theta1, theta2

def main():
    X, Y = 0, 0
    X, Y = getXY(HOME_THETA1, HOME_THETA2)
    counter = 0
    while True :
        counter += 1
        if counter == 10 :
            break
        X += 3
        print(getThetas(X, Y))


if __name__ == '__main__':
    main()
