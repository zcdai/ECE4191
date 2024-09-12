import sys
import os

# Ensure the util directory is in the Python path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), 'util')))
from util.robot import BallerRover
import util.comm as comm
import util.robot as bot

def m1():
    bot = BallerRover()
    bot.probe() # search for balls in vicinity
    bot.nav2ball(bot.ball_pos.pop()) # calculate the primitive path, with secondary scan

    # later down the line,stop if ball is close enough to line up with camera

    bot.return_to_origin()


def m2():
    bot = BallerRover()
    while True:
        bot.probe()

 
if __name__ == "__main__":
    bot = BallerRover(pos=[0, 0])
    # # while True:
    # #     x = float(input('x'))
    # #     y = float(input('y'))
    # #     bot.primitive_path([x, y])
    # while True:
    #     targets = bot.get_image()
    #     print(targets)
    m1()