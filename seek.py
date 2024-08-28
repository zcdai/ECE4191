import sys
import os

# Ensure the util directory is in the Python path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), 'util')))
from util.robot import BallerRover
import util.comm as comm
import util.robot as bot
import cv2

# def main():
#     bot = BallerRover()
#     while True:
#         bot.probe() # search for balls in vicinity
#         bot.primitive_path(bot.ball_pos.pop()) # calculate the primitive path

#         # later down the line,stop if ball is close enough to line up with camera

#         bot.return_to_origin()

 
if __name__ == "__main__":
    bot = BallerRover()
    # bot.direct_path([2,1])
    # bot.set_angle(0)
    # bot.return_to_origin()
    
    bot.rotate(90)
    print(bot.pos)
    bot.rotate(-90)
    bot.drive(distance=0.5)