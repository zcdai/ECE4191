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

def test(bot):
    while True:
        direction = input("Enter the direction: ").upper()
        distance = float(input("Enter the distance: "))
        bot.drive(direction, distance)
        user_input = input("Press any key to continue, or 'q' to quit: ")
        if user_input == 'q':
            break
 
if __name__ == "__main__":
    radius = 0.114
    bot = BallerRover(radius)
    
    while True:
        print(bot.get_image())
