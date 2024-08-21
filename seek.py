import sys
import os

# Ensure the util directory is in the Python path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), 'util')))
from util.robot import BallerRover
import util.comm as comm
import util.robot as bot

# def main():
#     bot = BallerRover()
#     while True:
#         bot.probe() # search for balls in vicinity
#         bot.primitive_path(bot.ball_pos.pop()) # calculate the primitive path

#         # later down the line,stop if ball is close enough to line up with camera

#         bot.return_to_origin()

def test(bot):
    while True:
        user_input = input("Press any key to move forward, or 'q' to quit: ")
        if user_input == 'q':
            break
        bot.drive(1)  # Go forward
 
if __name__ == "__main__":
    bot = BallerRover()
    test(bot)
