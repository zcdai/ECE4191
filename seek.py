import sys
import os

# Ensure the util directory is in the Python path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), 'util')))
from util.robot import BallerRover
import util.robot as bot

def m2(bot: BallerRover):
    collected_count = 0
    # bot.direct_path([2, 2])
    while True:
        bot.probe()
        bot.direct_path(bot.get_closest_ball(), shortstop=True)
        ball_distance = bot.center_ball()

        if ball_distance is not None:
            bot.drive('F', ball_distance + 0.1, modifier='S') # overshoot ball a little bit
            bot.pickup_ball()
            collected_count += 1
            if collected_count == 3:
                bot.deposit_ball()
                bot.direct_path([2.5, 3.5])
                collected_count = 0

 
if __name__ == "__main__":
    bot = BallerRover()
    m2(bot)