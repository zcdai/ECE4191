import sys
import os

# Ensure the util directory is in the Python path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), 'util')))
from util.robot import BallerRover
import util.comm as comm
import util.robot as bot

def m2(bot):
    while True:
        bot.probe()

 
if __name__ == "__main__":
    bot = BallerRover()
    m2(bot)