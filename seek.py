import util.robot as bot

# def main():
#     bot = BallerRover()
#     while True:
#         bot.probe() # search for balls in vicinity
#         bot.primitive_path(bot.ball_pos.pop()) # calculate the primitive path

#         # later down the line, stop if ball is close enough to line up with camera

#         bot.return_to_origin()

if __name__ == "__main__":
    bot = bot.BallerRover()
    bot.drive(2)
