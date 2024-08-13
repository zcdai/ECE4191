from util.robot import DriveCommand, PenguinPi

def check_contact(image):
    pass
    # if ball is in pickup range

def check_valid_ball(image):
    pass
    # if ball is on screen and close enough to go path to


def nav2ball(image):
    forward_speed = 0
    turn_speed = 0
    pass
    return DriveCommand(forward_speed, turn_speed)
    # rotate bot to center the ball in the image, and appraoch using pid?

def pickup():
    pass
    # pickup the ball


def main():
    bot = PenguinPi()
    while True:
        bot.probe() # search for balls in vicinity
        bot.primitive_path(bot.ball_pos.pop()) # calculate the primitive path