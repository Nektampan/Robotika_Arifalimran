#!/usr/bin/env python3

import rospy
from turtlesim.srv import Spawn, Kill
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import random
import sys
import termios
import tty
import threading

class TurtlePong:
    def __init__(self):
        rospy.init_node('turtle_pong_game')
        
        # Score initialization
        self.score_left = 0
        self.score_right = 0
        
        # Kill default turtle
        rospy.wait_for_service('kill')
        kill_turtle = rospy.ServiceProxy('kill', Kill)
        kill_turtle('turtle1')
        
        # Spawn paddles
        rospy.wait_for_service('spawn')
        spawn_turtle = rospy.ServiceProxy('spawn', Spawn)
        
        # Left paddle (Player 1 - WASD keys)
        self.left_paddle = spawn_turtle(1.0, 6.0, 0, 'left_paddle').name
        self.left_pub = rospy.Publisher(f'/{self.left_paddle}/cmd_vel', Twist, queue_size=10)
        
        # Right paddle (Player 2 - Arrow keys)
        self.right_paddle = spawn_turtle(10.0, 6.0, 0, 'right_paddle').name
        self.right_pub = rospy.Publisher(f'/{self.right_paddle}/cmd_vel', Twist, queue_size=10)
        
        # Ball
        self.ball = spawn_turtle(5.5, 5.5, 0, 'ball').name
        self.ball_pub = rospy.Publisher(f'/{self.ball}/cmd_vel', Twist, queue_size=10)
        
        # Subscribe to poses
        rospy.Subscriber(f'/{self.ball}/pose', Pose, self.ball_callback)
        rospy.Subscriber(f'/{self.left_paddle}/pose', Pose, self.left_callback)
        rospy.Subscriber(f'/{self.right_paddle}/pose', Pose, self.right_callback)
        
        # Initialize positions and velocities
        self.ball_pose = Pose()
        self.left_pose = Pose()
        self.right_pose = Pose()
        self.ball_velocity = Twist()
        self.ball_velocity.linear.x = 2.0
        self.ball_velocity.linear.y = 2.0
        
        # Keyboard control
        self.settings = termios.tcgetattr(sys.stdin)
        
        # Command velocities for paddles
        self.left_cmd = Twist()
        self.right_cmd = Twist()

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def ball_callback(self, pose):
        self.ball_pose = pose
        
    def left_callback(self, pose):
        self.left_pose = pose
        
    def right_callback(self, pose):
        self.right_pose = pose
        
    def keyboard_control(self):
        while not rospy.is_shutdown():
            key = self.getKey()
            
            # Left paddle control (WASD)
            if key == 'w':
                self.left_cmd.linear.y = 2.0
            elif key == 's':
                self.left_cmd.linear.y = -2.0
            elif key in ['a', 'd']:  # Stop vertical movement
                self.left_cmd.linear.y = 0.0
                
            # Right paddle control (IJKL)
            elif key == 'i':
                self.right_cmd.linear.y = 2.0
            elif key == 'k':
                self.right_cmd.linear.y = -2.0
            elif key in ['j', 'l']:  # Stop vertical movement
                self.right_cmd.linear.y = 0.0
                
            # Quit game
            elif key == 'q':
                rospy.signal_shutdown('Game ended by user')
                break

            self.left_pub.publish(self.left_cmd)
            self.right_pub.publish(self.right_cmd)
            
    def update_ball(self):
        # Check for collisions with walls
        if self.ball_pose.y >= 11.0 or self.ball_pose.y <= 0.0:
            self.ball_velocity.linear.y *= -1
            
        # Check for collisions with paddles
        if (self.ball_pose.x <= 2.0 and abs(self.ball_pose.y - self.left_pose.y) < 1.0):
            self.ball_velocity.linear.x *= -1
            self.ball_velocity.linear.x *= 1.1  # Increase speed after paddle hit
            
        if (self.ball_pose.x >= 9.0 and abs(self.ball_pose.y - self.right_pose.y) < 1.0):
            self.ball_velocity.linear.x *= -1
            self.ball_velocity.linear.x *= 1.1  # Increase speed after paddle hit
            
        # Check for scoring
        if self.ball_pose.x <= 0.0:
            self.score_right += 1
            self.reset_ball()
            rospy.loginfo(f"Score - Left: {self.score_left}, Right: {self.score_right}")
            
        elif self.ball_pose.x >= 11.0:
            self.score_left += 1
            self.reset_ball()
            rospy.loginfo(f"Score - Left: {self.score_left}, Right: {self.score_right}")
            
        self.ball_pub.publish(self.ball_velocity)
        
    def reset_ball(self):
        # Reset ball position and velocity
        reset_pose = Pose()
        reset_pose.x = 5.5
        reset_pose.y = 5.5
        
        self.ball_velocity.linear.x = 2.0 if random.random() > 0.5 else -2.0
        self.ball_velocity.linear.y = 2.0 if random.random() > 0.5 else -2.0
        
        # Brief pause before continuing
        rospy.sleep(1.0)
        
    def run(self):
        # Start keyboard control in a separate thread
        keyboard_thread = threading.Thread(target=self.keyboard_control)
        keyboard_thread.daemon = True
        keyboard_thread.start()
        
        rate = rospy.Rate(30)  # Increased rate for smoother gameplay
        
        rospy.loginfo("Game Started!")
        rospy.loginfo("Controls:")
        rospy.loginfo("Player 1 (Left): W (up) / S (down)")
        rospy.loginfo("Player 2 (Right): I (up) / K (down)")
        rospy.loginfo("Press Q to quit")
        
        while not rospy.is_shutdown():
            self.update_ball()
            rate.sleep()

if __name__ == '__main__':
    try:
        game = TurtlePong()
        game.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, game.settings)
