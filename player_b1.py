#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32MultiArray, String
import sys



class PlayerB:
    
    def __init__(self):
        
        self.server_hitpoints_sub = rospy.Subscriber("/serverB_hitpoints", Int32MultiArray, self.serverHitpointsCallback, queue_size=1)
        self.playerB_targetnames_pub = rospy.Publisher('/player_b_targets', String, queue_size=10)
        self.hitpoints_all= [0, 0, 0, 0, 0, 0]
        
       
    def serverHitpointsCallback(self, msg):
        monsterBNames = ["Rock", "Thunder", "Wind"]
        monsterANames = ["Fire", "Water", "Earth"]
        
        if(msg.data != self.hitpoints_all):
            self.hitpoints_all= msg.data
            hitpoints = msg.data[3:6]
            print("Player B's Monster Hitpoints:")
            self.printHitpoints(hitpoints, monsterBNames)
        
            opponent_hitpoints = msg.data[:3]
            print("Player A's Monster Hitpoints:")
            self.printHitpoints(opponent_hitpoints, monsterANames)
            targeted_monster_names = self.takeMovesInput()
            self.sendMoves(targeted_monster_names)
            print("received hitpoints")

    
    def takeMovesInput(self):
        targeted_monster_names = []
        for i in range(3):
            move = input("Enter attack move for Monster {}: (1 - Attack One, 2 - Attack All): ".format(i + 1))
            try:
                move = int(move)
            except ValueError:
                print("Invalid input. Please enter a number.")
                return self.takeMovesInput()  # Restart the input process if the input is invalid
        
            if move == 1:
                target_monster_name = input("Enter the targeted monster's name for Monster {}: ".format(i + 1))
                targeted_monster_names.append(target_monster_name)
            else:
                targeted_monster_names.append("0")  # Empty string if move is not 1

        return targeted_monster_names

    def sendMoves(self, targeted_monster_names):
         names_msg = String(data=" ".join(targeted_monster_names))
         self.playerB_targetnames_pub.publish(names_msg)

    def printHitpoints(self, hitpoints, names):
        for i in range(len(hitpoints)):
            print("{}: {} HP".format(names[i], hitpoints[i]))
        print()


if __name__ == '__main__':
    rospy.init_node('player_b')
    player_b = PlayerB()
    rospy.spin()
