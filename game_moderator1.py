#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32MultiArray, String
import sys

# Define the states
class GameState:
    WAITING_FOR_PLAYER_A = 0
    PROCESSING_PLAYER_A = 1
    WAITING_FOR_PLAYER_B = 2
    PROCESSING_PLAYER_B = 3
    GAME_OVER = 4

class GameModerator:
    def __init__(self):
        self.state = GameState.WAITING_FOR_PLAYER_A
        self.playerA_inputs = []
        self.playerB_inputs = []
        
        self.playerA_targets_sub = rospy.Subscriber("/player_a_targets", String, self.playerATargetsCallback, queue_size=10)
        self.playerB_targets_sub = rospy.Subscriber("/player_b_targets", String, self.playerBTargetsCallback, queue_size=10)
        self.serverA_hitpoints_pub = rospy.Publisher("/serverA_hitpoints", Int32MultiArray, queue_size=10)
        self.serverB_hitpoints_pub = rospy.Publisher("/serverB_hitpoints", Int32MultiArray, queue_size=10)
        self.maxplayerA_hitpoints = [300, 400, 500]
        self.maxplayerB_hitpoints = [300, 400, 500]
        self.playerA_hitpoints = [300, 400, 500]
        self.playerB_hitpoints = [300, 400, 500]
        self.monsterNames = ["Fire", "Water", "Earth", "Rock", "Thunder", "Wind"]

    def startGame(self):
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            if self.state == GameState.WAITING_FOR_PLAYER_A:
                # Waiting for input from Player A
                self.publishServerAHitpoints()
                rospy.sleep(1)  # Wait for Player A's input
                if self.checkInputsComplete("Player A"):
                    self.state = GameState.PROCESSING_PLAYER_A
                
            elif self.state == GameState.PROCESSING_PLAYER_A:
                # Processing Player A's moves
                self.processPlayerAMoves()
                self.state = GameState.WAITING_FOR_PLAYER_B
                
            elif self.state == GameState.WAITING_FOR_PLAYER_B:
                # Waiting for input from Player B
                self.publishServerBHitpoints()
                rospy.sleep(1)  # Wait for Player B's input
                if self.checkInputsComplete("Player B"):
                    self.state = GameState.PROCESSING_PLAYER_B
                
            elif self.state == GameState.PROCESSING_PLAYER_B:
                # Processing Player B's moves
                self.processPlayerBMoves()
                self.state = GameState.WAITING_FOR_PLAYER_A
                
            elif self.state == GameState.GAME_OVER:
                # Game over
                print("Game over!")
                rospy.signal_shutdown("Game Over")

            rate.sleep()
            
    def checkInputsComplete(self, player):
         if player == "PlayerA":
              return all(input_value is not None for input_value in self.playerA_inputs)
         elif player == "PlayerB":
              return all(input_value is not None for input_value in self.playerB_inputs)
         else:
              return False
        

    def playerATargetsCallback(self, msg):
        if self.state != GameState.WAITING_FOR_PLAYER_A:
            return
        
        targeted_monster_names = list(msg.data.split())[:3]
        self.playerA_inputs = targeted_monster_names

    def processPlayerAMoves(self):
        targeted_monster_names = self.playerA_inputs
        attack_moves = []
        monsterANames = ["Fire", "Water", "Earth"]
        
        targeted_monster_index = []
        for i in range(3):
            target_monster = self.selectMonsterToAttack(self.playerB_hitpoints, targeted_monster_names[i])
            targeted_monster_index.append(target_monster[0])
            
            if target_monster[0] != -1:
                attack_moves.append(1)
            else:
                attack_moves.append(2)
        
        self.printMoves(attack_moves, monsterANames, targeted_monster_names)
        self.processAMoves(attack_moves, monsterANames, targeted_monster_names, targeted_monster_index)
        
        self.playerA_inputs = []  # Clear the inputs after processing

    def playerBTargetsCallback(self, msg):
        if self.state != GameState.WAITING_FOR_PLAYER_B:
            return
        
        targeted_monster_names = list(msg.data.split())[:3]
        self.playerB_inputs = targeted_monster_names

    def processPlayerBMoves(self):
        targeted_monster_names = self.playerB_inputs
        attack_moves = []
        monsterBNames = ["Rock", "Thunder", "Wind"]
        
        targeted_monster_index = []
        for i in range(3):
            target_monster = self.selectMonsterToAttack(self.playerA_hitpoints, targeted_monster_names[i])
            targeted_monster_index.append(target_monster[0])
            
            if target_monster[0] != -1:
                attack_moves.append(1)
            else:
                attack_moves.append(2)
        
        self.printMoves(attack_moves, monsterBNames, targeted_monster_names)
        self.processBMoves(attack_moves, monsterBNames, targeted_monster_names, targeted_monster_index)
        
        self.playerB_inputs = []  # Clear the inputs after processing

    def processAMoves(self, attack_moves, attacker_names, opponent_names, targeted_index):
        for i in range(len(attack_moves)):
            move = attack_moves[i]
            attacker_monster = attacker_names[i]

            target_index = targeted_index[i]
            if move == 1:
                # Attack One: Deal 20% damage to the selected monster
                damage = int(0.2 * self.maxplayerA_hitpoints[target_index])
                self.playerB_hitpoints[target_index] -= damage
                print(attacker_monster + " attacked " + opponent_names[i])
            elif move == 2:
                # Attack All: Deal 10% damage to all opponent monsters
                damage = int(0.1 * self.maxplayerA_hitpoints[i])
                for j in range(len(self.maxplayerA_hitpoints)):
                    self.playerB_hitpoints[j] -= damage
                print(attacker_monster + " attacked all")
        
        

    def processBMoves(self, attack_moves, attacker_names, opponent_names, targeted_index):
        for i in range(len(attack_moves)):
            move = attack_moves[i]
            attacker_monster = attacker_names[i]

            target_index = targeted_index[i]
            if move == 1:
                # Attack One: Deal 20% damage to the selected monster
                damage = int(0.2 * self.maxplayerA_hitpoints[target_index])
                self.playerA_hitpoints[target_index] -= damage
                print(attacker_monster + " attacked " + opponent_names[i])
            elif move == 2:
                # Attack All: Deal 10% damage to all opponent monsters
                damage = int(0.1 * self.maxplayerA_hitpoints[i])
                for j in range(len(self.maxplayerA_hitpoints)):
                    self.playerA_hitpoints[j] -= damage
                print(attacker_monster + " attacked all")   

    def selectMonsterToAttack(self, opponent_hitpoints, target_monster_name):
        if target_monster_name == "0":
            return -1, ""
        
        for i in range(len(self.monsterNames)):
            if self.monsterNames[i] == target_monster_name:
                if opponent_hitpoints[i % 3] > 0 and i < 3:
                    return i, target_monster_name
                elif opponent_hitpoints[i % 3] > 0 and i >= 3:
                    return i % 3, target_monster_name
                else:
                    print(target_monster_name + " has already been defeated. Choose another target.")
                    break
        
        print("Invalid target monster name: " + target_monster_name)
        return -1, ""

    def publishServerAHitpoints(self):
        hitpoints_msg = Int32MultiArray()
        hitpoints_msg.data = self.playerA_hitpoints + self.playerB_hitpoints
        self.serverA_hitpoints_pub.publish(hitpoints_msg)

    def publishServerBHitpoints(self):
        hitpoints_msg = Int32MultiArray()
        hitpoints_msg.data = self.playerA_hitpoints + self.playerB_hitpoints
        self.serverB_hitpoints_pub.publish(hitpoints_msg)
    
    def printMoves(self, attack_moves, monster_names, targeted_monster_names):
        for i in range(len(attack_moves)):
            move = attack_moves[i]
            monster_name = monster_names[i]

            print("Move {}: ".format(i + 1), end="")
            if move == 1:
                print(monster_names[i] + "'s turn: 1 " + targeted_monster_names[i])
            elif move == 2:
                print(monster_names[i] + "'s turn: 2")
            else:
                print("Unknown Move")

    def checkInputsComplete(self, player):
        if player == "Player A":
            return bool(self.playerA_inputs)
        elif player == "Player B":
            return bool(self.playerB_inputs)
        else:
            return False

    def checkWinner(self, playerA, playerB, hitpointsA, hitpointsB):
        if all(hitpoint <= 0 for hitpoint in hitpointsA):
            print("Game over! {} wins!".format(playerB))
            return True

        if all(hitpoint <= 0 for hitpoint in hitpointsB):
            print("Game over! {} wins!".format(playerA))
            return True

        return False


if __name__ == '__main__':
    rospy.init_node('game_moderator')
    game_moderator = GameModerator()
    game_moderator.startGame()


