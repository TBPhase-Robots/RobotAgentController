import imp
import pygame
import colours
import sys
from pygame.locals import *
import numpy as np
from model.Agent import Agent
import json
from scipy.spatial import Voronoi, voronoi_plot_2d
import matplotlib.pyplot as plt
import math
import random
from datetime import datetime

import time
import os
from ProtoInputHandler import ProtoInputHandler
from model.Listener import Listener
from model.CommandListener import CommandListener
from model.AgentListener import AgentListener
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64
from std_msgs.msg import ColorRGBA
from std_msgs.msg import Int32



state = "standby_setup_loop"
postedUpdates = 0
sendUpdates = False

robot_dog_speed = 0
robot_pig_speed = 0
robot_sheep_speed = 0
robot_standby_speed = 0

pack = pygame.sprite.Group()
flock = pygame.sprite.Group()
pigs = pygame.sprite.Group()
standby = pygame.sprite.Group()
agents = pygame.sprite.Group()


config_name='defaultConfig'


with open(f"experiment_config_files/{config_name}.json") as json_file:
    cfg = json.load(json_file)

if ('show_empowerment' not in cfg):
    cfg['show_empowerment'] = True

# controller callback is called by an agent whenever it has successfully updated its state via ROS pose
def ControllerCallback(data):
    global postedUpdates
    postedUpdates += 1
   # print("topic contents:")
   # print(data)

def DispatchCallback(data):
    print("DispatchCallback")
    print(data.data)
    # if the user has sent command to dispatch
    if(data.data == "dispatch"):
        # check if there are any agents left in the standby pool
        if(len(standby) > 0):
            # grab the first in agent in the standby pool
            agent = GetAnyAgentFromGroup(standby)
            agent.role = "dog"
            SortAgentsByRole()
    # take a dog off the field and return to standby
    if(data.data == "recall"):
        if(len(pack) > 0):
            agent = GetAnyAgentFromGroup(pack)
            agent.role = "standby"
            SortAgentsByRole()



def GetAnyAgentFromGroup(group):
    for agent in group:
        return agent


# command listener callback is called whenever a new state is recieved by user via the state controller script, utilising CommandListener.py
def CommandListenerCallback(data):
    global state
    print("CommandListenerData:")
    print(data.data)
    state = data.data

def calc_voronoi_partitioning(flock, pack):
    for dog in pack:
        dog.empty_sub_flock()

    for sheep in flock:
        min_dist = 10000
        for dog in pack:
            dist = np.linalg.norm(sheep.position - dog.position)
            if dist < min_dist:
                min_dist = dist
                sheep.set_closest_dog(dog)
        sheep.closest_dog.add_sheep_to_sub_flock(sheep)
#end function

def add_agent(agents, position, cfg, id, camWidth, camHeight, worldWidth, worldHeight, screen):
    agent = Agent(position = position, id = id, cfg = cfg, rotation=0, poseAgentCallback=ControllerCallback, role = "agent", camWidth=camWidth, camHeight=camHeight, worldWidth=worldWidth, worldHeight=worldHeight, screen = screen)
    agents.add(agent)
    agent.role 
    return id + 1

def add_agent_callback(msg):
    global agents
    print("========= callback done")
    print(msg)
    new_id = msg.data

    randx = random.uniform(50, 300)
    randy = random.uniform(50, 300)
    add_agent(agents=agents, position = np.array([randx,randy]), cfg = cfg, id = new_id, camWidth=cfg['cam_width'], camHeight=cfg['cam_height'], worldWidth=cfg['world_width'], worldHeight=cfg['world_height'], screen=screen)



def DrawWorld(cfg):
    screen.fill(colours.DGREY)
    pygame.draw.rect(screen, colours.GREY, pygame.Rect(0, 0, cfg['world_width'], cfg['world_height']))
    # Draw target box
    pygame.draw.rect(screen, colours.RED, pygame.Rect(cfg['target_position'][0] - 100, cfg['target_position'][1] - 100, 200, 200), 3)

    pygame.draw.rect(screen, colours.BLUE, pygame.Rect(cfg['play_area_x'], cfg['play_area_y'], cfg['play_area_width'], cfg['play_area_height']), 3)

    for pos in cfg['initial_sheep_positions']:
        pygame.draw.circle(screen, colours.WHITE, pos, 8)

    for pos in cfg['initial_dog_positions']:
        pygame.draw.circle(screen, colours.BLUE, pos, 8)

    for pos in cfg['standby_positions']:
        pygame.draw.circle(screen, colours.RED, pos, 8)

    pygame.display.flip()


# calls standard behaviour on all sheep and dog agents for simulation
def ExperimentUpdateTimestep(agents, pack, flock, cfg):
    if (len(pack) > 0):
        calc_voronoi_partitioning(flock, pack)
        for dog in pack:
            dog.SimulationUpdate_Dog(screen, flock, pack, cfg)
        
    else:
        for sheep in flock:
            sheep.closest_dog = None

    if(len(flock) > 0):
        for sheep in flock:

            sheep.SimulationUpdate_Sheep(screen, flock, pack, cfg)                  
    for pig in pigs:
        pig.DrawSelf(screen)
    for agent in standby:
        agent.DrawSelf(screen)

def MoveToPointDecision(agent, movePos, cfg):
    point_x = movePos[0]
    point_y = movePos[1]
    agentPos = np.array([agent.position[0], agent.position[1]])
    if(np.linalg.norm(movePos - agentPos) > 25):
        agent.MoveToPoint(point_x = point_x, point_y = point_y, screen = screen, agents = agents, cfg = cfg)
    else:
        agent.HaltAgent(screen=screen)

def StandbySetupUpdateTimestep(agents, cfg):
    # make all agents go to top
    standbyPositions = cfg['standby_positions']
    i = 0
    for agent in agents:
        i +=1 
        point = standbyPositions[i]
        # if the distance between target point and self is very low, halt movement and do not send move command
        movePos = np.array(point)
        MoveToPointDecision(agent = agent, movePos = movePos, cfg = cfg)

def SetAllAgentRolesToStandby():
    for agent in agents:
        agent.role = "standby"

def SortAgentsByRole():
    # get maximum robot speeds and transmit
    global robot_dog_speed
    global robot_pig_speed
    global robot_sheep_speed
    global robot_standby_speed

    print("sort agents by role")

    pack.empty()
    flock.empty()
    pigs.empty()
    standby.empty()
    
    msg = Float64()
    msg.data = 1.0

    colourMsg = ColorRGBA()
    
    
    for agent in agents:
        if(agent.role == "dog"):
            pack.add(agent)
            msg.data = robot_dog_speed
            agent.maxSpeedPublisher.pub.publish(msg)
            #print("setting agent ", str(agent.id), " max speed to ", str(robot_dog_speed), " - DOG")

            # TODO - remove setting agent colour at start
            colourMsg.r = 0.3
            colourMsg.g = 0.3
            colourMsg.b = 1.0
            agent.colourPublisher.pub.publish(colourMsg)

        if(agent.role == "sheep"):
            flock.add(agent)
            msg.data = robot_sheep_speed
            agent.maxSpeedPublisher.pub.publish(msg)
            #print("setting agent ", str(agent.id), " max speed to ", str(robot_dog_speed), " - SHEEP")
            # TODO - remove setting agent colour at start
            colourMsg.r = 1.0
            colourMsg.g = 1.0
            colourMsg.b = 1.0
            agent.colourPublisher.pub.publish(colourMsg)
            print("setting agent ", str(agent.id), " colour to ", str(colourMsg), " - SHEEP")

        if(agent.role == "pig"):
            pigs.add(agent)
            msg.data = robot_pig_speed
            agent.maxSpeedPublisher.pub.publish(msg)
           # print("setting agent ", str(agent.id), " max speed to ", str(robot_dog_speed), " - PIG")

            # TODO - remove setting agent colour at start
            colourMsg.r = 1.0
            colourMsg.g = 0.0
            colourMsg.b = 0.0
            agent.colourPublisher.pub.publish(colourMsg)
            print("setting agent ", str(agent.id), " colour to ", str(colourMsg), " - PIG")
        if(agent.role == "standby"):
            standby.add(agent)
            msg.data = robot_standby_speed
            agent.maxSpeedPublisher.pub.publish(msg)
          #  print("setting agent ", str(agent.id), " max speed to ", str(robot_dog_speed), " - STANDBY")

            # TODO - remove setting agent colour at start
            colourMsg.r = 0.0
            colourMsg.g = 1.0
            colourMsg.b = 0.0
            agent.colourPublisher.pub.publish(colourMsg)
            print("setting agent ", str(agent.id), " colour to ", str(colourMsg), " - STANDBY")

        



def main(show_empowerment=False):

    rclpy.init(args=None)



    global screen
    global state
    global postedUpdates
    global sendUpdates

    global robot_dog_speed
    global robot_pig_speed
    global robot_sheep_speed
    global robot_standby_speed

    robot_dog_speed = cfg['robot_dog_max_speed']
    robot_pig_speed = cfg['robot_pig_max_speed']
    robot_sheep_speed = cfg['robot_sheep_max_speed']
    robot_standby_speed = cfg['robot_standby_max_speed']

    end_game = False

    pygame.init()

    screen = pygame.display.set_mode([cfg['world_width'] + 80,cfg['world_height']])

    agent_id = 0


    commandListenerTopicName = "/controller/command"
    dispatchListenerTopicName = "/controller/dispatch"
    agentListenerTopicName = "/global/robots/added"
    # define the state command listener:
    commandListener = CommandListener(commandListenerTopicName, CommandListenerCallback) 

    # define the dispatch listener
    dispatchListener = CommandListener(dispatchListenerTopicName, DispatchCallback)

    # define the add agent listener
    agentListener = AgentListener(agentListenerTopicName, add_agent_callback)



    ## add n agents
    for i in range (0):
        # adds agents with role agent (un initialised)
        #camWidth, camHeight, worldWidth, worldHeight
        agent_id = add_agent(agents = agents, position = np.array([30 + (i*15), 20]), cfg = cfg, id = agent_id, camWidth = cfg['cam_width'], camHeight = cfg['cam_height'], worldWidth = cfg['world_width'], worldHeight = cfg['world_height'], screen=screen)
    


    # put all robots into standby.
    SetAllAgentRolesToStandby()

    SortAgentsByRole()

    


    
    
    
    
    while (not end_game):

        pygame.display.update()
        time.sleep(0.01)

        # poll for poses for each agent
        for agent in agents:
            agent.RosUpdate()
        

        # look out for commands send to this script
        rclpy.spin_once(commandListener, timeout_sec=0.01)
        # check if user wants to dispatch/recall dogs
        rclpy.spin_once(dispatchListener, timeout_sec=0.01)
        # check if we have add/remove agents
        rclpy.spin_once(agentListener, timeout_sec=0.01)

        # draw world 
        DrawWorld(cfg=cfg)

        if(postedUpdates > 0):
            sendUpdates = True
            postedUpdates = 0
        else:
            sendUpdates = False

        
        # if we have recieved an update from one of the agents, then proceed with simulation
        if(sendUpdates or not cfg['event_driven']):
            if(state == "standby_setup_loop"):
                StandbySetupUpdateTimestep(agents = standby, cfg=cfg)
                for sheep in flock:
                    sheep.DrawSelf(screen = screen)

                for pig in pigs:
                    pig.DrawSelf(screen = screen)
                
                for dog in pack:
                    dog.DrawSelf(screen = screen)

            if(state == "setup_start"):

                # we assume all agents are in standby position. Set all agents to standby:
                SetAllAgentRolesToStandby()
                SortAgentsByRole()
                # look at experiment config file
                # choose first n standby agents as sheep
                sheepPositions = cfg['initial_sheep_positions']
                n = len(sheepPositions)
                i = 0

                for agent in standby:
                    agent.role = "sheep"
                    i += 1
                    if(i >= n):
                        break
                
                # re order the groups of agents
                SortAgentsByRole()

                dogPositions = cfg['initial_dog_positions']
                n = len(dogPositions)
                i = 0
                for agent in standby:
                    agent.role = "dog"
                    i += 1
                    if(i >= n):
                        break

                # re order the groups of agents
                SortAgentsByRole()

                # calculate the amount of reserve dogs left
                reserveDogs = (cfg['max_number_of_dogs'] - len(pack))

                # subtract the amount of reserve dogs from the standby agents. The remaining n should go to the pigsty
                pigAmount = len(standby) - reserveDogs

                if(pigAmount > 0):
                    # assign n pigs to the pigsty from the standby pool
                    i = 0
                    for agent in standby:
                        agent.role = "pig"
                        i += 1
                        if(i >= pigAmount):
                            break

                # re order the groups of agents
                SortAgentsByRole()

                # advance the state machine loop to the next state
                state = "sheep_setup_loop"

                

            if(state == "sheep_setup_loop"):

                # get the list of positions for sheep to move to
                sheepPositions = cfg['initial_sheep_positions']
                i = 0
                for sheep in flock:
                    pos = sheepPositions[i]
                    MoveToPointDecision(agent = sheep, movePos = np.array(pos), cfg = cfg)
                # sheep.MoveToPoint(point_x = point_x, point_y = point_y, screen = screen, agents = agents, cfg = cfg)
                    i += 1
                
                for dog in pack:
                    dog.DrawSelf(screen = screen)

                for agent in standby:
                    agent.DrawSelf(screen = screen)
                
                for pig in pigs:
                    pig.DrawSelf(screen = screen)
            
            if(state == "dog_setup_loop"):
                # get the list of positions for starting dogs to move to
                dogPositions = cfg['initial_dog_positions']
                i = 0
                for dog in pack:
                    pos = dogPositions[i]
                    MoveToPointDecision(agent = dog, movePos = np.array(pos), cfg = cfg)
                    i += 1
                for sheep in flock:
                    sheep.DrawSelf(screen = screen)

                for agent in standby:
                    agent.DrawSelf(screen = screen)
                
                for pig in pigs:
                    pig.DrawSelf(screen = screen)
            
            if(state == "pig_setup_loop"):
                # get the list of positions for the unused pigs to move to
                pigPositions = cfg['pigsty_positions']
                i = 0
                for pig in pigs:
                    pos = pigPositions[i]
                    MoveToPointDecision(agent = pig, movePos = np.array(pos), cfg = cfg)
                    i += 1

                for sheep in flock:
                    sheep.DrawSelf(screen = screen)

                for agent in standby:
                    agent.DrawSelf(screen = screen)
                
                for dog in pack:
                    dog.DrawSelf(screen = screen)

            elif(state == "experiment"):
                ExperimentUpdateTimestep(agents = agents, pack = pack, flock=flock,  cfg=cfg)
                StandbySetupUpdateTimestep(agents = standby, cfg=cfg)
                for pig in pigs:
                    pig.DrawSelf(screen = screen)
                    pig.HaltAgent(screen)
            else:
                for agent in agents:
                    agent.DrawSelf(screen)
        # if we are not cleared to send updates due to no incoming data (if in event driven mode), then draw all agents
        else:
            
            for agent in agents:
                agent.DrawSelf(screen)


            



        

        







#end function

if __name__ == '__main__':
    main()
        



