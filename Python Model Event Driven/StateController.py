from tkinter import *
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from model.Publisher import Publisher
from std_msgs.msg import String
class Window(Frame):


# states:

# setup_start

# sheep_setup_loop

# dog_setup_loop

# pig_setup_loop

# experiment



    def __init__(self, master=None):
        Frame.__init__(self, master)        
        self.master = master

        # widget can take all window
        self.pack(fill=BOTH, expand=1)

        # create button, link it to clickExitButton()
        setupStartButton = Button(self, text="Setup Start and sheep", command=self.clickSetupStartButton)
        #sheepSetupLoopButton = Button(self, text="Sheep setup loop", command=self.clickSheepSetupLoopButton)
        dogSetupLoopButton = Button(self, text="Dog setup loop", command=self.clickDogSetupLoopButton)
        pigSetupLoopButton = Button(self, text="Pig setup loop", command=self.clickPigSetupLoopButton)
        standbySetupLoopButton = Button(self, text="Standby setup loop", command=self.standbySetupLoopButton)
        experimentButton = Button(self, text="Experiment", command=self.clickExperimentButton)

        dispatchButton = Button(self, text="Dispatch", command=self.clickDispatchButton)
        recallButton = Button(self, text="Recall", command=self.clickRecallButton)
        # place button at (0,0)
        setupStartButton.place(x=100, y=0)
     #   sheepSetupLoopButton.place(x=100, y = 50)
        dogSetupLoopButton.place(x=100, y= 50)
        pigSetupLoopButton.place(x=100, y=100)
        standbySetupLoopButton.place(x=100, y = 150)
        experimentButton.place(x=100, y=200)

        dispatchButton.place(x=100, y=250)
        recallButton.place(x=200, y=250)
        # define command publisher
        commandListenerTopicName = "/controller/command"
        dispatchListenerTopicName = "/controller/dispatch"
        self.statePublisher = Publisher(commandListenerTopicName) 
        self.dispatchPublisher = Publisher(dispatchListenerTopicName)

    def clickSetupStartButton(self):

        msg = String()
        msg.data = "setup_start"

        
        self.statePublisher.pub.publish(msg)


        print(msg)
    def clickSheepSetupLoopButton(self):

        msg = String()
        msg.data = "sheep_setup_loop"

        
        self.statePublisher.pub.publish(msg)

        print(msg)

    def clickDogSetupLoopButton(self):

        msg = String()
        msg.data = "dog_setup_loop"

        
        self.statePublisher.pub.publish(msg)

        print(msg)

    def clickPigSetupLoopButton(self):

        msg = String()
        msg.data = "pig_setup_loop"

        
        self.statePublisher.pub.publish(msg)

        print(msg)

    def standbySetupLoopButton(self):

        msg = String()
        msg.data = "standby_setup_loop"
        self.statePublisher.pub.publish(msg)
        print(msg)

    def clickExperimentButton(self):

        msg = String()
        msg.data = "experiment"

        
        self.statePublisher.pub.publish(msg)

        print(msg)

    def clickDispatchButton(self):

        msg = String()
        msg.data = "dispatch"

        
        self.dispatchPublisher.pub.publish(msg)

        print(msg)

    def clickRecallButton(self):

        msg = String()
        msg.data = "recall"

        
        self.dispatchPublisher.pub.publish(msg)

        print(msg)

        


rclpy.init(args=None)

root = Tk()
app = Window(root)
root.wm_title("Tkinter button")
root.geometry("320x320")
root.mainloop()

    
