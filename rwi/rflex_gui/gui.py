#!/usr/bin/python

import roslib; roslib.load_manifest('rflex_gui')
import rospy
import wx

from std_msgs.msg import Bool
from std_msgs.msg import Float32

#    pub = rospy.Publisher('chatter', String)
#        pub.publish(String(str))

class RflexGui(wx.Frame):
	def __init__(self, parent, id, title):
		self.sonar_state = False
		self.brake_state = False
	
		wx.Frame.__init__(self, parent, id, title, (-1, -1));
		panel = wx.Panel(self, -1);
		box = wx.BoxSizer(wx.VERTICAL)
		
		### Buttons ###
		self.sonar = wx.Button(panel, 1, 'Sonar')
		self.brake = wx.Button(panel, 2, 'Brake')

		wx.EVT_BUTTON(self, 1, self.change_sonar_state)
		wx.EVT_BUTTON(self, 2, self.change_brake_state)

		#self.sonar.SetText("Sonar is OFF")
		box.Add(self.sonar, 1)
		box.Add(self.brake, 2)
		panel.SetSizer(box)
		self.Center()
		
		self.sonar_pub = rospy.Publisher('cmd_sonar_power', Bool)
		self.brake_pub = rospy.Publisher('cmd_brake_power', Bool)
		
	def change_sonar_state(self, event):
		rospy.loginfo("sonar button press")
		rospy.loginfo("Changing sonar state to %s" % (not self.sonar_state))
		self.sonar_pub.publish(not self.sonar_state)
		
	def change_brake_state(self, event):
		rospy.loginfo("brake button press")
		rospy.loginfo("Changing brake state to %s" % (not self.brake_state))
		self.brake_pub.publish(not self.brake_state)

	def onsonar(self, data):
		self.sonar_state = data.data
		if data.data:
			self.sonar.SetBackgroundColour('GREEN')
		else:
			self.sonar.SetBackgroundColour('RED')

		rospy.loginfo("Sonar is %s" % ("on" if data.data else "off"))

	def onbrake(self, data):
		self.brake_state = data.data
		if data.data:
			self.brake.SetBackgroundColour("RED")
		else:
			self.brake.SetBackgroundColour("GREEN")
			
		rospy.loginfo("Brake is %s" % ("on" if data.data else "off"))

if __name__ == '__main__':
	try:
		app = wx.App()
		gui = RflexGui(None, -1, "GUI")
		gui.Show()


		rospy.init_node('rflex_gui')
		rospy.Subscriber("sonar_power", Bool, gui.onsonar)
		rospy.Subscriber("brake_power", Bool, gui.onbrake)
		
		app.MainLoop()
	except rospy.ROSInterruptException: pass
