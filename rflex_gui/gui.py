#!/usr/bin/python

import roslib; roslib.load_manifest('rflex_gui')
import rospy
import wx
from time import time
from std_msgs.msg import Bool
from std_msgs.msg import Float32

#    pub = rospy.Publisher('chatter', String)
#        pub.publish(String(str))

class RflexGui(wx.Frame):
	def __init__(self, parent, id, title):
		self.sonar_state = None
		self.brake_state = None
		self.volts = 0.0
		
		self.last_update = 0.0
	
		wx.Frame.__init__(self, parent, id, title, (-1, -1));
		panel = wx.Panel(self, wx.ID_ANY);
		box = wx.BoxSizer(wx.VERTICAL)
		
		### Buttons ###
		self.sonar = wx.Button(panel, 1, 'Sonar')
		self.brake = wx.Button(panel, 2, 'Brake')
		
		### Text ###
		self.voltage = wx.TextCtrl(panel, 3, '00.00V')
		self.voltage.SetEditable(False)

		wx.EVT_BUTTON(self, 1, self.change_sonar_state)
		wx.EVT_BUTTON(self, 2, self.change_brake_state)

		box.Add(self.sonar, 0, wx.EXPAND)
		box.Add(self.brake, 0, wx.EXPAND)
		box.Add(self.voltage, 0, wx.ALL, 12)
		
		panel.SetSizer(box)
		self.Center()
		
		self.sonar_pub = rospy.Publisher('/b21/cmd_sonar_power', Bool)
		self.brake_pub = rospy.Publisher('/b21/cmd_brake_power', Bool)
		
		box.Fit(self)
		
	def change_sonar_state(self, event):
		rospy.loginfo("Changing sonar state to %s" % (not self.sonar_state))
		self.sonar_pub.publish(not self.sonar_state)
		
	def change_brake_state(self, event):
		rospy.loginfo("Changing brake state to %s" % (not self.brake_state))
		self.brake_pub.publish(not self.brake_state)

	def onsonar(self, data):
		if(self.sonar_state != data.data):
			self.sonar_state = data.data
			if data.data:
				# self.sonar.SetLabel('Sonar: ON')
				self.sonar.SetBackgroundColour('GREEN')
			else:
				# self.sonar.SetLabel('Sonar: OFF')
				self.sonar.SetBackgroundColour('RED')
			rospy.loginfo("Sonar is %s" % ("on" if data.data else "off"))

	def onbrake(self, data):
		if(self.brake_state != data.data):
			self.brake_state = data.data
			if data.data:
				# self.brake.SetLabel('Brake: ON')
				self.brake.SetBackgroundColour("RED")
			else:
				# self.brake.SetLabel('Brake: OFF')
				self.brake.SetBackgroundColour("GREEN")
			
			rospy.loginfo("Brake is %s" % ("on" if data.data else "off"))
		
	def onvoltage(self, data):
		# if self.volts != data.data:
		if True:
			self.volts = data.data;
			self.voltage.SetValue('%.2fV' % data.data)
			if data.data >= 24:
				self.voltage.SetBackgroundColour('GREEN')
			elif data.data >= 20:
				self.voltage.SetBackgroundColour('ORANGE')
			else:
				self.voltage.SetBackgroundColour('RED')
			self.last_update = time()

if __name__ == '__main__':
	try:
		app = wx.App()
		gui = RflexGui(None, -1, "rflex gui")
		gui.Show()

		rospy.init_node('rflex_gui')
		rospy.Subscriber("/b21/sonar_power", Bool, gui.onsonar)
		rospy.Subscriber("/b21/brake_power", Bool, gui.onbrake)
		rospy.Subscriber("/b21/voltage", Float32, gui.onvoltage)
		
		app.MainLoop()
	except rospy.ROSInterruptException: pass
