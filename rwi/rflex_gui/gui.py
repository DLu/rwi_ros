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
		wx.Frame.__init__(self, parent, id, title, (-1, -1));
		panel = wx.Panel(self, -1);
		box = wx.BoxSizer(wx.HORIZONTAL)
		self.sonar = wx.Button(panel, -1, 'Sonar')
		#self.sonar.SetText("Sonar is OFF")
		box.Add(self.sonar, 1)
		panel.SetSizer(box)
		self.Center()
	
		rospy.Subscriber("sonar_power", Bool, self.onsonar)

	def onsonar(self, data):
		if(data):
			self.sonar.SetForegroundColour('GREEN')
			#self.sonar.SetText("Sonar is ON")
		else:
			self.sonar.SetForegroundColour('BLACK')
			#self.sonar.SetText("Sonar is OFF")
		rospy.loginfo("XXXX")

if __name__ == '__main__':
	try:
		app = wx.App()
		gui = RflexGui(None, -1, "GUI")
		gui.Show()
		rospy.init_node('rflex_gui')
		
		app.MainLoop()
	except rospy.ROSInterruptException: pass
