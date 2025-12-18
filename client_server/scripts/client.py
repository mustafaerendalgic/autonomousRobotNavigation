#!/usr/bin/env python3

import rospy
from std_srvs.srv import SetBool, SetBoolRequest

# Node setup
rospy.init_node("client")

# Define my client and wait for the service
client = rospy.ServiceProxy("test_service", SetBool)
client.wait_for_service()

# Create the request message
request = SetBoolRequest()
request.data = True

# Receive the response and store it
response = client(request)

print(response)
