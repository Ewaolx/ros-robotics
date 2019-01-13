import rosbag
bag = rosbag.Bag('test.bag')

# Each iteration below is one <strong>msg</strong> from a speicific <strong>topic,</strong> in the order they were recorded w.r.t time. Hence you check which topic a msg in a iteration comes from using simple if statements and process the msg accordingly.
for topic, msg, t in bag.read_messages():
   if(topic == "Topic 1"):
	print topic  # Prints "Topic 1"
	print msg    # Prints the message from topic "Topic 1"
	# More Code to handle msg of "Topic 1"
   if(topic == "Topic 2"):
	print topic  # Prints "Topic 2"
	print msg    # Prints the message from topic "Topic 2"
	# More Code to handle msg of "Topic 2"
bag.close()
