import xml.etree.ElementTree as ET
import xml.dom.minidom
from ros_topology import *
# [db] dan@danbrooks.net
#
# Parses ros:v2 xml syntax and generates a RosSystemGraph object
#
# TODO:
# - Sort Nodes intelligently
# - give warnings about topics which are not fully connected (are not connected
#   on one side or another)
#
# BUGS:
# - Topics with / vs without / show up differently


def parseFile(filename):
    return parseTree(ET.parse(filename))

def parseTree(tree):
    root = tree.getroot()
    
    ros = RosSystemGraph()

    for xmlNode in root.findall("node"):
        # Create new node
        node = Node(ros)
        node.name = xmlNode.attrib["name"].strip()
        node.location = xmlNode.attrib["location"].strip()
        node.pid = xmlNode.attrib["pid"].strip()
        print "Adding Node",node.name

        # Setup Publishers and Subscribers
        # Before we can create a connection, we need to add the topic
        # to our SystemGraph. 
        for xmlTopic in xmlNode.find("topics"):
            name = xmlTopic.attrib["name"]
            msgType = xmlTopic.attrib["type"]
            # Grab existing topic if available 
            topic = None
            if (name,msgType) not in ros.topics.keys():
                print "Adding Topic ",name,msgType
                topic = Topic(ros)
                topic.name = name
                topic.msgType = msgType
            else:
                topic = ros.topics[(name,msgType)]
                
            if xmlTopic.tag == "publishes":
                print "Adding publisher",node.name,topic.name
                conn = Publisher(ros,node,topic)
            if xmlTopic.tag == "subscribes":
                print "Adding subscriber",node.name,topic.name
                conn = Subscriber(ros,node,topic)
            conn.bandwidth = int(xmlTopic.attrib["bw"].strip())
            conn.freq = int(xmlTopic.attrib["freq"].strip())

    return ros
