import xml.dom.minidom
import xml.etree.ElementTree as ET

import ros_topology as rsg

def parse_file(filename):
    """ Parses an xml configuration file and returns the encoded ros graph
    as a topology.
    """
    root = ET.parse(filename).getroot()
    newgraph = rsg.RosSystemGraph()
    for xmlNode in root.findall("node"):
        # Create new node
        node = rsg.Node(newgraph)
        node.name = xmlNode.attrib["name"].strip()
        node.location = xmlNode.attrib["location"].strip()
        node.pid = xmlNode.attrib["pid"].strip()
        print "Adding Node", node.name

        # Setup Publishers and Subscribers
        # Before we can create a connection, we need to add the topic
        # to our SystemGraph.
        for xmlTopic in xmlNode.find("topics"):
            name = xmlTopic.attrib["name"]
            msgType = xmlTopic.attrib["type"]
            # Grab existing topic if available
            topic = None
            if (name, msgType) not in newgraph.topics.keys():
                print "Adding Topic ", name, msgType
                topic = rsg.Topic(newgraph)
                topic.name = name
                topic.msgType = msgType
            else:
                topic = newgraph.topics[(name, msgType)]

            if xmlTopic.tag == "publishes":
                print "Adding publisher", node.name, topic.name
                conn = rsg.Publisher(newgraph, node, topic)
            if xmlTopic.tag == "subscribes":
                print "Adding subscriber", node.name, topic.name
                conn = rsg.Subscriber(newgraph, node, topic)
            conn.bandwidth = int(xmlTopic.attrib["bw"].strip())
            conn.freq = int(xmlTopic.attrib["freq"].strip())
    return newgraph


