#!/usr/bin/env python
import rospy
import time
import copy

import rosgraph 
import rosnode
import xmlrpclib # used to get pid
from rqt_graphprofiler.msg import *

class SimpleUpdater(object):
    def __init__(self):
        self._master = rosgraph.Master('simple_grapher')
        self._publisher = rospy.Publisher('/rosgraph',RosGraph, queue_size = 10)

    def update(self):
        nodes = dict()
        topics = dict()

        # Build a dictionary of topic.msg, by name
        alltopics = self._master.getTopicTypes()
        for name, type_ in alltopics:
            topic = Topic()
            topic.name = name
            topic.type = type_
            topics[name] = topic


        # Build a dictionary of node.msg, by name
        allnodenames = rosnode.get_node_names()
        for name in allnodenames:
            node = Node()
            node.name = name
            try:
                # Get the uri
                node.uri = self._master.lookupNode(name)
            except rosgraph.masterapi.MasterError:
                print "WARNING: MasterAPI Error"
            try:
                # The the pid
                node_api = rosnode.get_api_uri(rospy.get_master(), name)
                code, msg, pid = xmlrpclib.ServerProxy(node_api[2]).getPid('/NODEINFO')
                node.pid = pid
            except xmlrpclib.socket.error:
                print "WANRING: XML RPC ERROR"
                return
            # TODO: It would be nice to get the 'type' and 'package' information too
            nodes[name] = node

        # put topics into nodes as publishers and subscribers
        # TODO: We might need to copy topics into the nodes, in case serialization 
        # does not happen correctly
        systemstate = self._master.getSystemState()
        for topic_name, publisher_list in systemstate[0]:
            for publishername in publisher_list:
                if publishername in nodes:
                    nodes[publishername].publishes.append(copy.deepcopy(topics[topic_name]))
        for topic_name, subscriber_list in systemstate[1]:
            for subscribername in subscriber_list:
                if subscribername in nodes:
                    nodes[subscribername].subscribes.append(copy.deepcopy(topics[topic_name]))

        graph = RosGraph()
        graph.nodes = nodes.values()
        self._publisher.publish(graph)
        
            


if __name__ == "__main__":
    rospy.init_node('simple_grapher')
    updater = SimpleUpdater()
    r = rospy.Rate(1)
    try:
        while not rospy.is_shutdown():
            updater.update()
            print "spin"
            r.sleep()
    except rospy.ROSInterruptException:
        pass
