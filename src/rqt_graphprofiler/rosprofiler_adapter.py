from threading import *
from diarc import topology
from diarc.base_adapter import *
from diarc.view_attributes import *
import copy

import rospy
import ros_topology as rsg
from rosgraph_msgs.msg import *
from ros_topology_msgs.msg import *
from ros_statistics_msgs.msg import *


class ROSProfileAdapter(BaseAdapter):
    """ Implementes the Adapter interface for the View and provides hooks for
    populating and implementing the ros specific version of the topology.
    Subscribes to /statistics, /node_statistics, /host_statistics, and /topology.
    Publishes this combined information as /profile
    """

    def __init__(self,view):
        super(ROSProfileAdapter,self).__init__(rsg.RosSystemGraph(),view)
        self._topology.hide_disconnected_snaps = True

        self._TOPIC_QUIET_LIST = list()
        self._NODE_QUIET_LIST = list()

        # Data Buffers
        self._last_topology_received = Graph()

        # Callbacks
        self.node_statistics_subscriber = rospy.Subscriber('/node_statistics', NodeStatistics, self._node_statistics_callback)
        self.topic_statistics_subscriber = rospy.Subscriber('/statistics', TopicStatistics, self._topic_statistics_callback)
        self.host_statistics_subscriber = rospy.Subscriber('/host_statistics', HostStatistics, self._host_statistics_callback)
        self.topology_subscriber = rospy.Subscriber('/topology', Graph, self._topology_callback)
        self._lock = Lock()

    def set_topic_quiet_list(self, topic_names):
        rospy.loginfo("Updating topic quiet list to %r" % topic_names)
        self._TOPIC_QUIET_LIST = copy.copy(topic_names)

    def get_topic_quiet_list(self):
        return copy.copy(self._TOPIC_QUIET_LIST)

    def set_node_quiet_list(self, node_names):
        self._NODE_QUIET_LIST = copy.copy(node_names)

    def get_node_quiet_list(self):
        return copy.copy(self._NODE_QUIET_LIST)

    def _node_statistics_callback(self, data):
        pass

    def _topic_statistics_callback(self, data):
        pass

    def _host_statistics_callback(self, data):
        pass

    def _topology_callback(self, data):
        self._last_topology_received = copy.copy(data)
        self._topology_update()

    def _topology_update(self):
        """ Updates the model with current topology information """
        data = self._last_topology_received

        # Remove any topics from Ros System Graph not currently known by the profiling system
        rsgTopics = self._topology.topics
        allCurrentTopicNames = [t.name for t in data.topics]
        for topic in rsgTopics.values():
            if topic.name in self._TOPIC_QUIET_LIST:
                print "Removing Topic",topic.name, "found in quiet list"
                topic.release()
            elif topic.name not in allCurrentTopicNames:
                print "Removing Topic",topic.name, "not found in ",allCurrentTopicNames
                topic.release()

        # Add any topics not currently in the Ros System Graph
        for topic in data.topics:
            if topic.name not in rsgTopics and topic.name not in self._TOPIC_QUIET_LIST:
                topic = rsg.Topic(self._topology, topic.name, topic.type)
        
        # Get all the nodes we currently know about
        rsgNodes = self._topology.nodes

        # Remove any nodes from RosSystemGraph not currently known to master
        allCurrentNodeNames = [n.name for n in data.nodes]
        for node in rsgNodes.values():
            if node.name in self._NODE_QUIET_LIST:
                print "Removing node",node.name,"found on quiet list"
                node.release()
            elif node.name not in allCurrentNodeNames:
                print "Removing Node",node.name, "not found in ",allCurrentNodeNames
                node.release()
                # TODO: Remove any of the nodes publishers or subscribers now

        # Add any nodes not currently in the Ros System Graph
        for node in data.nodes:
            # Skip any nodes that are on the quiet list
            if node.name in self._NODE_QUIET_LIST:
                continue
            rsg_node = None
            if node.name not in rsgNodes: # and name not in QUIET_NAMES:
                rsg_node = rsg.Node(self._topology, node.name)
                rsg_node.location = node.uri
            else:
                rsg_node = self._topology.nodes[node.name]
                if not rsg_node.location == node.uri:
                    rospy.logerr("rsg_node and data.node uri's do not match for name %s"%node.name)

            # Filter published and subscribed topics we are ignoring
            publishes_list = [p for p in node.publishes if p not in self._TOPIC_QUIET_LIST]
            subscribes_list = [s for s in node.subscribes if s not in self._TOPIC_QUIET_LIST]
             
            # Add and remove publishers for this node only
            # Compile two dictionaries, one of existing topics and one of the most recently
            # reported topics. Remove existing publishers that are not mentioned in the 
            # current list, add publishers that not in the existing list but in the current list,
            # and update publishers that occur in both lists.
            existing_rsg_node_pub_topics = dict([(publisher.topic.name, publisher) for publisher in rsg_node.publishers])
            current_node_prof_pub_topics = dict([(topic_name, self._topology.topics[topic_name]) for topic_name in publishes_list])
            for existing_topic_name in existing_rsg_node_pub_topics.keys():
                # Remove Publisher
                if existing_topic_name not in current_node_prof_pub_topics.keys():
                    existing_rsg_node_pub_topics[existing_topic_name].release()
            for current_topic_name in current_node_prof_pub_topics.keys():
                # Add Publisher
                if current_topic_name not in existing_rsg_node_pub_topics.keys():
                    publisher = rsg.Publisher(self._topology, rsg_node, current_node_prof_pub_topics[current_topic_name])

            # Add and remove subscribers for this node only.
            # This follows the same patteren as the publishers above.
            existing_rsg_node_sub_topics = dict([(subscriber.topic.name, subscriber) for subscriber in rsg_node.subscribers])
            current_node_prof_sub_topics = dict([(topic_name, self._topology.topics[topic_name]) for topic_name in subscribes_list])
            for existing_topic_name in existing_rsg_node_sub_topics.keys():
                # Remove Subscriber
                if existing_topic_name not in current_node_prof_sub_topics.keys():
                    existing_rsg_node_sub_topics[existing_topic_name].release()
            for current_topic_name in current_node_prof_sub_topics.keys():
                # Add Subscriber
                if current_topic_name not in existing_rsg_node_sub_topics.keys():
                    subscriber = rsg.Subscriber(self._topology, rsg_node, current_node_prof_sub_topics[current_topic_name])

        self._update_view()


    def get_block_item_attributes(self, block_index):
        """ Overloads the BaseAdapters stock implementation of this method """
        block = self._topology.blocks[block_index]
        attrs = BlockItemViewAttributes()
        attrs.bgcolor = None
        attrs.border_color = "black"
        attrs.border_width = 5
        attrs.label = block.vertex.name
        attrs.label_rotation = -90
        attrs.label_color = "black"
#         attrs.spacerwidth = block.vertex.
        attrs.spacerwidth = 30
        return attrs

    def get_band_item_attributes(self, band_altitude):
        """ Overloads the BaseAdapters stock implementation of this method """
        band = self._topology.bands[band_altitude]
        attrs = BandItemViewAttributes()
        attrs.bgcolor = "white"
        attrs.border_color = "red"
        attrs.label = band.edge.name
        attrs.label_color = "red"
        attrs.width = 15
        return attrs

    def get_snap_item_attributes(self, snapkey):
        """ Default method for providing some stock settings for snaps """
        attrs = SnapItemViewAttributes()
        attrs.bgcolor = "darkCyan" if 'c' in snapkey else "green"
        attrs.border_color = "darkBlue" if 'c' in snapkey else "darkGreen"
        attrs.border_width = 1
        attrs.label = ""
        attrs.label_color = "white"
        attrs.width = 20
        return attrs

    

