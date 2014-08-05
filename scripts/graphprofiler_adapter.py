from threading import *
from base_adapter import *
from ros_topology import *
from view_attributes import *
import copy

import rospy
from rqt_graphprofiler.msg import *

QUIET_NAMES = ['/rosout','/tf']

class GraphProfileAdapter(BaseAdapter):
    """ Implementes the Adapter interface for the View and provides hooks for
    populating and implementing the ros specific version of the topology
    """

    def __init__(self,view):
        super(GraphProfileAdapter,self).__init__(RosSystemGraph(),view)
        self._topology.hide_disconnected_snaps = True
        self.host_profile_subscriber = rospy.Subscriber('/graphprofile', HostProfile, self.ros_callback)

        self._host_profiles = dict() # hostname, most recent HostProfile()
        self._host_profiles_lock = Lock()

    def get_block_item_attributes(self, block_index):
        """ Overloads the BaseAdapters stock implementation of this method """
        block = self._topology.blocks[block_index]
        attrs = BlockItemViewAttributes()
        attrs.bgcolor = None
        attrs.border_color = "red"
        attrs.label = block.vertex.name
        attrs.label_rotation = -90
        attrs.label_color = "red"
        attrs.spacerwidth = 20
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
        attrs.bgcolor = None
        attrs.border_color = "red"
        attrs.label = ""
        attrs.label_color = "red"
        attrs.width = 20
        return attrs


    def ros_callback(self, data):
        """ Saves the most recent HostProfile message from each host """
        if not isinstance(data,HostProfile):
            rospy.logerr("Wrong Data Type!")
            return
        with self._host_profiles_lock:
            self._host_profiles[data.hostname] = data

    def update_model(self):
        """ Updates the Topology model with information about the state of the system.

        This takes logged HostProfile information and determines if it is outdated 
        (such as if the Host disappeared) or not, and combines it with other recent host
        data.
        """

        time_now = rospy.get_rostime()

        # Make a copy of the latest data for working with to prevent blocking on the ros callback
        hostprofiles = None
        with self._host_profiles_lock:
            hostprofiles = copy.deepcopy(self._host_profiles)
            
        # Create a combined list of NodeProfiles
        nodeprofiles = dict() # name, NodeProfile
        for hostname in hostprofiles:
            host = hostprofiles[hostname]
            # Discard if the data is too old
            latency = time_now - host.window_stop 
            window = host.window_stop - host.window_start
            if latency > window:
                rospy.logerr("Data from '%s' too old"%hostname)
                continue
            for node in host.nodes:
                nodeprofiles[node.name] = node
        
        # Compile list of all topics from nodeprofiles
        allCurrentTopics = set()
        for node in nodeprofiles.values():
            for topic in node.published_topics:
                allCurrentTopics.add(tuple([topic.topic,topic.type]))
            for topic in node.subscribed_topics:
                allCurrentTopics.add(tuple([topic.topic,topic.type]))
        allCurrentTopics = list(allCurrentTopics)

        # Remove any topics from Ros System Graph not currently known by the profiling system
        rsgTopics = self._topology.topics
        allCurrentTopicNames = [x[0] for x in allCurrentTopics]
        for topic in rsgTopics.values():
            if topic.name not in allCurrentTopicNames:
                print "Removing Topic",topic.name, "not found in ",allCurrentTopicNames
                topic.release()

        # Add any topics not currently in the Ros System Graph
        for topicName, topicType in allCurrentTopics:
            if topicName not in rsgTopics: # and topicName not in QUIET_NAMES:
                topic = Topic(self._topology, topicName, topicType)

        # Get all the nodes we currently know about
        rsgNodes = self._topology.nodes

        # Remove any nodes from RosSystemGraph not currently known to master
        for node in rsgNodes.values():
            if node.name not in nodeprofiles:
                print "Removing Node",node.name, "not found in ",nodeprofiles.keys()
                node.release()
                # TODO: Remove any of the nodes publishers or subscribers now

        # Add any nodes not currently in the Ros System Graph
        for name in nodeprofiles.keys():
            rsg_node = None
            if name not in rsgNodes: # and name not in QUIET_NAMES:
                rsg_node = Node(self._topology,name)
                rsg_node.location = nodeprofiles[name].uri
            else:
                rsg_node = self._topology.nodes[name]
            #TODO: Add node statistics
             
            # Add and remove publishers for this node only
            # Compile two dictionaries, one of existing topics and one of the most recently
            # reported topics. Remove existing publishers that are not mentioned in the 
            # current list, add publishers that not in the existing list but in the current list,
            # and update publishers that occur in both lists.
            existing_rsg_node_pub_topics = dict([(publisher.topic.name, publisher) for publisher in rsg_node.publishers])
            current_node_prof_pub_topics = dict([(topic.topic, topic) for topic in nodeprofiles[name].published_topics])
            for existing_topic_name in existing_rsg_node_pub_topics.keys():
                if existing_topic_name not in current_node_prof_pub_topics.keys():
                    existing_rsg_node_pub_topics[existing_topic_name].release()
            for current_topic_name in current_node_prof_pub_topics.keys():
                publisher = None
                if current_topic_name not in existing_rsg_node_pub_topics.keys():
                    publisher = Publisher(self._topology, rsg_node, self._topology.topics[current_topic_name])
                else:
                    publisher = current_node_prof_pub_topics[current_topic_name]
                # TODO: Add publisher statistics

            # Add and remove subscribers for this node only.
            # This follows the same patteren as the publishers above.
            existing_rsg_node_sub_topics = dict([(subscriber.topic.name, subscriber) for subscriber in rsg_node.subscribers])
            current_node_prof_sub_topics = dict([(topic.topic, topic) for topic in nodeprofiles[name].subscribed_topics])
            for existing_topic_name in existing_rsg_node_sub_topics.keys():
                if existing_topic_name not in current_node_prof_sub_topics.keys():
                    existing_rsg_node_sub_topics[existing_topic_name].release()
            for current_topic_name in current_node_prof_sub_topics.keys():
                subscriber = None
                if current_topic_name not in existing_rsg_node_sub_topics.keys():
                    subscriber = Subscriber(self._topology, rsg_node, self._topology.topics[current_topic_name])
                else:
                    subscriber = current_node_prof_sub_topics[current_topic_name]
                # TODO: Add subscriber statistics

        self._update_view()


