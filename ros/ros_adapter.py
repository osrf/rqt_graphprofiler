import rosgraph
import rosnode
QUIET_NAMES = ['/rosout','/tf']

from diarc.base_adapter import *
from ros_topology import *
from diarc.view import BlockItemAttributes
from diarc.view import BandItemAttributes
from diarc.view import SnapItemAttributes
# from diarc.view_attributes import *


class RosAdapter(BaseAdapter):
    """ Implements the Adapter interface for the View and provides hooks for 
    populating and implementing the ros specific version of the topology.
    """

    def __init__(self,view):
        super(RosAdapter,self).__init__(RosSystemGraph(),view)
        self._topology.hide_disconnected_snaps = True
        self._master = rosgraph.Master('/RosSystemGraph')

    def get_block_item_attributes(self, block_index):
        """ Overloads the BaseAdapters stock implementation of this method """
        block = self._topology.blocks[block_index]
        attrs = BlockItemAttributes()
        attrs.bgcolor = "white"
        attrs.border_color = "red"
        attrs.border_width = 0
        attrs.label = block.vertex.name
        attrs.label_color = "red"
        attrs.spacerwidth = 20
        attrs.draw_debug = True
        return attrs

    def get_band_item_attributes(self, band_altitude):
        """ Overloads the BaseAdapters stock implementation of this method """
        band = self._topology.bands[band_altitude]
        attrs = BandItemAttributes()
        attrs.bgcolor = "white"
        attrs.border_color = "red"
        attrs.label = band.edge.name
        attrs.label_color = "red"
        attrs.width = 15
        attrs.draw_debug = True
        return attrs

    def get_snap_item_attributes(self, snapkey):
        """ Default method for providing some stock settings for snaps """
        attrs = SnapItemAttributes()
        attrs.bgcolor = "white"
        attrs.border_color = "red"
        attrs.border_width = 0
        attrs.label = ""
        attrs.label_color = "red"
        attrs.width = 20
        attrs.draw_debug = True
        return attrs

    def update_model(self):
        """ query the ros master for information about the state of the system """
        # Query master and compile a list of all published topics and their types
        allCurrentTopics = self._master.getPublishedTopics('/')
        allCurrentTopicNames = [x[0] for x in allCurrentTopics]
        # Get all the topics we currently know about
        rsgTopics = self._topology.topics

        # Remove any topics from Ros System Graph not currently known to master
        for topic in rsgTopics.values():
            if topic.name not in allCurrentTopicNames:
                print "Removing Topic",topic.name, "not found in ",allCurrentTopicNames
                topic.release()

        # Add any topics not currently in the Ros System Graph
        for topicName, topicType in allCurrentTopics:
            if topicName not in rsgTopics: # and topicName not in QUIET_NAMES:
                topic = Topic(self._topology,topicName,topicType)

        # Compile a list of node names
        allCurrentNodes = rosnode.get_node_names()
        # Get all nodes we currently know about
        rsgNodes = self._topology.nodes

        # Remove any nodes from RosSystemGraph not currently known to master
        for node in rsgNodes.values():
            if node.name not in allCurrentNodes:
                print "Removing Node",node.name, "not found in ",allCurrentNodes
                node.release()

        # Add any nodes not currently in the Ros System Graph
        for name in allCurrentNodes:
            if name not in rsgNodes: # and name not in QUIET_NAMES:
                node = Node(self._topology,name)
                try:
                    node.location = self._master.lookupNode(name)
                except socket.error:
                    raise Exception("Unable to communicate with master!")

        # Check for added or removed connections
        systemState = self._master.getSystemState()
        # Process publishers
        for topicName, publishersList in systemState[0]:
            if topicName in QUIET_NAMES: 
                continue
            rsgPublishers = self._topology.topics[topicName].publishers
            # Remove publishers that don't exist anymore
            for publisher in rsgPublishers:
                if publisher.node.name not in publishersList:
                    publisher.release()
            # Add publishers taht are not yet in the RosSystemGraph
            for nodeName in publishersList:
                if nodeName not in [pub.node.name for pub in rsgPublishers]:
                    publisher = Publisher(self._topology,self._topology.nodes[nodeName],self._topology.topics[topicName])

        # Process subscribers
        for topicName, subscribersList in systemState[1]:
            if topicName in QUIET_NAMES: 
                continue
            try:
                rsgSubscribers = self._topology.topics[topicName].subscribers
            except:
                print topicName,"not found in"
                continue
            # Remove subscribers that don't exist anymore
            for subscriber in rsgSubscribers:
                if subscriber.node.name not in subscribersList:
                    subscriber.release()
            # Add subscriber taht are not yet in the RosSystemGraph
            for nodeName in subscribersList:
                if nodeName not in [sub.node.name for sub in rsgSubscribers]:
                    subscriber = Subscriber(self._topology,self._topology.nodes[nodeName],self._topology.topics[topicName])

        self._update_view()




