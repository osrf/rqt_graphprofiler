# Copyright 2014 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from __future__ import print_function
import xml.dom.minidom
import xml.etree.ElementTree as ET

from diarc.base_adapter import BaseAdapter
from diarc.view import BlockItemAttributes
from diarc.view import BandItemAttributes
from diarc.view import SnapItemAttributes
import ros_topology as rsg


class ColorMapper(object):
    def __init__(self):
        self._choices = list()
        # Reds
        self._choices.extend(["IndianRed", "DarkSalmon", "Crimson"])
        # Pinks
        self._choices.extend(["HotPink", "DeepPink"])
        # Oranges
        self._choices.extend(["Coral", "OrangeRed", "DarkOrange"])
        # Yellows
        self._choices.extend(["Gold", "DarkKhaki"])
        # Purples
        self._choices.extend(["Thistle", "Orchid", "MediumPurple", "DarkOrchid", "Purple", "Indigo", "DarkSlateBlue"])
        # Greens
        self._choices.extend(["LawnGreen", "LimeGreen", "MediumSeaGreen", "ForestGreen", "OliveDrab", "Olive", "DarkOliveGreen", "DarkCyan"])
        # Blues
        self._choices.extend(["PaleTurquoise", "Turquoise", "CadetBlue", "SteelBlue", "DodgerBlue"])
        # Browns
#         self._choices.extend(["Cornsilk","Tan","RosyBrown","SandyBrown","Goldenrod","DarkGoldenrod","SaddleBrown"])
        self._used_colors = dict()

    def get_unique_color(self, name):
        if name not in self._used_colors:
            if len(self._choices) > 0:
                self._used_colors[name] = random.choice(self._choices)
                self._choices.remove(self._used_colors[name])
            else:
                self._used_colors[name] = "Gray"
        return self._used_colors[name]

    def release_unique_color(self, name):
        if name in self._used_colors:
            if not self._used_colors[name] == "Gray":
                self._choices.append(self._used_colors[name])
            self._used_colors.pop(name)
        else:
            rospy.logwarn("Unknown name mapped to color!")


class ParserAdapter(BaseAdapter):
    """ Implements the Adapter interface as a static xml parser. 
    This is run the by VisualizerWidget defined inside visualizer_plugin.py.
    It can act as an alternative to rosprofiler_adapter.
    TODO: the VisualizerWidget still needs to be changed to except this as an option.
    """

    def __init__(self, view):
        super(ParserAdapter, self).__init__(rsg.RosSystemGraph(), view)
        self._topology.hide_disconnected_snaps = True

        self._colormapper = ColorMapper()
        self._TOPIC_QUIET_LIST = list()
        self._NODE_QUIET_LIST = list()

    def set_topic_quiet_list(self, topic_names):
        rospy.loginfo("Updating topic quiet list to %r" % topic_names)
        self._TOPIC_QUIET_LIST = copy.copy(topic_names)

    def get_topic_quiet_list(self):
        return copy.copy(self._TOPIC_QUIET_LIST)

    def set_node_quiet_list(self, node_names):
        self._NODE_QUIET_LIST = copy.copy(node_names)

    def get_node_quiet_list(self):
        return copy.copy(self._NODE_QUIET_LIST)

    def enable_auto_update(self):
        """ This does nothing since the graph is not actually updating """
        pass

    def disable_auto_update(self):
        """ This does nothing since the graph is not actually updating """
        pass

    def show_disconnected_topics(self):
        self._topology.hide_disconnected_snaps = False
        self.topology_update()

    def hide_disconnected_topics(self):
        self._topology.hide_disconnected_snaps = True
        self.topology_update()

    def topology_update(self):
        """ parse the xml file """
        # TODO: It would be nice if this happened differentially, like rosprofiler_adapter
        # does. That way I could change the file and hit refresh and the graph
        # would change. But to start out just parsing it at all would be a win.
        root = ET.parse("input.xml").getroot()

        # TODO: The below code - topics show up differently with / vs without the /
#     ros = RosSystemGraph()
#     for xmlNode in root.findall("node"):
#         # Create new node
#         node = Node(ros)
#         node.name = xmlNode.attrib["name"].strip()
#         node.location = xmlNode.attrib["location"].strip()
#         node.pid = xmlNode.attrib["pid"].strip()
#         print "Adding Node",node.name
# 
#         # Setup Publishers and Subscribers
#         # Before we can create a connection, we need to add the topic
#         # to our SystemGraph. 
#         for xmlTopic in xmlNode.find("topics"):
#             name = xmlTopic.attrib["name"]
#             msgType = xmlTopic.attrib["type"]
#             # Grab existing topic if available 
#             topic = None
#             if (name,msgType) not in ros.topics.keys():
#                 print "Adding Topic ",name,msgType
#                 topic = Topic(ros)
#                 topic.name = name
#                 topic.msgType = msgType
#             else:
#                 topic = ros.topics[(name,msgType)]
#                 
#             if xmlTopic.tag == "publishes":
#                 print "Adding publisher",node.name,topic.name
#                 conn = Publisher(ros,node,topic)
#             if xmlTopic.tag == "subscribes":
#                 print "Adding subscriber",node.name,topic.name
#                 conn = Subscriber(ros,node,topic)
#             conn.bandwidth = int(xmlTopic.attrib["bw"].strip())
#             conn.freq = int(xmlTopic.attrib["freq"].strip())

    def statistics_update(self):
        """ this happens on a timer in rosprofiler_adapter, but that isn't 
        necessary for this implementation since the file won't change that often.
        Instead, we just do nothing.
        TODO: See where this method is called at in other code to make sure this
        is going to be ok.
        """
        pass


    def get_block_item_attributes(self, block_index):
        """ Overloads the BaseAdapters stock implementation of this method """
        block = self._topology.blocks[block_index]
        attrs = BlockItemAttributes()
        attrs.bgcolor = None
        attrs.border_color = "black"
        attrs.border_width = 5
        attrs.label = block.vertex.name
        attrs.tooltip_text = "Node:\t%s\nCPU:\t%d\nMEM:\t%s\nThreads:\t%d" % (block.vertex.name, block.vertex.cpu_load_mean, sizeof_fmt(block.vertex.virt_mem_mean), block.vertex.num_threads)
        attrs.label_color = "black"
#         attrs.spacerwidth = block.vertex.
        attrs.spacerwidth = 30
        return attrs

    def get_band_item_attributes(self, band_altitude):
        """ Overloads the BaseAdapters stock implementation of this method """
        band = self._topology.bands[band_altitude]
        attrs = BandItemAttributes()
        attrs.bgcolor = self._colormapper.get_unique_color(band.edge.name)
        attrs.border_color = "red"
        attrs.tooltip_text = "Topic:\t%s\nBw:\t%s/sec\nHz:\t%.1f" % (band.edge.name, sizeof_fmt(band.edge.bw), band.edge.hz)
        attrs.label = band.edge.name
        attrs.label_color = "white"
        attrs.width = 15
        return attrs

    def get_snap_item_attributes(self, snapkey):
        """ Default method for providing some stock settings for snaps """
        attrs = SnapItemAttributes()
        attrs.bgcolor = "darkCyan" if 'c' in snapkey else "green"
        attrs.border_color = "darkBlue" if 'c' in snapkey else "darkGreen"
        attrs.border_width = 1
        attrs.label = self._topology.snaps[snapkey].connection.edge.name
        attrs.label_color = "white"
        attrs.width = 20
        return attrs


def sizeof_fmt(num):
    # Taken from http://stackoverflow.com/a/1094933
    for x in ['bytes', 'KB', 'MB', 'GB', 'TB']:
        if num < 1024.0:
            return "%3.1f %s" % (num, x)
        num /= 1024.0
