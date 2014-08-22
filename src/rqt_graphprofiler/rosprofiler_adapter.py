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

import threading
import copy
import math
import numpy as np
import random

import rospy
from ros_statistics_msgs.msg import HostStatistics
from ros_statistics_msgs.msg import NodeStatistics
# from ros_topology_msgs.msg import Connection
from ros_topology_msgs.msg import Graph
# from ros_topology_msgs.msg import Node
# from ros_topology_msgs.msg import Service
# from ros_topology_msgs.msg import Topic
from rosgraph_msgs.msg import TopicStatistics

# from diarc import topology
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


class ROSProfileAdapter(BaseAdapter):
    """ Implementes the Adapter interface for the View and provides hooks for
    populating and implementing the ros specific version of the topology.
    Subscribes to /statistics, /node_statistics, /host_statistics, and /topology.
    Publishes this combined information as /profile
    """

    def __init__(self, view):
        super(ROSProfileAdapter, self).__init__(rsg.RosSystemGraph(), view)
        self._topology.hide_disconnected_snaps = True

        self._colormapper = ColorMapper()
        # Determines whether or not to update the visualization when new data is received
        self._auto_update = True

        self._TOPIC_QUIET_LIST = list()
        self._NODE_QUIET_LIST = list()

        # Data Buffers
        # To improve accuracy, we hold onto data in the buffer for two evaluation
        # periods (length of statistics timer). So we buffer the buffer...
        self._last_topology_received = Graph()
        self._node_statistics_buffer = dict()  # name: list(NodeStatistics())
        self._host_statistics_buffer = dict()  # hostname: list(HostStatistics())
        self._topic_statistics_buffer = dict()  # hostname: list(TopicStatistics()
        self._previous_node_statistics_buffer = dict()
        self._previous_host_statistics_buffer = dict()
        self._previous_topic_statistics_buffer = dict()

        # Callbacks
        self.node_statistics_subscriber = rospy.Subscriber('/node_statistics', NodeStatistics, self._node_statistics_callback)
        self.topic_statistics_subscriber = rospy.Subscriber('/statistics', TopicStatistics, self._topic_statistics_callback)
        self.host_statistics_subscriber = rospy.Subscriber('/host_statistics', HostStatistics, self._host_statistics_callback)
        self.topology_subscriber = rospy.Subscriber('/topology', Graph, self._topology_callback)
        self._lock = threading.Lock()

        # Timers
        self._stats_timer = rospy.Timer(rospy.Duration(2.0), lambda x: self.statistics_update())

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
        """ Automatically update the visualization when information is received """
        self._auto_update = True
        self._stats_timer = rospy.Timer(rospy.Duration(2.0), lambda x: self.statistics_update())

    def disable_auto_update(self):
        """ buffer information received from ROS, but do not automatically update the visualization """
        self._auto_update = False
        self._stats_timer.shutdown()
        self._stats_timer = None

    def show_disconnected_topics(self):
        self._topology.hide_disconnected_snaps = False
        self.topology_update()

    def hide_disconnected_topics(self):
        self._topology.hide_disconnected_snaps = True
        self.topology_update()

    def _node_statistics_callback(self, data):
        """ Buffers NodeStatistics data """
#         latency = rospy.get_rostime() - data.window_stop
#         window = data.window_stop - data.window_start
#         margin = window*2 - latency
#         if margin.to_sec() > 0:
#             rospy.logerr("Data from '%s' too old by %f secs"%(data.node,-margin.to_sec()))
#             return
        # If we have not collected any data from this node yet, initialize the node's buffer
        if data.node not in self._node_statistics_buffer:
            self._node_statistics_buffer[data.node] = list()
        self._node_statistics_buffer[data.node].append(data)

    def _topic_statistics_callback(self, data):
        """ Buffers TopicStatistics data """
        # Buffer Topic Statistics Data.
        if data.topic not in self._topic_statistics_buffer:
            self._topic_statistics_buffer[data.topic] = list()
        self._topic_statistics_buffer[data.topic].append(data)

    def _host_statistics_callback(self, data):
        """ Buffers HostStatistics data """
        # This information is useful for drawing NodeStatistics information
        # in context to nodes running on other machines
        if data.hostname not in self._host_statistics_buffer:
            self._host_statistics_buffer[data.hostname] = list()
        self._host_statistics_buffer[data.hostname].append(data)

    def _topology_callback(self, data):
        self._last_topology_received = copy.copy(data)
        if self._auto_update:
            self.topology_update()

    def topology_update(self):
        """ Updates the model with current topology information """
        data = self._last_topology_received

        # Remove any topics from Ros System Graph not currently known by the profiling system
        rsgTopics = self._topology.topics
        allCurrentTopicNames = [t.name for t in data.topics]
        for topic in rsgTopics.values():
            if topic.name in self._TOPIC_QUIET_LIST:
                print("Removing Topic", topic.name, "found in quiet list")
                self._colormapper.release_unique_color(topic.name)
                topic.release()
            elif topic.name not in allCurrentTopicNames:
                print("Removing Topic", topic.name, "not found in ", allCurrentTopicNames)
                self._colormapper.release_unique_color(topic.name)
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
                print("Removing node", node.name, "found on quiet list")
                node.release()
            elif node.name not in allCurrentNodeNames:
                print("Removing Node", node.name, "not found in ", allCurrentNodeNames)
                node.release()
                # TODO: Remove any of the nodes publishers or subscribers now

        # Add any nodes not currently in the Ros System Graph
        for node in data.nodes:
            # Skip any nodes that are on the quiet list
            if node.name in self._NODE_QUIET_LIST:
                continue
            rsg_node = None
            if node.name not in rsgNodes:  # and name not in QUIET_NAMES:
                rsg_node = rsg.Node(self._topology, node.name)
                rsg_node.location = node.uri
            else:
                rsg_node = self._topology.nodes[node.name]
                if not rsg_node.location == node.uri:
                    rospy.logerr("rsg_node and data.node uri's do not match for name %s" % node.name)

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

    def statistics_update(self):
        """ Updates the model with current statistics information """
        rospy.logdebug("Updating Statistics")
        # Combine current buffers with previous buffers for evaluation
        node_statistics_buffer = dict(self._node_statistics_buffer.items() + self._previous_node_statistics_buffer.items())
        host_statistics_buffer = dict(self._host_statistics_buffer.items() + self._previous_host_statistics_buffer.items())
        topic_statistics_buffer = dict(self._topic_statistics_buffer.items() + self._previous_topic_statistics_buffer.items())

        # TODO: Requires a lock with the callback and other threads
        rsgNodes = self._topology.nodes
        for node_name, data_buffer in node_statistics_buffer.items():
            # Don't process node statistics that we do not have in our internal topology
            # (we don't have a place to store the information).
            if node_name not in rsgNodes:
                if node_name not in self._NODE_QUIET_LIST:
                    rospy.logwarn("Received Statistics Information for untracked node %s" % node_name)
                continue
            # Populate datasets for this node
            samples = list()
            num_threads = list()
            cpu_load_mean = list()
            cpu_load_std = list()
            cpu_load_max = list()
            virt_mem_mean = list()
            virt_mem_std = list()
            virt_mem_max = list()
            # TODO: Real memory
            for data in data_buffer:
                samples.append(data.samples)
                num_threads.append(data.threads)
                cpu_load_mean.append(data.cpu_load_mean)
                cpu_load_std.append(data.cpu_load_std)
                cpu_load_max.append(data.cpu_load_max)
                virt_mem_mean.append(data.virt_mem_mean)
                virt_mem_std.append(data.virt_mem_std)
                virt_mem_max.append(data.virt_mem_max)
            rsgNodes[node_name].num_threads = max(num_threads)
            rsgNodes[node_name].cpu_load_mean = np.mean(np.array(cpu_load_mean))
            rsgNodes[node_name].cpu_load_std = math.sqrt(sum(
                    [math.pow(sd, 2)/n for sd, n in zip(cpu_load_std, samples)]))
            rsgNodes[node_name].cpu_load_max = max(cpu_load_max)
            rsgNodes[node_name].virt_mem_mean = np.mean(np.array(virt_mem_mean))
            rsgNodes[node_name].virt_mem_std = math.sqrt(sum(
                    [math.pow(sd, 2)/n for sd, n in zip(virt_mem_std, samples)]))
            rsgNodes[node_name].virt_mem_max = max(virt_mem_max)

        # Process Topic Statistics Data
        # TODO: we are not currently processing all the topic data found in TopicStatistics() message
        # TODO: These are actually piecewise between individual publishers and subscribers.
        #       Eventually we want to be able to draw each connections individual contribution to the
        #       whole topic, but for now just lump it all together
        rsgTopics = self._topology.topics
        for topic_name, data_buffer in topic_statistics_buffer.items():
            # Don't process topic statistics that we do not have in our internal topology
            # (We don't have a place to store the information)
            if topic_name not in rsgTopics:
                if topic_name not in self._TOPIC_QUIET_LIST:
                    rospy.logwarn("Received Statistics Information for untracked topic %s" % topic_name)
                continue
            # populate datasets for this topic
            delivered_msgs = list()
            traffic = list()
            period_mean = list()
            window_start = list()
            window_stop = list()
            node_sub = list()  # we need to know the number of subscribers
            for data in data_buffer:
                delivered_msgs.append(data.delivered_msgs)
                traffic.append(data.traffic)
                period_mean.append(data.period_mean)
                window_start.append(data.window_start.to_sec())
                window_stop.append(data.window_stop.to_sec())
                node_sub.append(data.node_sub)
            start_time = min(window_start)
            stop_time = max(window_stop)
            unique_subs = len(set(node_sub))
            # avoid divide by 0 errors
            if stop_time == start_time or unique_subs == 0:
                continue
            # Approximate the hz (per subscriber)
            total_msgs_sent = sum(delivered_msgs)
            messages_sent = total_msgs_sent / unique_subs
            hz = messages_sent / (stop_time - start_time)
            rsgTopics[topic_name].hz = hz
            # Approximate the bw in bytes per seconds (per subscriber)
            bytes_sent = sum(traffic) / unique_subs
            bw = bytes_sent / (stop_time - start_time)
            rsgTopics[topic_name].bw = bw

        # Reset data buffers
        self._previous_node_statistics_buffer = copy.copy(self._node_statistics_buffer)
        self._previous_host_statistics_buffer = copy.copy(self._host_statistics_buffer)
        self._previous_topic_statistics_buffer = copy.copy(self._topic_statistics_buffer)
        self._node_statistics_buffer.clear()
        self._host_statistics_buffer.clear()
        self._topic_statistics_buffer.clear()

        self._update_view()

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
