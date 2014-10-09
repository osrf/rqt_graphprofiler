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
    """ Implements the Adapter interface as a static xml parser. """

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

    def disable_auto_update(self)
        """ This does nothing since the graph is not actually updating """
        pass

    def show_disconnected_topics(self):
        self._topology.hide_disconnected_snaps = False
        self.topology_update()

    def hide_disconnected_topics(self):
        self._topology.hide_disconnected_snaps = True
        self.topology_update()

#     def topology_update(self):
#         """ parse the xml file """
#         with open("input.xml","r") as f:
