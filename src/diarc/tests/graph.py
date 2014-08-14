#!/usr/bin/python
from parser import *
from topology import *
from util import *
import objgraph


# Make topology
t = parseFile('v3.xml')
objList = [t] + t.vertices.values() + t.edges.values() 
# for v in t.vertices.values():
#     objList.append(v.emitters.values())
#     objList.append(v.collectors.values())
# for e in t.edges.values():
#     objList.append(e.sources)
#     objList.append(e.sinks)

filterList = [  Topology,
                Vertex,
                Edge,
                Vertex.Collectors,
                Vertex.Emitters,
                Edge.Sources,
                Edge.Sinks,
                TypedDict,
                TypedList]
# objgraph.show_backrefs(objList,filter=lambda x: type(x) in filterList,filename='tmp.png')
# objgraph.show_backrefs(objList,filter=lambda x: not type(x) in [list], filename='tmp.png')
objgraph.show_backrefs(objList, filename='tmp.png')
