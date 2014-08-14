import xml.etree.ElementTree as ET
import xml.dom.minidom
from topology import *
""" v5 topology parser and serializer """

def parseFile(filename):
    return parseTree(ET.parse(filename))
    
def parseString(data):
    return parseTree(ET.fromstring(data))

def parseTree(tree):
    # Get XML Tree root and initialize topology
    root = tree.getroot()
    t = Topology()
 
    # Populate Edges
    edges = root.find("edges").findall("edge")

#     print "Num Edges Detected:",len(edges)

    # Keep track of edges for reference later
    edgeList = dict()

    for edge in edges:
        e = Edge(t)
        eid = int(edge.attrib['id'].strip())
        edgeList[eid] = e
        for band in edge.findall("band"):
            altitude = int(band.attrib["altitude"].strip())
            rank = int(band.attrib["rank"].strip())
            matches = filter(lambda x: x.rank==rank,t.bands.values())
            b = e.posBand if altitude > 0 else e.negBand
            b.altitude = altitude
            b.rank = rank

   
    # Populate Vertices
    vertices = root.find("vertices").findall("vertex")
#     print "Num Vertices Detected: %d"%len(vertices)
    for vertex in vertices:
        index = int(vertex.attrib['index'].strip())
        v = Vertex(t)
#         print "Creating Vertex with index=",index,v
        v.block.index = index

        # Make edge connections to this vertex
        for sink in vertex.find("collector").findall("sink"):
            order = int(sink.attrib["order"].strip())
            edgeid = int(sink.attrib["edge"].strip())
            e = edgeList[edgeid]
            if v in [s.vertex for s in e.sinks]:
                pass
#                 print "Existing Vertex found!"
            else:
                tmp = Sink(t,v,e)
                tmp.snap.order = order
#                 print "Creating sink with order=",order,"altitude=",altitude,tmp

        for source in vertex.find("emitter").findall("source"):
            order = int(source.attrib["order"].strip())
            edgeid = int(source.attrib["edge"].strip())
            e = edgeList[edgeid]
            if v in [src.vertex for src in e.sources]:
                pass
#                 print "Existing Vertex found"
            else:
                tmp = Source(t,v,e)
                tmp.snap.order = order
#                 print "Creating source with order=",order,"altitude=",altitude,tmp
    return t

def serialize(topology):
    """ Generate xml from topology """
    xmlRoot = ET.Element('topology')
    xmlVertices = ET.SubElement(xmlRoot,'vertices')
    xmlEdges = ET.SubElement(xmlRoot,'edges')

    nextEdgeId = 0
    revEdgeList = dict()

    # Serialize Edges
    for edge in topology.edges:
        xmlEdge = ET.SubElement(xmlEdges,'edge')
        eid = nextEdgeId
        nextEdgeId+=1
        xmlEdge.attrib["id"] = eid
        for band in [edge.posBand,edge.negBand]:
            b = ET.SubElement(xmlEdge,'band')
            b.attrib["altitude"] = str(band.altitude)
            b.attrib["rank"] = str(band.rank)
            revEdgeList[band.altitude] = eid

    # Serialize Vertices
    for vertex in topology.vertices:
        xmlVertex = ET.SubElement(xmlVertices,'vertex')
#         for source in vertex.sources:
# 
# 
# 
#         for sink in vertex.sinks:

    return xmlify(xmlRoot)




# Search children of ETree 
def find_element_by_attribute(root,elementname,attribname,attribval):
    element_list = root.findall(elementname)
    if element_list is None:
        raise Exception("No Elements of name %s found"%elementname)
    for tmp in element_list:
        try:
            if tmp.attrib[attribname] == attribval:
                return tmp
        except:
            raise Exception("Element %s has not attribute %s"%(elementname,attribname))
    raise Exception("Could not find %s with %s=%s"%(elementname,attribname,attribval))


def xmlify(root):
    # Split continuous string by newlines
    content = xml.dom.minidom.parseString(ET.tostring(root)).toprettyxml().split("\n")
    # Remove right hand whitespace
    content = [str(l).rstrip() for l in content]
    # Filter Blank lines
    content = filter(lambda x: not x == "",content)
    # Repack as a single string
    content = "\n".join(content)
    return content


