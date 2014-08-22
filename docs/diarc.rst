.. code-block:: python

 import diarc

 Define a graph with 3 vertices (v1, v2, and v3), and two edges. Edge e1 has a
 single input from v1 and two outputs to v2 and v3. Edge e2 has two inputs from
 v1 and v2 and a single output to v3.

.. code-block:: python

 t = diarc.Topology()
 v1 = diarc.Vertex(t)
 v2 = diarc.Vertex(t)
 v3 = diarc.Vertex(t)
 e1 = diarc.Edge(t)
 src1 = diarc.Source(t,v1,e1)
 sink1 = diarc.Sink(t,v2,e1)
 sink2 = diarc.Sink(t,v3,e1)
 e2 = diarc.Edge(t)
 src2 = diarc.Source(t,v1,e2)
 src3 = diarc.Source(t,v2,e2)
 sink3 = diarc.Sink(t,v3,e2)

# Define visual characteristics describing how to draw graph. 
# Assign the order in which Vertices are displayed. Vertices are represented by
# visual objects called blocks. Values must be unique and lower values will be 
# displayed to the left.

.. code-block:: python

 v1.block.index = 1
 v2.block.index = 2
 v3.block.index = 3

# Edges are represented at visual objets called bands. Since edges must point 
# from left to right above the vertices and right to left below them, each edge
# must define two band objects corresponding to the two possible positions the 
# edge could be in. The position above the vertices is represented by the positive
# band (posBand) while the position below by the negative band (negBand). Bands
# are drawn in an order defined by their altitude. Positive bands have positive
# altitudes and negative bands have negative altitudes. The higher the altitude
# the further towards the top of the screen the band is drawn. Altitude 0 is 
# reserved for the line of Vertices.
e1.posBand.altitude = 1
e1.negBand.altitude = -1
e2.posBand.altitude = 2
e2.negBand.altitude = -2

# Since edges can overlap we need to define the drawing order in which to layer
# them. This is defined by a band's rank, with higher values being drawn on top
# of lower values. Rank values must be unique among posBand's and negBand's 
# seperately, and negative values are not allowed.
e1.posBand.rank = 1
e1.negBand.rank = 1
e2.posBand.rank = 2
e2.negBand.rank = 2

# To display the Visualization
from diarc.qt_view import QtView
from diarc.base_adapter import BaseAdapter
view = QtView()
adapter = BaseAdapter(t, view)
adapter._update_view()



diarc
   Topology
   Vertex
   Edge
   Source
   Sink
   Block
   Band
   Snap
   Adapter
   View

