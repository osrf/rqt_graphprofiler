import unittest
import types




class Test_BlockNeighbors(unittest.TestCase):
    def test(self):
        import topology
        t = topology.Topology()
        v0 = topology.Vertex(t)
        v1 = topology.Vertex(t)
        v2 = topology.Vertex(t)
        v3 = topology.Vertex(t)

        assert(v0.block.index is None)
        assert(v1.block.index is None)
        assert(v2.block.index is None)
        assert(v3.block.index is None)

        v1.block.index = 1
        assert(v1.block.leftBlock is None)
        assert(v1.block.rightBlock is None)

        v3.block.index = 3
        assert(v1.block.leftBlock is None)
        assert(v1.block.rightBlock == v3.block)
        assert(v3.block.leftBlock == v1.block)
        assert(v3.block.rightBlock is None)

        v2.block.index = 0
        assert(v1.block.leftBlock == v2.block)
        assert(v1.block.rightBlock == v3.block)
        assert(v2.block.leftBlock is None)
        assert(v2.block.rightBlock == v1.block)
        assert(v3.block.leftBlock == v1.block)
        assert(v3.block.rightBlock is None)

        v2.block.index = 2
        assert(v1.block.leftBlock is None)
        assert(v1.block.rightBlock == v2.block)
        assert(v2.block.leftBlock == v1.block)
        assert(v2.block.rightBlock == v3.block)
        assert(v3.block.leftBlock == v2.block)
        assert(v3.block.rightBlock is None)

        v0.block.index = 0
        assert(v0.block.leftBlock is None)
        assert(v0.block.rightBlock == v1.block)
        assert(v1.block.leftBlock == v0.block)
        assert(v1.block.rightBlock == v2.block)
        assert(v2.block.leftBlock == v1.block)
        assert(v2.block.rightBlock == v3.block)
        assert(v3.block.leftBlock == v2.block)
        assert(v3.block.rightBlock is None)





class Test_v5_a(unittest.TestCase):
    def setUp(self):
        import parser
        self.t = parser.parseFile('data/v5_a.xml')
 
    def test_band_emitters_collectors(self):
        t = self.t

        assert([snap.block.index for snap in t.edges[0].posBand.emitters] == [0])
        assert([snap.block.index for snap in t.edges[0].posBand.collectors] == [1])
        assert([snap.block.index for snap in t.edges[0].negBand.emitters] == [2])
        assert([snap.block.index for snap in t.edges[0].negBand.collectors] == [1])

    def test_block(self):
        t = self.t
        # Make sure the matching is correct
        for index in t.blocks:
            assert(t.blocks[index].index == index)

        # Count the number of emitters and collectors of each block
        # {blockIndex: (#collectors,#emitters), ...}
        vals = { 0: (0,1),  1: (1,0),  2: (0,1)}
        for index in t.blocks:
            assert(len(t.blocks[index].collector) == vals[index][0])
            assert(len(t.blocks[index].emitter) == vals[index][1])

    def test_snaps_to_bands_connectivity(self):
        """ Test to make sure that snaps are connected to the correct bands """
        t = self.t
        assert(t.blocks[0].emitter[0].posBandLink.altitude == 1)
        assert(t.blocks[0].emitter[0].negBandLink is None)
        assert(t.blocks[1].collector[0].posBandLink.altitude == 1)
        assert(t.blocks[1].collector[0].negBandLink.altitude == -1)
        assert(t.blocks[2].emitter[0].posBandLink is None)
        assert(t.blocks[2].emitter[0].negBandLink.altitude == -1)



class Test_v5_b(unittest.TestCase):
    def setUp(self):
        import parser
        self.t = parser.parseFile('data/v5_b.xml')
 
    def test_band_emitters_collectors(self):
        t = self.t


        assert([snap.block.index for snap in t.edges[0].posBand.emitters] == [1])
        assert([snap.block.index for snap in t.edges[0].posBand.collectors] == [2])
        assert([snap.block.index for snap in t.edges[0].negBand.emitters] == [1])
        assert([snap.block.index for snap in t.edges[0].negBand.collectors] == [0])

    def test_block(self):
        t = self.t
        # Make sure the matching is correct
        for index in t.blocks:
            assert(t.blocks[index].index == index)

        # Count the number of emitters and collectors of each block
        # {blockIndex: (#collectors,#emitters), ...}
        vals = { 0: (1,0),  1: (0,1),  2: (1,0)}
        for index in t.blocks:
            assert(len(t.blocks[index].collector) == vals[index][0])
            assert(len(t.blocks[index].emitter) == vals[index][1])

    def test_snaps_to_bands_connectivity(self):
        """ Test to make sure that snaps are connected to the correct bands """
        t = self.t
        assert(t.blocks[0].collector[0].posBandLink is None)
        assert(t.blocks[0].collector[0].negBandLink.altitude == -1)
        assert(t.blocks[1].emitter[0].posBandLink.altitude == 1)
        assert(t.blocks[1].emitter[0].negBandLink.altitude == -1)
        assert(t.blocks[2].collector[0].posBandLink.altitude == 1)
        assert(t.blocks[2].collector[0].negBandLink is None)



class Test_v5_c(unittest.TestCase):
    def setUp(self):
        import parser
        self.t = parser.parseFile('data/v5_c.xml')
 
    def test_band_emitters_collectors(self):
        t = self.t

        assert([snap.block.index for snap in t.edges[0].posBand.emitters] == [0])
        assert([snap.block.index for snap in t.edges[0].posBand.collectors] == [1,2])
        assert([snap.block.index for snap in t.edges[0].negBand.emitters] == [])
        assert([snap.block.index for snap in t.edges[0].negBand.collectors] == [])

    def test_block(self):
        t = self.t
        # Make sure the matching is correct
        for index in t.blocks:
            assert(t.blocks[index].index == index)

        # Count the number of emitters and collectors of each block
        # {blockIndex: (#collectors,#emitters), ...}
        vals = { 0: (0,1),  1: (1,0),  2: (1,0)}
        for index in t.blocks:
            assert(len(t.blocks[index].collector) == vals[index][0])
            assert(len(t.blocks[index].emitter) == vals[index][1])

    def test_snaps_to_bands_connectivity(self):
        """ Test to make sure that snaps are connected to the correct bands """
        t = self.t
        assert(t.blocks[0].emitter[0].posBandLink.altitude == 1)
        assert(t.blocks[0].emitter[0].negBandLink is None)
        assert(t.blocks[1].collector[0].posBandLink.altitude == 1)
        assert(t.blocks[1].collector[0].negBandLink is None)
        assert(t.blocks[2].collector[0].posBandLink.altitude == 1)
        assert(t.blocks[2].collector[0].negBandLink is None)

class Test_v5_d(unittest.TestCase):
    def setUp(self):
        import parser
        self.t = parser.parseFile('data/v5_d.xml')
 
    def test_band_emitters_collectors(self):
        t = self.t
        assert([snap.block.index for snap in t.edges[0].posBand.emitters] == [])
        assert([snap.block.index for snap in t.edges[0].posBand.collectors] == [])
        assert([snap.block.index for snap in t.edges[0].negBand.emitters] == [1,2])
        assert([snap.block.index for snap in t.edges[0].negBand.collectors] == [0])

    def test_block_index(self):
        """ checks that topology block indexing follows block index values """
        t = self.t
        # Make sure the matching is correct
        for index in t.blocks:
            assert(t.blocks[index].index == index)

    def test_emitter_collector_count(self):
        """ Counts the number of emitters and collectors in each block """
        # {blockIndex: (#collectors,#emitters), ...}
        vals = { 0: (1,0),  1: (0,1),  2: (0,1)}
        t = self.t
        for index in t.blocks:
            assert(len(t.blocks[index].collector) == vals[index][0])
            assert(len(t.blocks[index].emitter) == vals[index][1])

    def test_snaps_connectivity(self):
        """ Test to make sure that snaps are connected to the correct bands """
        t = self.t
        assert(t.blocks[0].collector[0].posBandLink is None)
        assert(t.blocks[0].collector[0].negBandLink.altitude == -1)
        assert(t.blocks[1].emitter[0].posBandLink is None)
        assert(t.blocks[1].emitter[0].negBandLink.altitude == -1)
        assert(t.blocks[2].emitter[0].posBandLink is None)
        assert(t.blocks[2].emitter[0].negBandLink.altitude == -1)

class Test_v5_e(unittest.TestCase):
    def setUp(self):
        import parser
        self.t = parser.parseFile('data/v5_e.xml')
 
    def test_band_emitters_collectors(self):
        t = self.t
        # Check locations
        assert([snap.block.index for snap in t.edges[0].posBand.emitters] == [0,2])
        assert([snap.block.index for snap in t.edges[0].posBand.collectors] == [1,3])
        assert([snap.block.index for snap in t.edges[0].negBand.emitters] == [2])
        assert([snap.block.index for snap in t.edges[0].negBand.collectors] == [1])

    def test_block_index(self):
        """ checks that topology block indexing follows block index values """
        t = self.t
        # Make sure the matching is correct
        for index in t.blocks:
            assert(t.blocks[index].index == index)

    def test_emitter_collector_count(self):
        """ Counts the number of emitters and collectors in each block """
        # {blockIndex: (#collectors,#emitters), ...}
        vals = { 0: (0,1),  1: (1,0),  2: (0,1), 3: (1,0)}
        t = self.t
        for index in t.blocks:
            assert(len(t.blocks[index].collector) == vals[index][0])
            assert(len(t.blocks[index].emitter) == vals[index][1])

    def test_snaps_connectivity(self):
        """ Test to make sure that snaps are connected to the correct bands """
        t = self.t
        assert(t.blocks[0].emitter[0].posBandLink.altitude == 1)
        assert(t.blocks[0].emitter[0].negBandLink is None)
        assert(t.blocks[1].collector[0].posBandLink.altitude == 1)
        assert(t.blocks[1].collector[0].negBandLink.altitude == -1)
        assert(t.blocks[2].emitter[0].posBandLink.altitude == 1)
        assert(t.blocks[2].emitter[0].negBandLink.altitude == -1)
        assert(t.blocks[3].collector[0].posBandLink.altitude == 1)
        assert(t.blocks[3].collector[0].negBandLink is None)


class Test_v5_f(unittest.TestCase):
    def setUp(self):
        import parser
        self.t = parser.parseFile('data/v5_f.xml')
 
    def test_band_emitters_collectors(self):
        t = self.t
       
        # Check locations
        assert([snap.block.index for snap in t.edges[0].posBand.emitters] == [1])
        assert([snap.block.index for snap in t.edges[0].posBand.collectors] == [2])
        assert([snap.block.index for snap in t.edges[0].negBand.emitters] == [1,3])
        assert([snap.block.index for snap in t.edges[0].negBand.collectors] == [0,2])

    def test_block_index(self):
        """ checks that topology block indexing follows block index values """
        t = self.t
        # Make sure the matching is correct
        for index in t.blocks:
            assert(t.blocks[index].index == index)

    def test_emitter_collector_count(self):
        """ Counts the number of emitters and collectors in each block """
        # {blockIndex: (#collectors,#emitters), ...}
        vals = { 0: (1,0),  1: (0,1),  2: (1,0), 3: (0,1)}
        t = self.t
        for index in t.blocks:
            assert(len(t.blocks[index].collector) == vals[index][0])
            assert(len(t.blocks[index].emitter) == vals[index][1])

    def test_snaps_connectivity(self):
        """ Test to make sure that snaps are connected to the correct bands """
        t = self.t
        assert(t.blocks[0].collector[0].negBandLink.altitude == -1)
        assert(t.blocks[0].collector[0].posBandLink is None)
        assert(t.blocks[1].emitter[0].negBandLink.altitude == -1)
        assert(t.blocks[1].emitter[0].posBandLink.altitude == 1)
        assert(t.blocks[2].collector[0].negBandLink.altitude == -1)
        assert(t.blocks[2].collector[0].posBandLink.altitude == 1)
        assert(t.blocks[3].emitter[0].negBandLink.altitude == -1)
        assert(t.blocks[3].emitter[0].posBandLink is None)




if __name__ == "__main__":
    unittest.main()




