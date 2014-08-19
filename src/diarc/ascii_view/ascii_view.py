from diarc.snapkey import gen_snapkey
from diarc.snapkey import parse_snapkey
from diarc.util import TypedDict
from diarc.view import View
from CharGrid import CharGrid

block_spacing = 5

class BlockItem(object):
    def __init__(self, parent, index):
        self.parent = parent
        self._index = index
        self.left_block = None
        self.right_block = None

        # These are populated
        self.emitters = list()
        self.collectors = list()

        # Layout properties
        self._leftCol = None
        self._centerCol = None
        self._rightCol = None
        self._topRow = None
        self._centerRow = None
        self._botRow = None


    def release(self):
        self.left_block = None
        self.right_block = None
        for item in self.emitters:
            item.release()
        for item in self.collectors:
            item.release()

    @property
    def leftCol(self):
        if self._leftCol is None: self.layout()
        return self._leftCol
        
    @property
    def centerCol(self):
        if self._centerCol is None: self.layout()
        return self._centerCol

    @property
    def rightCol(self):
        if self._rightCol is None: self.layout()
        return self._rightCol

    @property
    def topRow(self):
        if self._topRow is None: self.layout()
        return self._topRow

    @property
    def centerRow(self):
        if self._centerRow is None: self.layout()
        return self._centerRow

    @property
    def botRow(self):
        if self._botRow is None: self.layout()
        return self._botRow

    def layout(self):
        if self.left_block is None:
            self.left_block = self.parent.corner_stone
            self.parent.corner_stone.right_block = self

        if None in [self.left_block.topRow,self.left_block.centerRow,self.left_block.topRow]:
            self.left_block.layout()
        self._topRow = self.left_block.topRow
        self._centerRow = self.left_block.centerRow
        self._botRow = self.left_block.botRow

        if None in [self.left_block.leftCol,self.left_block.centerCol,self.left_block.rightCol]:
            self.left_block.layout()
        self._leftCol = self.left_block.rightCol + block_spacing + 1
        self._centerCol =  self.leftCol + 2 + 2*len(self.collectors)
        self._rightCol = self.centerCol + 2 + 2*len(self.emitters) 

    def draw(self,grid):
        """ draw this Block into the grid """
        # Draw the left side of the box
        grid[(self.topRow,self.leftCol)] = '+-'
        grid[(self.centerRow,self.leftCol)] = '| '
        grid[(self.botRow,self.leftCol)] = '+-'

        # Draw Collector spacers
        grid[(self.topRow,self.leftCol+1)] = "-"*(self.centerCol-self.leftCol-1)
        grid[(self.botRow,self.leftCol+1)] = "-"*(self.centerCol-self.leftCol-1)

        # Draw the middle line
        grid[(self.topRow,self.centerCol)] = '+-'
        grid[(self.centerRow,self.centerCol)] = '| '
        grid[(self.botRow,self.centerCol)] = '+-'

        # Draw the emitter spacers
        grid[(self.topRow,self.centerCol+1)] = "-"*(self.rightCol-self.centerCol-1)
        grid[(self.botRow,self.centerCol+1)] = "-"*(self.rightCol-self.centerCol-1)
 
        # Draw the right line
        grid[(self.topRow,self.rightCol)] = '+'
        grid[(self.centerRow,self.rightCol)] = '|'
        grid[(self.botRow,self.rightCol)] = '+'


class BandItem(object):
    def __init__(self, parent, altitude, rank):
        self.parent = parent
        self._altitude = altitude
        self._rank = rank
        self.top_band = None
        self.bot_band = None
        self.left_most_snap = None
        self.right_most_snap = None

        # Visual parameters
        self._row = None
        self._startCol = None
        self._endCol = None
        self._isUsed = False

    def release(self):
        self.top_band = None
        self.bot_band = None
        self.left_most_snap = None
        self.right_most_snap = None

    @property
    def row(self):
        if self._row is None: self.layout()
        return self._row

    @property
    def startCol(self):
        if self._startCol is None: self.layout()
        return self._startCol

    @property
    def endCol(self):
        if self._endCol is None: self.layout()
        return self._endCol


    def layout(self):
        if self.bot_band is None:
            if self._altitude > 0:
                self.bot_band = self.parent.corner_stone
                self.parent.corner_stone.top_band = self
                self.parent.corner_stone.layout()
                return
            
        if self.top_band is None:
            if self._altitude > 0:
                self.top_band = self.parent.top_line
                self.parent.top_line.bot_band = self
            else:
                self.top_band = self.parent.corner_stone
                self.parent.corner_stone.bot_band = self
        else:
            if self.top_band.row is None:
                self.top_band.layout()
            self._row = self.top_band.row + 1
            # Get the snap positions
            if self.left_most_snap.col is None:
                self.left_most_snap.layout()
            if self.right_most_snap.col is None:
                self.right_most_snap.layout()
            self._startCol = self.left_most_snap.col+2
            self._endCol = self.right_most_snap.col-2
           

    def draw(self,grid):
        grid[(self.row,0)] = str(self._altitude)
        grid[(self.row,self.startCol)] = '-'*((self.endCol-self.startCol)+1)


class SnapItem(object):
    def __init__(self, parent, snapkey):
        self.parent = parent
        self._snapkey = snapkey
        self.block_index, self.container_name, self.snap_order = parse_snapkey(snapkey)

        self.block_item = None
        self.left_snap = None
        self.right_snap = None
        self.posBandItem = None
        self.negBandItem = None

        # Layout properties
        self.col = None
        self.centerRow = None
        self.posBandRow = None
        self.negBandRow = None

    def release(self):
        if self.isSource():
            self.block_item.emitters.remove(self)
        else:
            self.block_item.collectors.remove(self)
        self.block_item = None
        self.left_snap = None
        self.right_snap = None
        self.posBandItem = None
        self.negBandItem = None

    def isSource(self):
        return True if self.container_name == "emitter" else False

    def isSink(self):
        return True if self.container_name == "collector" else False

    def layout(self):
        # Determine your col location
        # First, figure out what to offset against
        if self.isSource():
            self.col = self.block_item.centerCol 
        elif self.isSink():
            self.col = self.block_item.leftCol 
        else:
            raise Exception("Snap isnt Source or Sink")

        # Then add the order offset
        # TODO: This assumes that there are no skipped order values, which 
        # would leave holes when drawing. The correct way would be to get relative 
        # ordering of the order values and take into account missing values.
        self.col += 2 + 2*self.snap_order

        # Get the center row
        self.centerRow = self.block_item.centerRow

        # Get the band rows (may require doing layout)
        #TODO: row = posBand.visual.row etc should be calculated here instead of in draw


    def draw(self,grid):
        centerRow = self.centerRow
        col = self.col
        if self.isSink():
            if self.posBandItem:
                # Draw a Positive Sink Snap
                row = self.posBandItem.row
                grid[(centerRow-1,col)] = 'V-'
                grid[(row,col-1)] = '.'
                for i in range(centerRow-2-row):
                    grid[(centerRow-2-i,col)] = '|'
            if self.negBandItem:
                # Draw a Negative Sink Snap
                row = self.negBandItem.row
                grid[(centerRow+1,col)] = 'A-'
                grid[(row,col+1)] = "'"
                for i in range(row-(centerRow+2)):
                    grid[(centerRow+2+i,col)] = '|'
        elif self.isSource():
            if self.posBandItem:
                # Draw a Positive Source Snap
                row = self.posBandItem.row
                grid[(centerRow-1,col)] = 'A-'
                grid[(row,col+1)] = '.'
                for i in range(centerRow-2-row):
                    grid[(centerRow-2-i,col)] = '|'
            if self.negBandItem:
                # Draw a Negative Source Snap
                row = self.negBandItem.row
                grid[(centerRow+1,col)] = 'V-'
                grid[(row,col-1)] = "'"
                for i in range(row-(centerRow+2)):
                    grid[(centerRow+2+i,col)] = '|'
        else:
            raise Exception("Snap is not source or sink")


class TopLine(object):
    def __init__(self):
        self.row = 0

        #Adjoining bands
        self.below = None

    def layout(self):
        self.row = 0

    def draw(self):
        pass


class CornerStone(object):
    """ What everything tries to be relative to """
    def __init__(self):
        #Adjoining blocks
        self.rightBlock = None
        # Adjoining bands
        self.top_band = None
        self.bot_band = None
        # Positioning Parameters
        self.leftCol = None
        self.centerCol = None
        self.rightCol = None
        self.topRow = None
        self.centerRow = None
        self.botRow = None

    @property
    def row(self):
        """ return a calculated row since the spacing here is non-standard. Basically
        we return the empty row below the band of Blocks. This is used for band layout """
        if not self.botRow:
            self.layout()
        return self.botRow+1
        

    def layout(self):
        # TODO: topRow should be replaced with finding height of adjacent bands bottom
        if self.top_band is None:
            self.topRow = 0
            self.centerRow = 1
            self.botRow = 2
        else:
            if self.top_band.row is None:
                self.top_band.layout()
            self.topRow = self.top_band.row + 2
            self.centerRow = self.topRow+1
            self.botRow = self.centerRow+1
        
        self.leftCol = -block_spacing-1
        self.centerCol = -block_spacing-1
        self.rightCol = -block_spacing-1

    def draw(self,grid):
        # This is not a real thing, don't draw anything
        pass




class AsciiView(View):
    def __init__(self):
        View.__init__(self)
        
        # Special layout objects
        self.top_line = TopLine()
        self.corner_stone = CornerStone()

        # Visual Objects we are tracking
        self._block_items = TypedDict(int, BlockItem)
        self._band_items = TypedDict(int, BandItem)
        self._snap_items = TypedDict(str, SnapItem)

    def add_block_item(self, index):
        """ Create new a drawable object to correspond to a Block with this index. """
        if not index in self._block_items:
            print "Adding block item",index
            item = BlockItem(self, index)
            self._block_items[index] = item
            return item

    def has_block_item(self, index):
        return True if index in self._block_items else False

    def set_block_item_settings(self, index, left_index, right_index):
        item = self._block_items[index]
        item.left_block = self._block_items[left_index] if left_index is not None else None
        item.right_block = self._block_items[right_index] if right_index is not None else None

    def set_block_item_attributes(self, index, attributes):
        """ Not yet implemented """
        pass

    def remove_block_item(self, index):
        print "Removing BlockItem %d"%index
        self._block_items[index].release()
        self._block_items.pop(index)

    def add_band_item(self, altitude, rank):
        """ Create a new drawable object to correspond to a Band. """
        print "Adding BandItem with altitude %d"%altitude
        if altitude in self._band_items:
            raise DuplicateItemExistsError("BandItem with altitude %d already exists"%(altitude))
        item = BandItem(self, altitude, rank)
        self._band_items[altitude] = item
        return item

    def has_band_item(self, altitude):
        return True if altitude in self._band_items else False

    def remove_band_item(self, altitude):
        """ Remove the drawable object to correspond to a band """ 
        print "Removing BandItem altitude %d"%altitude
        self._band_items[altitude].release()
        self._band_items.pop(altitude)

    def get_band_item(self, altitude):
        return self._band_items[altitude]
    
    def set_band_item_settings(self, altitude, rank,
                                top_band_alt, bot_band_alt,
                                leftmost_snapkey, rightmost_snapkey):
        item = self._band_items[altitude]
        item.top_band = self._band_items[top_band_alt] if top_band_alt is not None else None
        item.bot_band = self._band_items[bot_band_alt] if bot_band_alt is not None else None
        item.left_most_snap = self._snap_items[leftmost_snapkey]
        item.right_most_snap = self._snap_items[rightmost_snapkey]

    def set_band_item_attributes(self, index, attributes):
        """ not yet implemented for this style view """
        pass


    def add_snap_item(self, snapkey):
        print "Adding SnapItem %s"%snapkey
        if snapkey in self._snap_items:
            raise DuplicateItemExistsError("SnapItem with snapkey %s already exists"%(snapkey))
        item = SnapItem(self, snapkey)
        item.block_item = self._block_items[item.block_index]
        if item.isSource():
            self._block_items[item.block_index].emitters.append(item)
        else:
            self._block_items[item.block_index].collectors.append(item)
        self._snap_items[snapkey] = item
        return item

    def remove_snap_item(self, snapkey):
        print "Removing SnapItem %s"%snapkey
        self._snap_items[snapkey].release()
        self._snap_items.pop(snapkey)

    def has_snap_item(self, snapkey):
        return True if snapkey in self._snap_items else False


    def set_snap_item_settings(self, snapkey, left_order, right_order, pos_band_alt, neg_band_alt):
        item = self._snap_items[snapkey]
        if left_order is not None:
            left_snapkey = gen_snapkey(item.block_index,item.container_name,left_order)
            item.left_snap = self._snap_items[left_snapkey]
        else:
            item.left_snap = None
        if right_order is not None:
            right_snapkey = gen_snapkey(item.block_index,item.container_name,right_order)
            item.right_snap = self._snap_items[right_snapkey]
        else:
            item.right_snap = None
        item.posBandItem = self._band_items[pos_band_alt] if pos_band_alt is not None else None
        item.negBandItem = self._band_items[neg_band_alt] if neg_band_alt is not None else None

    def set_snap_item_attributes(self, snapkey, attributes):
        """ not yet implemented for this style view """
        pass

    def update_view(self):
        
        # Initialize Visual Elements
        cornerStone = CornerStone()
       
        for item in self._band_items.values():
            item.layout()

        for item in self._block_items.values():
            item.layout()

        for item in self._snap_items.values():
            item.layout()

        # Draw
        grid = CharGrid()
        
        for item in self._band_items.values():
            item.draw(grid)

        for item in self._block_items.values():
            item.draw(grid)

        for item in self._snap_items.values():
            item.draw(grid)
          
        print grid




class DuplicateItemExistsError(Exception):
    """ An Item with the specified parameters already exists """
    pass
