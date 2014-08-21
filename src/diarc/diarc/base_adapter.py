from view import View
from view import BlockItemAttributes
from view import BandItemAttributes
from view import SnapItemAttributes
from adapter import Adapter
from topology import *
import sys
import logging

log = logging.getLogger('diarc.base_adapter')

class BaseAdapter(Adapter):
    """ Basic implementation of the adapter interface.
    This should not have any QT or non-standard topology specific code. """
    def __init__(self, model, view):
        super(BaseAdapter, self).__init__(model, view)

        # These are caching lists so I can remember what I had last time I drew.
        # That way, if something becomes outdated, I have a thing to compare against.
        # These lists are updated at the end of _update_view()
        self._cached_block_item_indexes = list()
        self._cached_band_item_altitudes = list()
        self._cached_snap_item_snapkeys = list()

    def get_block_item_attributes(self, block_index):
        """ Default method for providing some stock settings for blocks """
        attrs = BlockItemAttributes()
        attrs.bgcolor = "white"
        attrs.border_color = "red"
        attrs.border_width = 0
        attrs.label = str(block_index)
#         attrs.label_rotation = -90
        attrs.label_color = "red"
        attrs.spacerwidth = 20
        return attrs

    def get_band_item_attributes(self, band_altitude):
        """ Default method for providing some stock settings for bands """
        attrs = BandItemAttributes()
        attrs.bgcolor = "white"
        attrs.border_color = "red"
        attrs.label = str(band_altitude)
        attrs.label_color = "red"
        attrs.width = 15
        return attrs

    def get_snap_item_attributes(self, snapkey):
        """ Default method for providing some stock settings for snaps """
        attrs = SnapItemAttributes()
        attrs.bgcolor = "white"
        attrs.border_color = "blue" if "e" in snapkey else "green"
        attrs.border_width = 0
        attrs.label = snapkey
        attrs.label_color = "red"
        attrs.width = 20
        return attrs


    def reorder_blocks(self,srcIdx,lowerIdx,upperIdx):
        """ reorders the index values of blocks and triggers the view to redraw.
        This also requires updating the corresponding block_items.
        """ 
        blocks = self._topology.blocks

        lastIdx = None
        currIdx = srcIdx
        # If we are moving to the right, lowerIdx is the target index.
        # Clear the dragged block's index, then shift all effected block
        # indices left.
        # NOTE: See issue #12
        if lowerIdx is not None and lowerIdx > srcIdx:
            while isinstance(currIdx,int) and currIdx < (upperIdx or lowerIdx+1): # In case upperIdx is None, use lower+1
                nextIdx = blocks[currIdx].rightBlock.index if blocks[currIdx].rightBlock else None
                blocks[currIdx].index = lastIdx
                log.debug("%s -> %s"%(str(currIdx),str(lastIdx)))
                lastIdx = currIdx
                currIdx = nextIdx
            assert lastIdx == lowerIdx, "%r %r"%(lastIdx,upperIdx)

        # If we are moving to the left, upperIdx is the target index.
        # Clear the dragged blocks index, then shift all effected blocks right
        elif upperIdx is not None and upperIdx < srcIdx:
            while isinstance(currIdx,int) and currIdx > lowerIdx:
                nextIdx = blocks[currIdx].leftBlock.index if blocks[currIdx].leftBlock else None
                blocks[currIdx].index = lastIdx
                log.debug("%s -> %s"%(str(currIdx),str(lastIdx)))
                lastIdx = currIdx
                currIdx = nextIdx
            assert lastIdx == upperIdx, "%r %r"%(lastIdx,upperIdx)

        # Otherwise we are just dragging to the side a bit and nothing is 
        # really moving anywhere. Return immediately to avoid trying to give
        # the block a new index and unnecessary extra linking actions.
        else:
            return False
        # Finally give the moved object its desired destination. Then make 
        # the TopologyWidget relink all the objects again.
        blocks[srcIdx].index = lastIdx
        self._update_view()
        return True


    def reorder_bands(self, srcAlt, lowerAlt, upperAlt):
        """ Reorders the altitude values of bands """
        bands = self._topology.bands

        lastAlt = None
        currAlt = srcAlt

        # If we are moving up, lowerAlt is the target altitude. 
        # Clear the dragged bands's altitude, then shift all effected bands
        # down. See issue #12
        if isinstance(lowerAlt,int) and lowerAlt > srcAlt:
            while isinstance(currAlt,int) and currAlt < (upperAlt or lowerAlt+1):
                tband = bands[currAlt].topBand
                nextAlt = tband.altitude if isinstance(tband,Band) else None
                bands[currAlt].altitude = lastAlt
                lastAlt = currAlt
                currAlt = nextAlt
            # Assertion check
            assert lastAlt == lowerAlt, "%r %r"%(lastAlt,lowerAlt)

        # If we are moving down, upperAlt is the target altitude.
        # Clear the dragged bands altitude, then shift all effected bands up.
        elif isinstance(upperAlt,int) and upperAlt <= srcAlt:
            while isinstance(currAlt,int) and currAlt > (lowerAlt or upperAlt-1):
                lband = bands[currAlt].bottomBand
                nextAlt = lband.altitude if isinstance(lband,Band) else None
                bands[currAlt].altitude = lastAlt
                lastAlt = currAlt
                currAlt = nextAlt
            # Assertion check
            assert lastAlt == upperAlt, "%r %r"%(lastAlt,upperAlt)

        else:
            return False

        # Finally, give the moved object its desired destination. Then make
        # the TopologyWidget relink all the objects again
        bands[srcAlt].altitude = lastAlt
        self._update_view()
        return True

    def reorder_snaps(self, blockIdx, container, srcIdx, lowerIdx, upperIdx):
        assert(container in ["emitter","collector"])
        block = self._topology.blocks[blockIdx]
        snaps = block.emitter if container == "emitter" else block.collector
        log.debug("%s"%snaps.keys())
        log.debug("move snap %s between %s and %s"%(srcIdx,lowerIdx,upperIdx))

        lastIdx = None
        currIdx = srcIdx
        # If we are moving to the right, lowerIdx is the target index.
        # Clear the dragged snaps's order, then shift all effected snap
        # indices left.
        # NOTE: see #12
        if lowerIdx is not None and lowerIdx > srcIdx:
            while isinstance(currIdx,int) and currIdx < (upperIdx or lowerIdx+1):
                nextIdx = snaps[currIdx].rightSnap.order if snaps[currIdx].rightSnap else None
                snaps[currIdx].order = lastIdx
                log.debug("%s -> %s"%(str(currIdx),str(lastIdx)))
                lastIdx = currIdx
                currIdx = nextIdx
            # Assertion check. TODO: Remove
            assert lastIdx == lowerIdx, "%r %r"%(lastIdx,lowerIdx)

        # If we are moving to the left, upperIdx is the target index.
        # Clear the dragged snaps order, then shift all effected snaps
        # indices right
        elif upperIdx is not None and upperIdx < srcIdx:
            while isinstance(currIdx,int) and currIdx > lowerIdx:
                nextIdx = snaps[currIdx].leftSnap.order if snaps[currIdx].leftSnap else None
                snaps[currIdx].order = lastIdx
                log.debug("%s -> %s"%(str(currIdx),str(lastIdx)))
                lastIdx = currIdx
                currIdx = nextIdx
            # Assertion check. TODO remove
            assert lastIdx == upperIdx, "%r %r"%(lastIdx,upperIdx)

        # Otherwise we are just dragging to the side a bit and nothing is
        # really moving anywhere. Return immediately to avoid trying to
        # give the snap a new order and unnecessary extra linking actions.
        else:
            return False

        # Finally give the moved object its desired destination. Then
        # make the TopologyWidget relink all the objects again.
        snaps[srcIdx].order = lastIdx
        self._update_view()
        return True

    def bring_band_to_front(self, altitude):
        bands = self._topology.bands

        src_band = bands[altitude]
        sibling_alts = [(alt, band.rank) for alt, band in bands.items() 
                if (src_band.isPositive and band.altitude > 0 and band.rank >= src_band.rank) or 
                (not src_band.isPositive and band.altitude < 0 and band.rank >= src_band.rank)]
        if len(sibling_alts) <= 1:
#             print "Band is already on top!"
            return
        # Order sibling altitudes by rank from lowest to highest
        sibling_alts.sort(lambda x,y: x[1] - y[1])
#         print "Bringing band with altitude %d to front of %s" % (src_band.altitude, sibling_alts)
        # Get the highest rank value (what we want this band to have)
        target_rank = max([x[1] for x in sibling_alts])
#         print "target rank value is %d" % target_rank
        # Strip rank information so we just have altitudes ordered by rank
        sibling_alts = [x[0] for x in sibling_alts]
        last_rank = src_band.rank
        src_band.rank = None
        for idx in range(1,len(sibling_alts)):
            band = bands[sibling_alts[idx]]
            next_rank = band.rank
            band.rank = last_rank
            last_rank = next_rank
        src_band.rank = target_rank

        self._update_view()

    def _update_view(self):
        """ updates the view - compute each items neigbors and then calls linking. """

        # Determine what items are in the model
        blocks = self._topology.blocks
        bands = self._topology.bands
        snaps = self._topology.snaps
        
        # Delete outdated BlockItems still in the view but no longer in the topology
        old_block_item_indexes = list(set(self._cached_block_item_indexes) - set(blocks.keys()))
        for index in old_block_item_indexes:
            self._view.remove_block_item(index)
            self._cached_block_item_indexes.remove(index)

        # Add new BlockItems for blocks in model that are not in view
        for index in blocks:
            if not self._view.has_block_item(index):
                self._view.add_block_item(index)
                self._cached_block_item_indexes.append(index)

        # Update the BlockItem cache list
#         self._cached_block_item_indexes = blocks.keys()



        # Delete outdated BandItems still in the view but not in the topology
        old_band_item_altitudes = list(set(self._cached_band_item_altitudes) - set(bands.keys()))
        for altitude in old_band_item_altitudes:
            self._view.remove_band_item(altitude)
            self._cached_band_item_altitudes.remove(altitude)

        # Delete BandItems that exist, but are not being used, and add BandItems
        # that are being used, but are not yet in the view
        for altitude in bands:
            band = bands[altitude]
            isUsed = band.isUsed()
            if isUsed and not self._view.has_band_item(altitude):
                self._view.add_band_item(altitude,band.rank)
                self._cached_band_item_altitudes.append(altitude)
            elif not isUsed and self._view.has_band_item(altitude):
                self._view.remove_band_item(altitude)
                self._cached_band_item_altitudes.remove(altitude)

        # Update the BandItem cache list
#         self._cached_band_item_altitudes = bands.keys()

        # Delete outdated SnapItems still in the view but no longer in the topology
        old_snap_item_snapkeys = list(set(self._cached_snap_item_snapkeys) - set(snaps.keys()))
        for snapkey in old_snap_item_snapkeys:
            self._view.remove_snap_item(snapkey)
            self._cached_snap_item_snapkeys.remove(snapkey)

        # Delete SnapItems that exist, but are not being used, and add SnapItems
        # that are being used, but are not yet in the view
        for snapkey in snaps:
            snap = snaps[snapkey]
            isUsed = snap.isUsed()
            if isUsed and not self._view.has_snap_item(snapkey):
                self._view.add_snap_item(snapkey)
                self._cached_snap_item_snapkeys.append(snapkey)
            elif not isUsed and self._view.has_snap_item(snapkey):
                self._view.remove_snap_item(snapkey)
                self._cached_snap_item_snapkeys.remove(snapkey)

        # Update the SnapItem cache list
#         self._cached_snap_item_snapkeys = snaps.keys()

        log.debug("*** Computing neighbors ***")
        sys.stdout.flush()
        log.debug("Blocks and snaps")
        sys.stdout.flush()
        # Compute left and right blocks
        for index in blocks:
            block = blocks[index]
            left_index = block.leftBlock.index if block.leftBlock is not None else None
            right_index = block.rightBlock.index if block.rightBlock is not None else None
            self._view.set_block_item_settings(index, left_index, right_index)
            # Compute left and right snaps, and what bands are being touched
            emitter = blocks[index].emitter
            collector = blocks[index].collector
            for snap in emitter.values() + collector.values():
                order = snap.order
                containername = "emitter" if snap.isSource() else "collector"
                # TODO: This assertion check should be commented out eventually
                if not snap.isUsed():
                    items = [item for item in self._view.layout_manager._snap_items if item.snap_order == order]
                    items = [item for item in items if item.container.strType() == containername]
                    items_orders = [item.snap_order for item in items if item.block_index == snap.block.index]
                    assert(order not in items_orders)
                    continue
                left_order = snap.leftSnap.order if snap.leftSnap is not None else None
                right_order = snap.rightSnap.order if snap.rightSnap is not None else None
                pos_alt = snap.posBandLink.altitude if snap.posBandLink else None
                neg_alt = snap.negBandLink.altitude if snap.negBandLink else None
                self._view.set_snap_item_settings(snap.snapkey(), left_order, right_order, pos_alt, neg_alt)
        log.debug("bands")
        sys.stdout.flush()
        # Compute top and bottom bands, rank, leftmost, and rightmost snaps
        for altitude in bands:
            # Skip bands that don't have an item 
            if not bands[altitude].isUsed():
                # The assertion below must be true for qt objects, however this 
                # code should remain view implementation agnostic and the so
                # the asserition is commented out.
#                 item_alts = [a for a in self._view.layout_manager._band_items]
#                 assert(altitude not in item_alts)
                continue
            band = bands[altitude]
            top_alt = band.topBand.altitude if band.topBand else None
            bot_alt = band.bottomBand.altitude if band.bottomBand else None
            emitters = band.emitters
            collectors = band.collectors
            emitters.sort(lambda x,y: x.block.index - y.block.index)
            collectors.sort(lambda x,y: x.block.index - y.block.index)
            left_snap = None
            right_snap = None
            if band.isPositive:
                left_snap = emitters[0]
                right_snap = collectors[-1]
            else:
                left_snap = collectors[0]
                right_snap = emitters[-1]
            left_snapkey = left_snap.snapkey() if left_snap is not None else None
            right_snapkey = right_snap.snapkey() if right_snap is not None else None
            self._view.set_band_item_settings(altitude, band.rank, top_alt, bot_alt, left_snapkey, right_snapkey )

        log.debug("*** Finished Computing neighbors ***")
        log.debug("*** Assigning Attributes ***")

        # Update block visual attribtutes
        for index in self._cached_block_item_indexes:
            attributes = self.get_block_item_attributes(index)
            self._view.set_block_item_attributes(index,attributes)

        # Update band visual attributes
        for altitude in self._cached_band_item_altitudes:
            attributes = self.get_band_item_attributes(altitude)
            self._view.set_band_item_attributes(altitude, attributes)

        # Update snap visual attribtutes
        for snapkey in self._cached_snap_item_snapkeys:
            attributes = self.get_snap_item_attributes(snapkey)
            self._view.set_snap_item_attributes(snapkey, attributes)
        log.debug("*** Finished Assigning Attributes ***")

        self._view.update_view()


