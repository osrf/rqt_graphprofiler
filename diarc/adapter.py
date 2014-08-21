class Adapter(object):
    """ Interface definations provided by the Adapter for use by the View.

    NOTE: There should be no Qt specific code here!
    """
    
    def __init__(self, model, view):
        self._topology = model
        self._view = view
        self._view.register_adapter(self)

    def reorder_blocks(self, srcIdx, lowerIdx, upperIdx):
        """ Move block with index srcIdx between blocks with lowerIdx and upperIdx. 
        Associated BlockItems are also be updated as necessary to reflect the change.

        srcIdx, lowerIdx, and upperIdx should all be ints, except for when 
        lowerIdx or upperIdx does not exists (at the far left and right) in which
        case they should be None.
        """
        raise NotImplementedError()

    def reorder_bands(self, srcAlt, lowerAlt, upperAlt):
        """ Move band with altitude srcAlt between bands with lowerAlt and upperAlt.
        Bands cannot be dragged to the opposite side of the BlockContainer, that
        is, positive bands can only be dragged with positive bands.
        """
        raise NotImplementedError()

    def reorder_snaps(self, blockIdx, container, srcIdx, lowerIdx, upperIdx):
        """ Move a snap with order srcIdx between snaps in the same container of
        the same block with index blockIdx with orders lowerIdx and upperIdx. 

        blockIdx - the index of the block the snaps are in
        container - either "emitter" or "collector"
        """
        raise NotImplementedError()

    def bring_band_to_front(self, altitude):
        """ change a bands rank to bring it to the front. This should not affect
        the relative order of the other band's ranks, but may change their 
        actual rank value. 
        """
        raise NotImplementedError()

#     def update_model(self):
#         raise NotImplementedError()
# 
#     def update_view(self):
#         raise NotImplementedError()
