
class View(object):
    """ Interface definations provided by the View for use by the Adapter.  
    
    NOTE: There should be no Qt specific code here!
    """
    def __init__(self):
        self.adapter = None

    def register_adapter(self,adapter):
        """ Saves a pointer to the adapter object."""
        self.adapter = adapter

    def update_view(self):
        raise NotImplementedError()


    def add_block_item(self, index):
        """ Create a new drawable BlockItem object inside the View with index.
        This is intended to be used to create drawable objects that correespond 
        with a block of the same index value.
        :param int index: position of the BlockItem
        :raises: DuplicateItemExistsError
        """
        raise NotImplementedError()

    def has_block_item(self, index):
        """ Returns if a BlockItem with specified index exists inside the view
        :param int index: index of desired BlockItem
        :rtype: bool
        :returns: True if BlockItem exists inside View, else False
        """
        raise NotImplementedError()

    def remove_block_item(self, index):
        """ Remove the drawable BlockItem object from the View that corresponds with index 
        This is intended to be used to remove drawable objects that correspond 
        with a block of the same index value.
        :param int index: index of BlockItem
        :raises: ItemDoesNotExistError
        """
        raise NotImplementedError()
   
    def set_block_item_settings(self, index, left_index, right_index):
        """ Sets the relative position of the BlockItem at index. 
        All three BlockItems must exist before this can be called.
        'None' may be specified in place of an int for left_index if no BlockItem
        exists to the left/right of the target (because it is the leftmost or
        rightmost BlockItem). 
        :param int index: index of target BlockItem to set position of
        :param int left_index: the index of the BlockItem directly to the left of the target
        :param int right_index: the index of the BlockItem directly to the right of the target
        :raises: ItemDoesNotExistError
        """
        raise NotImplementedError()

    def set_block_item_attributes(self, index, attributes):
        """ Copy settings from a BlockItemViewAttributes object to a BlockItem.
        For more information about attributes that can be set, see BlockItemViewAttributes.
        :param int index: index of the target BlockItem
        :param BlockItemViewAttributes attributes: class containing the desired settings
        :raises: ItemDoesNotExistError
        """ 
        raise NotImplementedError()

    def add_band_item(self, altitude, rank):
        """ Create a new drawable BandItem object inside the View. 
        This is intended to be used to create drawable objects that correspond 
        with a band of the same altitude and rank values.
        :param int altitude: vertical position of the BandItem
        :param int rank: initial drawing order of the BandItem
        :raises: DuplicateItemExistsError
        """
        raise NotImplementedError()

    def has_band_item(self, altitude):
        """ Returns if a BandItem with specified altitude exists inside the view
        :param int altitude: altitude of desired BandItem
        :rtype: bool
        :returns: True if BandItem exists inside View, else False
        """
        raise NotImplementedError()

    def remove_band_item(self, altitude):
        """ Remove the drawable object to correspond to a band
        This is intended to be used to remove drawable objects that correspond 
        with a band of the same altitude value.
        :param int altitude: altitude of BandItem
        :raises: ItemDoesNotExistError
        """ 
        raise NotImplementedError()

    def set_band_item_settings(self, altitude, rank, top_band_alt, bot_band_alt,
                                leftmost_snapkey, rightmost_snapkey):
        """ Sets the relative position and size of the BandItem.
        The two BandItems corresponding to top_band_alt and bot_band_alt, as well
        as the two SnapItems corresponding to leftmost_snapkey and rightmost_snapkey
        must exist before this can be called.
        'None' may be specified in place of an int for top_band_alt or bot_band_alt
        if no BandItem exists to the top/bottom of the target band. This could 
        either be because it is the topmost or bottommost BandItem, or because it
        is adjacent to the line of BlockItems. 
        :param int altitude: altitude of the target BandItem 
        :param int rank: the drawing order of the BandItem, higher numbers are drawn above lower numbers
        :param int top_band_alt: the altitude of the BandItem directly above the target
        :param int bot_band_alt: the altitude of the BandItem directly below the target
        :param str leftmost_snapkey: the snapkey of the leftmost SnapItem touched by this band. For positive BandItems this must be a source, for negative bandItems this must be a sink.
        :param str rightmost_snapkey: the snapkey of the rightmost SnapItem touched by this band. For positive BandItems this must be a sink, for negative bandItems this must be a source.
        :raises: ItemDoesNotExistError
        """
        raise NotImplementedError()

    def set_band_item_attributes(self, index, attributes):
        """ Copy settings from a BandItemViewAttributes object to a BandItem
        For more information about attributes that can be set, see BandItemViewAttributes.
        :param int index: index of the target BandItem
        :param BandItemViewAttributes attributes: class containing the desired settings
        :raises: ItemDoesNotExistError
        """ 
        raise NotImplementedError()


    def add_snap_item(self, snapkey):
        """ Creates a new SnapItem in the view referenced by the given snapkey
        This is intended to be used to create drawable objects that correespond 
        with a snap of the same snapkey value.
        :param str snapkey: position of the SnapItem
        :raises: DuplicateItemExistsError
        """
        raise NotImplementedError()

    def has_snap_item(self, snapkey):
        """ Checks to see if a SnapItem corresponding to the snapkey exists in the view 
        :param str snapkey: snapkey of desired SnapItem
        :rtype: bool
        :returns: True if BandItem exists inside View, else False
        """
        raise NotImplementedError()

    def remove_snap_item(self, snapkey):
        """ Removes a SnapItem from the view referenced by the given snapkey
        This is intended to be used to remove drawable objects that correspond 
        with a snap of the same snapkey value.
        :param str snapkey: snapkey of SnapItem
        :raises: ItemDoesNotExistError
        """
        raise NotImplementedError()

    def set_snap_item_settings(self, snapkey, left_order, right_order, pos_band_alt, neg_band_alt):
        """ Sets the relative position of the target SnapItem specified by snapkey 
        with respect to other SnapItems in the same container of the same Block. 
        The SnapItems corresponding to the target, left_order, and right_order, 
        as well as the BandItems corresponding to pos_band_alt and neg_band_alt
        must all exist before this can be called.
        'None' may be specified in place of an int for left_order or right_order
        if no SnapItem exists to the left/right of the target SnapItem. This could 
        either be because it is the leftmost or rightmost SnapItem in the container. 
        It could also be used for pos_band_alt or neg_band_alt.
        :param str snapkey: snapkey of the target SnapItem 
        :param int left_order: the order of the SnapItem directly to the left of the target
        :param int right_order: the order of the SnapItem directly to the right of the target
        :param int pos_band_alt: the altitude of the positive BandItem attached to the target, if any
        :param int neg_band_alt: the altitude of the negative BandItem attached to the target, if any
        :raises: ItemDoesNotExistError
        """
        raise NotImplementedError()

    def set_snap_item_attributes(self, snapkey, attributes):
        """ Copy settings from a SnapItemViewAttributes object to a SnapItem
        For more information about attributes that can be set, see SnapItemViewAttributes.
        :param str snapkey: snapkey of the target SnapItem
        :param SnapItemViewAttributes attributes: class containing the desired settings
        :raises: ItemDoesNotExistError
        """ 
        raise NotImplementedError()


class ViewItemAttributes(object):
    """ Visual Attributes for Items 
    These settings may or may not be applied by the view.

    bgcolor -- the background color
    border_color -- color of the border
    label -- label text
    label_color -- color of the label text
    """
    def __init__(self):
        self.bgcolor = None
        self.border_color = None
        self.border_width = 0
        self.label = None
        self.label_color = None
        self.tooltip_text = None
        self.draw_debug = False

    def copy_attributes(self, attrs):
        """ Copies attributes from attrs to this object """
        # Subclasses may override some member values with @property methods.
        # This copies all the values of the source using whatever method is
        # implemented by the destination
        for key in attrs.__dict__:
            setattr(self, key, attrs.__dict__[key])

class BlockItemAttributes(ViewItemAttributes):
    """ Visual Attributes for BlockItems 
    These settings may or may not be applied by the view.
    spacerwidth -- distance between emitter and collector
    """
    def __init__(self):
        super(BlockItemAttributes, self).__init__()
        self.spacerwidth = None
      
class BandItemAttributes(ViewItemAttributes):
    def __init__(self):
        super(BandItemAttributes, self).__init__()
        self.width = None

class SnapItemAttributes(ViewItemAttributes):
    def __init__(self):
        super(SnapItemAttributes, self).__init__()
        self.width = None



class DuplicateItemExistsError(Exception):
    """ An Item with the specified parameters already exists """
    pass

class ItemDoesNotExistError(Exception):
    """ An Item with the specified parameters does not already exist """
    pass
