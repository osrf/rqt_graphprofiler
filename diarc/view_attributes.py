class BlockItemViewAttributes(object):
    """ Visual Attributes for BlockItems 
    These settings may or may not be applied by the view.

    bgcolor -- the background color
    border_color -- color of the border
    label -- label text
    label_color -- color of the label text
    spacerwidth -- distance between emitter and collector
    """
    def __init__(self):
        self.bgcolor = None
        self.border_color = None
        self.border_width = None
        self.label = None
        self.label_color = None
        self.label_rotation = 0
        self.spacerwidth = None

    def _copy_attributes(self, attrs):
        self.__dict__.update(attrs.__dict__)

      
class BandItemViewAttributes(object):
    def __init__(self):
        self.bgcolor = None
        self.border_color = None
        self.label = None
        self.label_color = None
        self.width = None

    def _copy_attributes(self, attrs):
        self.__dict__.update(attrs.__dict__)

class SnapItemViewAttributes(object):
    def __init__(self):
        self.bgcolor = None
        self.border_color = None
        self.border_width = None
        self.label = None
        self.label_color = None
        self.width = None

    def _copy_attributes(self, attrs):
        self.__dict__.update(attrs.__dict__)


