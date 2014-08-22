from python_qt_binding.QtCore import Qt, QMimeData, QPoint, QEvent
from python_qt_binding.QtGui import QPen, QColor, QSizePolicy, QDrag, QBrush, QGraphicsWidget
from python_qt_binding.QtGui import QGraphicsView, QGraphicsAnchorLayout, QGraphicsScene
from python_qt_binding.QtGui import QFontMetrics, QToolTip, QPixmap, QImage, QPolygon
from python_qt_binding.QtCore import pyqtSignal as Signal

from diarc.snapkey import gen_snapkey, parse_snapkey
from diarc.util import typecheck, TypedDict
from diarc.view import View
from diarc.view import BlockItemAttributes
from diarc.view import BandItemAttributes
from diarc.view import SnapItemAttributes
from .SpacerContainer import SpacerContainer
import json
import sys
import logging

log = logging.getLogger('diarc.qt_view')

class QtBlockItemAttributes(BlockItemAttributes):
    def __init__(self):
        BlockItemAttributes.__init__(self)
        self.__bgcolor = "black"
        self.__border_color = "black"
        self.__label_color = "black"
        self.border_width = 1
    @property
    def bgcolor(self):
        return QColor(self.__bgcolor)
    @bgcolor.setter
    def bgcolor(self, value):
        self.__bgcolor = value
    @property
    def border_color(self):
        return QColor(self.__border_color)
    @border_color.setter
    def border_color(self, value):
        self.__border_color = value
    @property
    def label_color(self):
        return QColor(self.__label_color)
    @label_color.setter
    def label_color(self, value):
        self.__label_color = value

class QtBandItemAttributes(BandItemAttributes):
    def __init__(self):
        BandItemAttributes.__init__(self)
        self._bgcolor = "black"
        self._border_color = "black"
        self._label_color = "black"
    @property
    def bgcolor(self):
        return QColor(self._bgcolor)
    @bgcolor.setter
    def bgcolor(self, value):
        self._bgcolor = value
    @property
    def border_color(self):
        return QColor(self._border_color)
    @border_color.setter
    def border_color(self, value):
        self._border_color = value
    @property
    def label_color(self):
        return QColor(self._label_color)
    @label_color.setter
    def label_color(self, value):
        self._label_color = value

class QtSnapItemAttributes(SnapItemAttributes):
    def __init__(self):
        SnapItemAttributes.__init__(self)
        self._bgcolor = "white"
        self._border_color = "black"
        self._label_color = "black"
    @property
    def bgcolor(self):
        return QColor(self._bgcolor)
    @bgcolor.setter
    def bgcolor(self, value):
        self._bgcolor = value
    @property
    def border_color(self):
        return QColor(self._border_color)
    @border_color.setter
    def border_color(self, value):
        self._border_color = value
    @property
    def label_color(self):
        return QColor(self._label_color)
    @label_color.setter
    def label_color(self, value):
        self._label_color = value



class BandStack(SpacerContainer):
    def __init__(self, parent):
        super(BandStack, self).__init__(parent)
        self.setSizePolicy(QSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.Preferred))
        self.setMinimumWidth(15)
        self.spacerType = BandSpacer

class BandSpacer(SpacerContainer.Spacer):
    def __init__(self, parent):
        super(BandSpacer, self).__init__(parent)
        self._layout_manager = parent.parent
        self._view = parent.parent.view()
        self._adapter = parent.parent.adapter()
        self.dragOver = False
        self.setSizePolicy(QSizePolicy(QSizePolicy.Preferred, QSizePolicy.MinimumExpanding))
        self.setPreferredHeight(15)
        self.setMinimumHeight(15)
        self.setAcceptDrops(True)

    def _release(self):
        topAltitude = self.topBand.altitude if self.topBand else None
        bottomAltitude = self.bottomBand.altitude if self.bottomBand else None
        log.debug("... Releasing BandSpacer between topBand %r and botBand %r"%(topAltitude, bottomAltitude))
        super(BandSpacer, self)._release()

    @property
    def topBand(self):
        return self.itemA

    @property
    def bottomBand(self):
        return self.itemB

    def link(self):
        l = self.layout()
        # Link To your Top! If you have a band above, connect
        if self.topBand:
            l.addAnchor(self, Qt.AnchorTop, self.topBand, Qt.AnchorBottom)
        # Otherwise, if it is a positive band, connect to the top of the BandStack
        elif self.bottomBand.isPositive():
            l.addAnchor(self, Qt.AnchorTop, self.parent, Qt.AnchorTop)
        # Otherwise, if it is a negative band, connect to Block Container
        elif not self.bottomBand.isPositive(): 
            l.addAnchor(self, Qt.AnchorTop, self._layout_manager.block_container, Qt.AnchorBottom)

        # Link to your bottom! If you have a band below, connect
        if self.bottomBand:
            l.addAnchor(self, Qt.AnchorBottom, self.bottomBand, Qt.AnchorTop)
        # Otherwise, if it is a positive band, connect to the block ribbon
        elif self.topBand.isPositive():
            l.addAnchor(self, Qt.AnchorBottom, self._layout_manager.block_container, Qt.AnchorTop)
        elif not self.topBand.isPositive():
            l.addAnchor(self, Qt.AnchorBottom, self.parent, Qt.AnchorBottom)
        
        # Connect sides
        l.addAnchor(self, Qt.AnchorLeft, self.parent, Qt.AnchorLeft)
        l.addAnchor(self, Qt.AnchorRight, self.parent, Qt.AnchorRight)

    def dragEnterEvent(self,event):
        if not event.mimeData().hasText():
            event.setAccepted(False)
            return
        data = json.loads(str(event.mimeData().text()))
        if not 'band' in data:
            event.setAccepted(False)
            return
        # To know if the band being dragged is on the same side of the blockRibbon,
        # we look at the altitudes of the bands on either side of the spacer. 
        # We need to look at both because neither is guarenteed to be a real band
        # (the furthest and closest bands to the blockRibbon will each have a 
        # non existant band on one side).
        topAltitude = self.topBand.altitude if self.topBand else 0
        bottomAltitude = self.bottomBand.altitude if self.bottomBand else 0
        # Don't let bands get dragged to adjacent spots (it results in non movement)
        if data['band'] in [topAltitude, bottomAltitude]:
            event.setAccepted(False)
            return
        # Accept a positive altitude band
        if data['band'] > 0 and (topAltitude > 0 or bottomAltitude > 0):
            event.setAccepted(True)
            self.dragOver = True
            self.update()
            log.debug("Drag Positive ENTER between topBand %r and botBand %r"%(topAltitude, bottomAltitude))
        # Accept a negative altitude band
        elif data['band'] < 0 and (topAltitude < 0 or bottomAltitude < 0):
            event.setAccepted(True)
            self.dragOver = True
            self.update()
            log.debug("Drag Negative ENTER between topBand %r and botBand %r"%(topAltitude, bottomAltitude))
        else:
            event.setAccepted(False)
            return
        self.setZValue(max([self.topBand.altitude if self.topBand else None,
                            self.bottomBand.altitude if self.bottomBand else None]))

    def dragLeaveEvent(self,event):
        self.dragOver = False
        self.update()

    def dropEvent(self,event):
        self.dragOver = False
        self.update()
        """ Reorder the band altitudes to put the dropped band in the new position"""
        # Not all bands are necessarily being shown due to the fact that depending
        # on the ordering of the blocks, edge connections could travel different 
        # directions. When reordering bands, we need to take into account ALL
        # the bands (both shown and not shown). 

        # Decode the drag metadata into a dict
        data = json.loads(str(event.mimeData().text()))
        # Get the altitude of the band that was dragged
        srcAlt = data['band']
        # Get the altitudes of the bands displayed above and below this spacer.
        topAltitude = self.topBand.altitude if self.topBand else None
        bottomAltitude = self.bottomBand.altitude if self.bottomBand else None
        log.debug("Moving Band %d between topBand %r and botBand %r"%(srcAlt, topAltitude, bottomAltitude))
        self._adapter.reorder_bands(srcAlt,bottomAltitude,topAltitude)

    def paint(self,painter,option,widget):
        if self.dragOver:
            pen = QPen()
            pen.setBrush(Qt.lightGray)
            pen.setStyle(Qt.DashLine)
            painter.setPen(pen)
        else:
            painter.setPen(Qt.NoPen)
        painter.drawRect(self.rect())

class BandItem(SpacerContainer.Item, QtBandItemAttributes):
    def __init__(self, parent, altitude, rank):
        self._layout_manager = typecheck(parent, LayoutManagerWidget, "parent")
        self._view = parent.view()
        self._adapter = parent.adapter()
        super(BandItem,self).__init__(parent,parent.bandStack)
        QtBandItemAttributes.__init__(self)

        # Band properties - these must be kept up to date with topology
        self.altitude = altitude
        self._rank = rank
        self.top_band = None
        self.bot_band = None
        self.left_most_snap = None
        self.right_most_snap = None

        # Set Qt properties
        self.setContentsMargins(5,5,5,5)
        self.setSizePolicy(QSizePolicy(QSizePolicy.Preferred, QSizePolicy.MinimumExpanding))
        self.set_width(15)
        self.setAcceptHoverEvents(True)
#         self.setZValue(rank)

    def release(self):
        self.top_band = None
        self.bot_band = None
        self.left_most_snap = None
        self.right_most_snap = None
        self.setParent(None)
        self.setVisible(False)
        super(BandItem, self)._release()

    def itemA(self):
        """ Set itemA to be the topBand """
        # This is computed and assigned by the adapter prior to linking
        return self.top_band

    def itemB(self):
        """ Set itemB to be the bottomBand """
        # This is computed and assigned by the adapter prior to linking
        return self.bot_band

    def isUsed(self):
        """ Deprecated """
        return True

    @property 
    def rank(self):
        return self._rank
    @rank.setter
    def rank(self, value):
        self._rank = value
        self.setZValue(self._rank)

    def set_attributes(self, attrs):
        """ Applies a set of BandItemViewAttributes to this object. Transforms 
        some values such as color into qt specific values. """
        typecheck(attrs, BandItemAttributes, "attrs")
        self.copy_attributes(attrs)
        self.set_width(self.width)
        self.update(self.rect())

    def set_width(self, width):
        """ Sets the 'width' of the band. 
        This is actually setting the height, but is referred to as the width.
        """
        self.setPreferredHeight(width)
        self.setMinimumHeight(width)

    def isPositive(self):
        return True if self.altitude > 0 else False

    def link(self):
        sys.stdout.flush()
        # Assign the vertical anchors
        super(BandItem,self).link()
        # Assign the horizontal Anchors
        l = self.parent.layout()
        l.addAnchor(self, Qt.AnchorLeft, self.left_most_snap, Qt.AnchorLeft)
        l.addAnchor(self, Qt.AnchorRight, self.right_most_snap, Qt.AnchorRight)

    def mousePressEvent(self, event):
        """ This is necessary to capture the mouse clicking event to drag"""
        pass

    def mouseReleaseEvent(self, event):
        log.debug("Bringing band %d to front" % self.altitude)
        self._adapter.bring_band_to_front(self.altitude)

    def hoverEnterEvent(self, event):
        if self.tooltip_text:
            QToolTip.showText(event.screenPos(),self.tooltip_text)

    def hoverLeaveEvent(self, event):
        QToolTip.hideText()


    def mouseMoveEvent(self, event):
        if event.buttons() != Qt.LeftButton:
            super(BandItem,self).mouseMoveEvent(event)
        else:
            drag = QDrag(event.widget())
            mimeData = QMimeData()
            mimeData.setText(json.dumps({'band':self.altitude}))
            drag.setMimeData(mimeData)
            drag.start()

    def paint(self,painter,option,widget):
        brush = QBrush()
        brush.setStyle(Qt.SolidPattern)
        brush.setColor(self.bgcolor)
        painter.fillRect(self.rect(),brush)
        painter.setPen(self.border_color)
        painter.drawRect(self.rect())
        rect = self.geometry()
        painter.setPen(self.label_color)
        fm = painter.fontMetrics()
        elided = fm.elidedText(self.label, Qt.ElideRight, rect.width())
        twidth = fm.width(elided)
        painter.drawText((rect.width()-twidth)/2,rect.height()-2,elided)




class BlockContainer(SpacerContainer):
    def __init__(self, parent):
        super(BlockContainer,self).__init__(parent)
        self.spacerType = BlockSpacer
        self.parent = typecheck(parent,LayoutManagerWidget, "parent")
        self._view = parent.view()
        self._adapter = parent.adapter()
        self.setSizePolicy(QSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.Preferred))
        self.setMinimumWidth(15)

    def paint(self, painter, option, widget):
        painter.setPen(Qt.NoPen)
        painter.drawRect(self.rect())

class BlockSpacer(SpacerContainer.Spacer):
    def __init__(self,parent):
        super(BlockSpacer,self).__init__(parent)
        self._view = parent.parent.view()
        self._adapter = parent.parent.adapter()
        self.dragOver = False
        self.setSizePolicy(QSizePolicy(QSizePolicy.MinimumExpanding,QSizePolicy.Preferred))
        self.setPreferredWidth(50)
        self.setMinimumWidth(50)
        self.setAcceptDrops(True)

    @property
    def leftBlock(self):
        return self.itemA

    @property
    def rightBlock(self):
        return self.itemB

    def link(self):
        l = self.parent.parent.layout()
        # If you have a block to your left, connect
        if self.leftBlock:
            l.addAnchor(self, Qt.AnchorLeft, self.leftBlock, Qt.AnchorRight)
            l.addAnchor(self, Qt.AnchorTop, self.leftBlock, Qt.AnchorTop)
            l.addAnchor(self, Qt.AnchorBottom, self.leftBlock, Qt.AnchorBottom)
        # Otherwise connect to left container edge
        else:
            l.addAnchor(self, Qt.AnchorLeft, self.parent, Qt.AnchorLeft)
            l.addAnchor(self, Qt.AnchorTop, self.parent, Qt.AnchorTop)
            l.addAnchor(self, Qt.AnchorBottom, self.parent, Qt.AnchorBottom)

        # If you have a block to your right, connect
        if self.rightBlock:
            l.addAnchor(self, Qt.AnchorRight, self.rightBlock, Qt.AnchorLeft)
            l.addAnchor(self, Qt.AnchorTop, self.rightBlock, Qt.AnchorTop)
            l.addAnchor(self, Qt.AnchorBottom, self.rightBlock, Qt.AnchorBottom)
        # Otherwise connect to the right container edge
        else:
            l.addAnchor(self, Qt.AnchorRight, self.parent, Qt.AnchorRight)
            l.addAnchor(self, Qt.AnchorTop, self.parent, Qt.AnchorTop)
            l.addAnchor(self, Qt.AnchorBottom, self.parent, Qt.AnchorBottom)

    def dragEnterEvent(self,event):
        if not event.mimeData().hasText():
            event.setAccepted(False)
            return
        data = json.loads(str(event.mimeData().text()))
        if 'block' in data:
            if self.leftBlock and data['block'] == self.leftBlock.block_index:
                event.setAccepted(False)
                return
            if self.rightBlock and data['block'] == self.rightBlock.block_index:
                event.setAccepted(False)
                return
            event.setAccepted(True)
            self.dragOver = True
            log.debug("Drag ENTER")
            self.update()
        else:
            event.setAccepted(False)

    def dragLeaveEvent(self, event):
        self.dragOver = False
        self.update()

    def dropEvent(self, event):
        """ Dropping a 'block' on a BlockSpacer triggers a reordering event """
        self.dragOver = False
        self.update()
        # Dragged Index
        data = json.loads(str(event.mimeData().text()))
        if not 'block' in data:
            raise Exception("Wrong drag data type!")
        srcIdx = data['block']
        # Left Index
        lowerIdx = self.leftBlock.block_index if self.leftBlock else None
        upperIdx = self.rightBlock.block_index if self.rightBlock else None
        self._adapter.reorder_blocks(srcIdx, lowerIdx, upperIdx)

    def paint(self, painter, option, widget):
        if self.dragOver:
            pen = QPen()
            pen.setBrush(Qt.lightGray)
            pen.setStyle(Qt.DashLine)
            painter.setPen(pen)
        else:
            painter.setPen(Qt.NoPen)
        painter.drawRect(self.rect())

class BlockItem(SpacerContainer.Item, QtBlockItemAttributes):
    """ This is a QGraphicsWidget for a Diarc Block. """
    def __init__(self, parent, block_index):
        self._layout_manager = typecheck(parent, LayoutManagerWidget, "parent")
        self._view = parent.view()
        self._adapter = parent.adapter()
        super(BlockItem, self).__init__(parent, parent.block_container)
        QtBlockItemAttributes.__init__(self)

        # Qt Settings
        self.setSizePolicy(QSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.MinimumExpanding))
        self.setPreferredHeight(5)
        self.setMinimumHeight(5)
        self.setContentsMargins(5,5,5,5)

        # Properties - these values must be kept up-to-date whenever the model changes
        self.block_index = block_index
        self.left_block = None
        self.right_block = None

        # We want to have a little space above and below the Emitter/Collector,
        # Set up top and bottom margin to give that space. 
        self._topMargin = BlockItem.HorizontalSpacer(self)
        self._botMargin = BlockItem.HorizontalSpacer(self)
        self._middleSpacer = BlockItem.MiddleSpacer(self)

        # Set up Emitter and Collector Containers. They will sit "inside" the 
        # block margins
        self.myEmitter = MyEmitter(self)
        self.myCollector = MyCollector(self)

    def set_attributes(self, attrs):
        typecheck(attrs, BlockItemAttributes, "attrs")
        self.copy_attributes(attrs)
        self._middleSpacer.set_width(self.spacerwidth)
        self.update(self.rect())

    def release(self):
        super(BlockItem, self)._release()
        self.left_block = None
        self.right_block = None
        self._topMargin.release()
        self._botMargin.release()
        self._middleSpacer.release()
        self._topMargin = None
        self._botMargin = None
        self._middleSpacer = None
        self.myEmitter.release()
        self.myCollector.release()
        self.myEmitter = None
        self.myCollector = None
        self._layout_manager = None
        self._view = None
        self._adapter = None

        

    def itemA(self):
        """ We use itemA for the BlockItem to the left. """
        # Get the index to our left, and return the BlockItem with that value
        return self.left_block

    def itemB(self):
        """ We use itemB for the BlockItem to the right. """
        # Get the index to our right, and return the BlockItem with that value
        return self.right_block

    def isUsed(self):
        return True

    def link(self):
        """ Link to other objects around you. In addition to linking to other 
        blocks, we need to link to our top and bottom margins, and the emitter
        and collector containers 
        """
        # Link with other BlockItems
        super(BlockItem, self).link()
        
        l = self.parent.layout()
        # Link with top and bottom margins
        l.addAnchor(self,Qt.AnchorTop, self._topMargin, Qt.AnchorTop)
        l.addAnchor(self,Qt.AnchorLeft, self._topMargin, Qt.AnchorLeft)
        l.addAnchor(self,Qt.AnchorRight, self._topMargin, Qt.AnchorRight)
        l.addAnchor(self,Qt.AnchorBottom, self._botMargin, Qt.AnchorBottom)
        l.addAnchor(self,Qt.AnchorLeft, self._botMargin, Qt.AnchorLeft)
        l.addAnchor(self,Qt.AnchorRight, self._botMargin, Qt.AnchorRight)

        # Link with emitter and collector containers
        l.addAnchor(self._topMargin, Qt.AnchorBottom, self.myEmitter, Qt.AnchorTop)
        l.addAnchor(self._botMargin, Qt.AnchorTop, self.myEmitter, Qt.AnchorBottom)
        l.addAnchor(self._topMargin, Qt.AnchorBottom, self.myCollector, Qt.AnchorTop)
        l.addAnchor(self._botMargin, Qt.AnchorTop, self.myCollector, Qt.AnchorBottom)

        l.addAnchor(self._middleSpacer, Qt.AnchorTop, self.myEmitter, Qt.AnchorTop)
        l.addAnchor(self._middleSpacer, Qt.AnchorTop, self.myCollector, Qt.AnchorTop)
        l.addAnchor(self._middleSpacer, Qt.AnchorBottom, self.myEmitter, Qt.AnchorBottom)
        l.addAnchor(self._middleSpacer, Qt.AnchorBottom, self.myCollector, Qt.AnchorBottom)

        l.addAnchor(self, Qt.AnchorLeft, self.myCollector, Qt.AnchorLeft)
        l.addAnchor(self, Qt.AnchorRight, self.myEmitter, Qt.AnchorRight)
        l.addAnchor(self.myCollector, Qt.AnchorRight, self._middleSpacer, Qt.AnchorLeft)
        l.addAnchor(self._middleSpacer, Qt.AnchorRight, self.myEmitter, Qt.AnchorLeft)

    def mousePressEvent(self, event):
        pass

    def mouseMoveEvent(self, event):
        """ Creates a drag event with the block information """
        if event.buttons() != Qt.LeftButton:
            super(BlockItem, self).mouseMoveEvent(event)
        else:
            drag = QDrag(event.widget())
            mimeData = QMimeData()
            mimeData.setText(json.dumps({'block': self.block_index}))
            drag.setMimeData(mimeData)
            drag.start()

    def paint(self,painter,option,widget):
        border_pen = QPen()
        border_pen.setBrush(self.border_color)
        border_pen.setStyle(Qt.SolidLine)
        border_pen.setWidth(self.border_width)
        painter.setPen(border_pen)
        painter.drawRect(self.rect())

    class MiddleSpacer(QGraphicsWidget):
        def __init__(self,parent):
            super(BlockItem.MiddleSpacer,self).__init__(parent=parent)
            # NOTE: I originally tried to set this to Preferred, MinimumExpanding
            # but ran into trouble when the layout could not compute itself when
            # it set that mode. For some reason, it was very unhappy about giving
            # it height policy information.
            self.blockItem = parent
            self.setSizePolicy(QSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred))
            # The width of the block
            self.set_width(20)
            self.setAcceptHoverEvents(True)

        def set_width(self, width):
            self.setPreferredWidth(width)
            self.setMinimumWidth(width)

        def release(self):
            self.setParent(None)
            self.blockItem = None

        def hoverEnterEvent(self, event):
            if self.blockItem.tooltip_text:
                QToolTip.showText(event.screenPos(),self.blockItem.tooltip_text)

        def hoverLeaveEvent(self, event):
            QToolTip.hideText()

#         def event(self, event):
#             if event.type() == QEvent.ToolTip:
#             QToolTip.showText(event.screenPos(),self.label+" ToolTip!")
#         else:
#             QToolTip.hideText()
#             event.ignore()
#         return super(BlockItem, self).event(event)

        def paint(self,painter,option,widget):
            painter.setPen(Qt.NoPen)
            painter.drawRect(self.rect())
            painter.setPen(self.blockItem.label_color)
            painter.rotate(-90)
            rect = self.rect()
            fm = painter.fontMetrics()
            elided = fm.elidedText(self.blockItem.label, Qt.ElideRight, rect.height()-2)
            twidth = fm.width(elided)
            painter.drawText(-twidth-(rect.height()-twidth)/2,rect.width()-2,elided)

    class HorizontalSpacer(QGraphicsWidget):
        def __init__(self,parent):
            super(BlockItem.HorizontalSpacer,self).__init__(parent=parent)
            self.setSizePolicy(QSizePolicy(QSizePolicy.MinimumExpanding,QSizePolicy.Preferred))
            self.setPreferredHeight(0)
            self.setMinimumHeight(0)
            self.setMaximumHeight(0)
            self.setMinimumWidth(5)

        def release(self):
            self.setParent(None)

class SnapContainer(SpacerContainer):
    def __init__(self,parent):
        super(SnapContainer, self).__init__(parent.parent)
        self.parentBlock = typecheck(parent, BlockItem, "parent")

        self.spacerType = SnapSpacer
        self.setSizePolicy(QSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.Preferred))
        self.setMinimumWidth(1)
        self.setPreferredWidth(1)
    
    def release(self):
        super(SnapContainer, self)._release()
        self.parentBlock = None

    def strType(self):
        """ prints the container type as a string """
        return "emitter" if isinstance(self,MyEmitter) else "collector" if isinstance(self,MyCollector) else "unknown"

    def paint(self,painter,option,widget):
        painter.setPen(Qt.green)
        painter.drawRect(self.rect())
    

class MyEmitter(SnapContainer):
    def paint(self, painter, option, widget):
        if self.parentBlock.draw_debug:
            painter.setPen(Qt.green)
            painter.drawRect(self.rect())
   
class MyCollector(SnapContainer):
    def paint(self, painter, option, widget):
        if self.parentBlock.draw_debug:
            painter.setPen(Qt.blue)
            painter.drawRect(self.rect())
 

class SnapSpacer(SpacerContainer.Spacer):
    def __init__(self,parent):
        super(SnapSpacer,self).__init__(parent)
        self._layout_manager = parent.parent
        self._view = parent.parent.view()
        self._adapter = parent.parent.adapter()

        self.dragOver = False
        
        # Qt Properties
        self.setSizePolicy(QSizePolicy(QSizePolicy.MinimumExpanding,QSizePolicy.Preferred))
        self.collapseWidth()
        self.setAcceptDrops(True)

    @property
    def leftSnap(self):
        return self.itemA

    @property
    def rightSnap(self):
        return self.itemB

    def link(self):
        l = self.parent.parent.layout()
        # If you have a snap to your left, connect
        if self.leftSnap:
            l.addAnchor(self, Qt.AnchorLeft, self.leftSnap, Qt.AnchorRight)
            l.addAnchor(self, Qt.AnchorTop, self.leftSnap, Qt.AnchorTop)
            l.addAnchor(self, Qt.AnchorBottom, self.leftSnap, Qt.AnchorBottom)
        # Otherwise connect to left container edge
        else:
            l.addAnchor(self, Qt.AnchorLeft, self.parent, Qt.AnchorLeft)
            l.addAnchor(self, Qt.AnchorTop, self.parent, Qt.AnchorTop)
            l.addAnchor(self, Qt.AnchorBottom, self.parent, Qt.AnchorBottom)
        # if you have a snap to your right, connect
        if self.rightSnap:
            l.addAnchor(self, Qt.AnchorRight, self.rightSnap, Qt.AnchorLeft)
            l.addAnchor(self, Qt.AnchorTop, self.rightSnap, Qt.AnchorTop)
            l.addAnchor(self, Qt.AnchorBottom, self.rightSnap, Qt.AnchorBottom)
        # Otherwise connect to the right container edge
        else:
            l.addAnchor(self, Qt.AnchorRight, self.parent, Qt.AnchorRight)
            l.addAnchor(self, Qt.AnchorTop, self.parent, Qt.AnchorTop)
            l.addAnchor(self, Qt.AnchorBottom, self.parent, Qt.AnchorBottom)

    def dragEnterEvent(self, event):
        """ Decides whether or not the information being dragged can be placed here"""
        if not event.mimeData().hasText():
            event.setAccepted(False)
            return
        data = json.loads(str(event.mimeData().text()))
        if not set(['block', 'container', 'snap']).issubset(set(data.keys())):
            event.setAccepted(False)
            return
        if not data['block'] == self.parent.parentBlock.block_index:
            event.setAccepted(False)
            return
        if not self.parent.strType() == data['container']:
            event.setAccepted(False)
            return
        if self.leftSnap and data['snap'] == self.leftSnap.snap_order:
            event.setAccepted(False)
            return
        if self.rightSnap and data['snap'] == self.rightSnap.snap_order:
            event.setAccepted(False)
            return
        event.setAccepted(True)
        self.expandWidth()
        self.dragOver = True
        self.update()

    def dragLeaveEvent(self, event):
        self.dragOver = False
        self.collapseWidth()
        self.update()

    def dropEvent(self,event):
        """ Relocates a MySnap to this position """
        self.collapseWidth()
        self.dragOver = False
        data = json.loads(str(event.mimeData().text()))
        assert(data['block'] == self.parent.parentBlock.block_index)
        assert(data['container'] == self.parent.strType())

        srcIdx = data['snap']
        lowerIdx = self.leftSnap.snap_order if self.leftSnap else None
        upperIdx = self.rightSnap.snap_order if self.rightSnap else None
        self._adapter.reorder_snaps(data['block'], data['container'], srcIdx, lowerIdx, upperIdx)

    def paint(self, painter, option, widget):
        if self.dragOver:
            brush = QBrush()
            brush.setStyle(Qt.Dense4Pattern)
            brush.setColor(Qt.yellow)
            painter.fillRect(self.rect(),brush)
            pen = QPen()
            pen.setBrush(Qt.darkGray)
            pen.setStyle(Qt.DotLine)
            pen.setWidth(1)
            painter.setPen(pen)
            painter.drawRect(self.rect())
            rect = self.geometry()
        else:
            painter.setPen(Qt.NoPen)
#             painter.setPen(Qt.magenta)
            painter.drawRect(self.rect())

    def collapseWidth(self):
        """ Make the width very small """
        self.setMinimumWidth(0.1)
        self.setMaximumWidth(0.1)
        self.setPreferredWidth(0.1)

    def expandWidth(self):
        """ Widen the spacer to afford seperation """
        self.setMinimumWidth(10)
        self.setMaximumWidth(10)
        self.setPreferredWidth(10)



class SnapItem(SpacerContainer.Item, QtSnapItemAttributes):
    def __init__(self, parent, snapkey):
        QtSnapItemAttributes.__init__(self)
        block_index, container_name, snap_order = parse_snapkey(snapkey)
        self._snapkey = snapkey
        self._layout_manager = typecheck(parent, LayoutManagerWidget, "parent")
        self._view = parent.view()
        self._adapter = parent.adapter()

        assert(container_name in ["emitter","collector"])
        self.block_index = block_index
        self.snap_order = snap_order
        self.block_item = self._layout_manager.get_block_item(block_index)
        self.container = self.block_item.myEmitter if container_name == "emitter" else self.block_item.myCollector
        # SnapItems to the left and to the right - populated by the adapter
        self.left_snap = None
        self.right_snap = None
        # Positive and Negative BandItems this snap connects to - only exists if
        # the band is being used - populated by the adapter
        self.posBandItem = None
        self.negBandItem = None
        super(SnapItem,self).__init__(parent,self.container)

        # Qt Properties
        self.setSizePolicy(QSizePolicy(QSizePolicy.Preferred,QSizePolicy.Preferred))
        self.set_width(20)
        # These values dictate the block Container Height
        self.setPreferredHeight(150)
        self.setMaximumHeight(150)

        #Create two SnapBandLinks - one for each band
        _is_source = True if container_name == "emitter" else False
        self.upLink = SnapBandLink(None, is_uplink=True, is_source=_is_source)
        self.downLink = SnapBandLink(None,is_source=_is_source)
 
    def release(self):
        self.left_snap = None
        self.right_snap = None
        self.upLink.setParent(None)
        self.downLink.setParent(None)
        self.upLink = None
        self.downLink = None
        self.setVisible(False)
        super(SnapItem, self)._release()


    def itemA(self):
        """ We use itemA for the SnapItem to the left """
        return self.left_snap

    def itemB(self):
        """ We use itemB for the SnapItem to the right """
        # When we don't display unused snaps, we are still reporting unused snaps to 
        # our left and right here - only they don't visually exists which causes problems
        return self.right_snap

    def isUsed(self):
        """ Deprecated """
        return True

    def set_attributes(self, attrs):
        typecheck(attrs, SnapItemAttributes, "attrs")
        self.copy_attributes(attrs)
        self.set_width(attrs.width)
        self.update(self.rect())

    def set_width(self, width):
        self.width = width
        self.setPreferredWidth(width)

    def link(self):
        super(SnapItem, self).link()
        l = self.parent.layout()
        l.addAnchor(self, Qt.AnchorTop, self.container, Qt.AnchorTop)
        l.addAnchor(self, Qt.AnchorBottom, self.container, Qt.AnchorBottom)

        #Connect bandlinks
        if self.posBandItem:
            self.upLink.setVisible(True)
            self.upLink.setZValue(self.posBandItem.rank+0.5)
            l.addAnchor(self, Qt.AnchorTop, self.upLink, Qt.AnchorBottom)
            l.addAnchor(self.posBandItem, Qt.AnchorBottom, self.upLink, Qt.AnchorTop)
            l.addAnchor(self, Qt.AnchorLeft, self.upLink, Qt.AnchorLeft)
            l.addAnchor(self, Qt.AnchorRight, self.upLink, Qt.AnchorRight)
            self.upLink.bgcolor = self.posBandItem.bgcolor
            self.upLink.border_color = self.posBandItem.border_color
            self.upLink.label_color = self.posBandItem.label_color
        else:
            self.upLink.setVisible(False)
            self.upLink.setParent(None)

        if self.negBandItem:
            self.downLink.setVisible(True)
            self.downLink.setZValue(self.negBandItem.rank+0.5)
            l.addAnchor(self, Qt.AnchorBottom, self.downLink, Qt.AnchorTop)
            l.addAnchor(self.negBandItem, Qt.AnchorTop, self.downLink, Qt.AnchorBottom)
            l.addAnchor(self, Qt.AnchorLeft, self.downLink, Qt.AnchorLeft)
            l.addAnchor(self, Qt.AnchorRight, self.downLink, Qt.AnchorRight)
            self.downLink.bgcolor = self.negBandItem.bgcolor
            self.downLink.border_color = self.negBandItem.border_color
            self.downLink.label_color = self.negBandItem.label_color
        else:
            self.downLink.setVisible(False)
            self.downLink.setParent(None)

    def mousePressEvent(self, event):
        """ Captures the mouse press event for dragging """
        pass

    def mouseReleaseEvent(self, event):
        if self.posBandItem:
            self._adapter.bring_band_to_front(self.posBandItem.altitude)
        if self.negBandItem:
            self._adapter.bring_band_to_front(self.negBandItem.altitude)

    def mouseMoveEvent(self, event):
        if event.buttons() != Qt.LeftButton:
            super(SnapItem,self).mouseMoveEvent(event)
        else:
            drag = QDrag(event.widget())
            mimeData = QMimeData()
            mimeData.setText(json.dumps({'block': self.block_index,'container': self.container.strType(),'snap':self.snap_order}))
            drag.setMimeData(mimeData)
            drag.start()

    def paint(self, painter, option, widget):
        # Paint background
        brush = QBrush()
        brush.setStyle(Qt.SolidPattern)
        brush.setColor(self.bgcolor)
        painter.fillRect(self.rect(),brush)
        # Paint border
        border_pen = QPen()
        border_pen.setBrush(self.border_color)
        border_pen.setStyle(Qt.SolidLine)
        border_pen.setWidth(self.border_width)
        painter.setPen(border_pen)
        painter.drawRect(self.rect())
        rect = self.geometry()
        painter.setPen(self.label_color)
        if self.draw_debug:
            if self.posBandItem:
                painter.drawText(6,12,str(self.posBandItem.altitude))
            if self.negBandItem:
                painter.drawText(3,rect.height()-3,str(self.negBandItem.altitude))
#         painter.drawText(2,rect.height()/2+4,self.label)
        painter.rotate(-90)
        fm = painter.fontMetrics()
        elided = fm.elidedText(self.label, Qt.ElideRight, rect.height())
        twidth = fm.width(elided)
        painter.drawText(-twidth-(rect.height()-twidth)/2,rect.width()-2,elided)


class SnapBandLink(QGraphicsWidget, QtBandItemAttributes):
    def __init__(self,parent, is_uplink=False, is_source=False):
        super(SnapBandLink,self).__init__(parent=parent)
        QtBandItemAttributes.__init__(self)
        self.setVisible(False)
        self.setSizePolicy(QSizePolicy(QSizePolicy.MinimumExpanding,QSizePolicy.MinimumExpanding))
        self.setPreferredWidth(5)
        self.setMinimumWidth(5)
        self.setPreferredHeight(5)
        self.setMinimumHeight(5)

        self._is_uplink = is_uplink
        self._is_source = is_source

 
    def paint(self,painter,option,widget):
        brush = QBrush()
        brush.setStyle(Qt.SolidPattern)
        brush.setColor(self.bgcolor)
        painter.fillRect(self.rect(),brush)
        pen = QPen()
        pen.setBrush(self.border_color)
        pen.setStyle(Qt.DashLine)
        rect = self.rect()
        painter.setPen(pen)
        painter.drawRect(rect)
        # Create arrows
        arrow_scale = 0.5
        arrow_width = rect.width()*arrow_scale
        arrow_height = arrow_width * 0.8
        arrow_margin = (rect.width()-arrow_width)/2.0

        brush.setColor(self.label_color)
        painter.setPen(Qt.NoPen)
        painter.setBrush(brush)
        arrow = None
        # Determine which direction to draw arrow
        if (self._is_uplink and self._is_source) or (not self._is_uplink and not self._is_source):
            # Draw pointing up
            arrow = QPolygon([QPoint(0,arrow_height), QPoint(arrow_width,arrow_height), QPoint(arrow_width/2.0,0)])
        else:
            # Draw pointing down
            arrow = QPolygon([QPoint(0,0), QPoint(arrow_width,0), QPoint(arrow_width/2.0,arrow_height)])

        # Determine which side to draw arrow on
        if self._is_uplink:
            arrow.translate(rect.x()+arrow_margin,rect.y()+rect.height()-arrow_height-arrow_margin)
        else:
            arrow.translate(rect.x()+arrow_margin,rect.y()+arrow_margin)
        painter.drawPolygon(arrow)

class LayoutManagerWidget(QGraphicsWidget):
    """ Holds the actual qt anchoredlayout and top level SpacerContainers """
    def __init__(self, view):
        super(LayoutManagerWidget, self).__init__(parent=None)
        self._view = view
        self.resize(0,0)
        
        # Top Level Layout Containers
        self.block_container = BlockContainer(self)
        self.bandStack = BandStack(self)

        # Visual Object we are tracking
        self._block_items = TypedDict(int,BlockItem)  # index    #TypedList(BlockItem)
        self._band_items = TypedDict(int,BandItem)    # altitude    #TypedList(BandItem)
        self._snap_items = TypedDict(str,SnapItem)  # snapkey  #TypedList(SnapItem)

    def add_block_item(self, index):
        log.debug("... Adding BlockItem %d"%index)
        """ create a new BlockItem """
        if index in self._block_items:
            raise DuplicateItemExistsError("Block Item with index %d already exists"%(index))
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
        self._block_items[index].set_attributes(attributes)

    def remove_block_item(self, index):
        log.debug("... Removing BlockItem %d"%index)
        self._block_items[index].release()
        self._block_items.pop(index)

    def get_block_item(self, index):
        """ Returns a BlockItem with specified index """
        return self._block_items[index]

    def add_band_item(self, altitude, rank):
        """ Create a new drawable object to correspond to a Band. """
        log.debug("... Adding BandItem with altitude %d"%altitude)
        if altitude in self._band_items:
            raise DuplicateItemExistsError("BandItem with altitude %d already exists"%(altitude))
        item = BandItem(self, altitude, rank)
        self._band_items[altitude] = item
        return item

    def has_band_item(self, altitude):
        return True if altitude in self._band_items else False

    def remove_band_item(self, altitude):
        """ Remove the drawable object to correspond to a band """ 
        log.debug("... Removing BandItem altitude %d"%altitude)
        self._band_items[altitude].release()
        self._band_items.pop(altitude)

    def get_band_item(self, altitude):
        return self._band_items[altitude]
    
    def set_band_item_settings(self, altitude, rank,
                                top_band_alt, bot_band_alt,
                                leftmost_snapkey, rightmost_snapkey):
        item = self._band_items[altitude]
        item.rank = rank
        item.top_band = self._band_items[top_band_alt] if top_band_alt is not None else None
        item.bot_band = self._band_items[bot_band_alt] if bot_band_alt is not None else None
        item.left_most_snap = self._snap_items[str(leftmost_snapkey)]
        item.right_most_snap = self._snap_items[str(rightmost_snapkey)]

    def set_band_item_attributes(self, altitude, attrs):
        self._band_items[altitude].set_attributes(attrs)

    def add_snap_item(self, snapkey):
        # snapkey gets passed as a QString automatically since it goes across
        # a signal/slot interface
        snapkey = str(snapkey)
        log.debug("... Adding SnapItem %s"%snapkey)
        if snapkey in self._snap_items:
            raise DuplicateItemExistsError("SnapItem with snapkey %s already exists"%(snapkey))
        item = SnapItem(self, snapkey)
        self._snap_items[snapkey] = item
        return item

    def remove_snap_item(self, snapkey):
        # snapkey gets passed as a QString automatically since it goes across
        # a signal/slot interface
        snapkey = str(snapkey)
        log.debug("... Removing SnapItem %s"%snapkey)
        self._snap_items[snapkey].release()
        self._snap_items.pop(snapkey)

    def has_snap_item(self, snapkey):
        return True if snapkey in self._snap_items else False

    def get_snap_items(self, snapkey):
        # snapkey gets passed as a QString automatically since it goes across
        # a signal/slot interface
        snapkey = str(snapkey)
        return self._snap_item[snapkey]

    def set_snap_item_settings(self, snapkey, left_order, right_order, pos_band_alt, neg_band_alt):
        # snapkey gets passed as a QString automatically since it goes across
        # a signal/slot interface
        snapkey = str(snapkey)
        item = self._snap_items[snapkey]
        if left_order is not None:
            left_snapkey = gen_snapkey(item.block_index,item.container.strType(),left_order)
            item.left_snap = self._snap_items[left_snapkey]
        else:
            item.left_snap = None
        if right_order is not None:
            right_snapkey = gen_snapkey(item.block_index,item.container.strType(),right_order)
            item.right_snap = self._snap_items[right_snapkey]
        else:
            item.right_snap = None
        item.posBandItem = self._band_items[pos_band_alt] if pos_band_alt is not None else None
        item.negBandItem = self._band_items[neg_band_alt] if neg_band_alt is not None else None

    def set_snap_item_attributes(self, snapkey, attributes):
        # snapkey gets passed as a QString automatically since it goes across
        # a signal/slot interface
        snapkey = str(snapkey)
        self._snap_items[snapkey].set_attributes(attributes)

    def view(self):
        return self._view

    def adapter(self):
        return self._view.adapter

    def link(self):
        log.debug("*** Begining Linking ***")
        sys.stdout.flush()
        # Create a new anchored layout. Until I can figure out how to remove
        # objects from the layout, I need to make a new one each time
        l = QGraphicsAnchorLayout()
        l.setSpacing(0.0)
        self.setLayout(l)

        # Anchor BandStack to Layout, and BlockContainer to BandStack
        self.layout().addAnchor(self.block_container, Qt.AnchorTop, self.layout(), Qt.AnchorTop)
        self.layout().addAnchor(self.block_container, Qt.AnchorLeft, self.layout(), Qt.AnchorLeft)
        self.layout().addAnchor(self.bandStack, Qt.AnchorLeft, self.block_container, Qt.AnchorLeft)
        self.layout().addAnchor(self.bandStack, Qt.AnchorRight, self.block_container, Qt.AnchorRight)

        # Link block items
        for item in self._block_items.values():
            item.link()

        # Link band items
        for item in self._band_items.values():
            item.link()

        # Link Snap Items
        for item in self._snap_items.values():
            item.link()

        log.debug("*** Finished Linking ***\n")
        sys.stdout.flush()

#     def mousePressEvent(self, event):
#         print "updating model"
#         self.adapter().update_model()
# 
#     def paint(self, painter, option, widget):
#         painter.setPen(Qt.blue)
#         painter.drawRect(self.rect())


class QtView(QGraphicsView, View):
    """ This is a Qt based stand-alone widget that provides a visual rendering 
    of a Topology. It provides a window into a self contained GraphicsScene in
    which we draw the topology. 
    It also implements the View interface as a passthrough to the LayoutManager.
    """
    # Qt Signals. The following signals correspond to diarc.View() API calls that
    # are called from outside the main qt thread. Rather then call the implementations
    # defined in layout_manager directly, we call them from these signals so that
    # the call happens from the correct thread.
    __update_view_signal = Signal()

    __add_block_item_signal = Signal(int)
    __remove_block_item_signal = Signal(int)
    __set_block_item_settings_signal = Signal(int, object, object)
    __set_block_item_attributes_signal = Signal(int, BlockItemAttributes)

    __add_band_item_signal = Signal(int, int)
    __remove_band_item_signal = Signal(int)
    __set_band_item_settings_signal = Signal(int, int, object, object, str, str)
    __set_band_item_attributes_signal = Signal(int, BandItemAttributes)

    __add_snap_item_signal = Signal(str)
    __remove_snap_item_signal = Signal(str)
    __set_snap_item_settings_signal = Signal(str, object, object, object, object)
    __set_snap_item_attributes_signal = Signal(str, SnapItemAttributes)

    def __init__(self):
        super(QtView, self).__init__(None)
        View.__init__(self)

        # Qt properties - Enable click-n-drag paning and initialize Scene
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setScene(QGraphicsScene(self))

        # Enable for debuging
        self.setViewportUpdateMode(QGraphicsView.FullViewportUpdate)
        
        # Add the LayoutManagerWidget to the scene
        self.layout_manager = LayoutManagerWidget(self)
        self.scene().addItem(self.layout_manager)

        # Hook up the signals and slots
        self.__update_view_signal.connect(self.layout_manager.link)
        self.__add_block_item_signal.connect(self.layout_manager.add_block_item)
        self.__remove_block_item_signal.connect(self.layout_manager.remove_block_item)
        self.__set_block_item_settings_signal.connect(self.layout_manager.set_block_item_settings)
        self.__set_block_item_attributes_signal.connect(self.layout_manager.set_block_item_attributes)
        self.__add_band_item_signal.connect(self.layout_manager.add_band_item)
        self.__remove_band_item_signal.connect(self.layout_manager.remove_band_item)
        self.__set_band_item_settings_signal.connect(self.layout_manager.set_band_item_settings)
        self.__set_band_item_attributes_signal.connect(self.layout_manager.set_band_item_attributes)
        self.__add_snap_item_signal.connect(self.layout_manager.add_snap_item)
        self.__remove_snap_item_signal.connect(self.layout_manager.remove_snap_item)
        self.__set_snap_item_settings_signal.connect(self.layout_manager.set_snap_item_settings)
        self.__set_snap_item_attributes_signal.connect(self.layout_manager.set_snap_item_attributes)
        self.resize(1024,768)
        QColor.setAllowX11ColorNames(True)
        if not QColor.allowX11ColorNames():
            rospy.logwarn("Coloring will not work properly")
        self.show()

    def update_view(self):
        self.__update_view_signal.emit()

    def add_block_item(self, index):
        """ Allows the adapter to create a new BlockItem """
        self.__add_block_item_signal.emit(index)

    def has_block_item(self, index):
        return self.layout_manager.has_block_item(index)

    def remove_block_item(self, index):
        self.__remove_block_item_signal.emit(index)

    def set_block_item_settings(self, index, left_index, right_index):
        return self.__set_block_item_settings_signal.emit(index, left_index, right_index)

    def set_block_item_attributes(self, index, attributes):
        self.__set_block_item_attributes_signal.emit(index, attributes)

    def add_band_item(self, altitude, rank):
        """ Create a new drawable object to correspond to a Band. """
        self.__add_band_item_signal.emit(altitude, rank)

    def has_band_item(self, altitude):
        return self.layout_manager.has_band_item(altitude)

    def remove_band_item(self, altitude):
        """ Remove the drawable object to correspond to a band """ 
        self.__remove_band_item_signal.emit(altitude)

    def set_band_item_settings(self, altitude, rank, top_band_alt, bot_band_alt,
                                leftmost_snapkey, rightmost_snapkey):
        self.__set_band_item_settings_signal.emit(altitude, rank, top_band_alt, bot_band_alt, leftmost_snapkey, rightmost_snapkey)

    def set_band_item_attributes(self, altitude, attributes):
        self.__set_band_item_attributes_signal.emit(altitude, attributes)

    def add_snap_item(self, snapkey):
        self.__add_snap_item_signal.emit(snapkey)

    def has_snap_item(self, snapkey):
        return self.layout_manager.has_snap_item(snapkey)

    def remove_snap_item(self, snapkey): 
        self.__remove_snap_item_signal.emit(snapkey)

    def set_snap_item_settings(self, snapkey, left_order, right_order, pos_band_alt, neg_band_alt):
        self.__set_snap_item_settings_signal.emit(snapkey, left_order, right_order, pos_band_alt, neg_band_alt)

    def set_snap_item_attributes(self, snapkey, attributes):
        self.__set_snap_item_attributes_signal.emit(snapkey, attributes)

    def wheelEvent(self,event):
        """ Implements scrollwheel zooming """
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        scaleFactor = 1.15
        if event.delta() > 0:
            self.scale(scaleFactor, scaleFactor)
        else:
            self.scale(1.0/scaleFactor, 1.0/scaleFactor)







class DuplicateItemExistsError(Exception):
    """ An Item with the specified parameters already exists """
    pass
