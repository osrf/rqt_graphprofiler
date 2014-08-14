#!/usr/bin/python

class CharGrid(dict):
    """ Grid of single characters for drawing ascii art.
    Keys are (row,col) """
    def __init__(self):
        self.maxRow = 0  # total number of rows
        self.maxCol = 0  # total number of cols
        self._defaultVal = ' '

    def __checkkey(self,key):
        """ Make sure key is a valid key """
        if not isinstance(key,tuple):
            raise Exception("Key must be 2-tuple, got %r"%key)
        if not len(key) == 2:
            raise Exception("Key must be 2-tuple, got %d-tuple"%len(key))
        if not (isinstance(key[0],int) and isinstance(key[1],int)):
            raise Exception("Key value must be of type (int,int), got (%s,%s)"%(key[0].__class__.__name__,key[1].__class__.__name__))

    def __getitem__(self,key):
        """ Get a single character from the grid """
        self.__checkkey(key)
        # If value does not already exist - send back default value
        # Increase our 'size' to report if necessary
        # Don't add it though - this unnecessarily increases the size
        if not key in self:
            self.maxRow = max(key[0],self.maxRow)
            self.maxCol = max(key[1],self.maxCol)
            return self._defaultVal
        return super(CharGrid,self).__getitem__(key)

    def __setitem__(self,key,val):
        """ Set a single character or string into the grid starting at key"""
        self.__checkkey(key)
        if not isinstance(val,str):
            raise Exception("Val must be 'str', got %r"%val)
        if len(val) < 1:
            raise Exception("Val must be at least 1 character long, got %d (%s)"%(len(val),val))

        cOffset = 0
        for c in val:
            row = key[0]
            col = key[1]+cOffset
            # Update size values if necessary
            if not (row,col) in self:
                self.maxRow = max(row,self.maxRow)
                self.maxCol = max(col,self.maxCol)
            # Set value
            super(CharGrid,self).__setitem__((row,col),c)
            cOffset += 1

    def insertRowsAbove(self,row,num):
        """ add a new row above 'row' (shifting existing rows down) """
        keys = filter(lambda k: k[0] >= row,self.keys())
        self.__moveCells(keys,(num,0))

    def insertColsToLeft(self,col,num):
        """ add a new column to the left of 'col' (shifting existing cols right """
        keys = filter(lambda k: k[1] >= col,self.keys())
        self.__moveCells(keys,(0,num))

    def __moveCells(self,keys,direction):
        """ Called by insertRowAbove...
        Moves all cells in 'keys' in 'direction'.
        Each key is 'keys' specified by (row,col). Direction specified by 
        (rowOffset,colOffset).
        """
        while len(keys) > 0:
            keys = self.__moveCell(keys[0],keys,direction)

    def __moveCell(self,srcKey,keys,direction):
        """ Called by __moveCells - recursively moves cells to move srcKey cell """
        self.__checkkey(srcKey)
        self.__checkkey(direction)
        destKey = (srcKey[0]+direction[0],srcKey[1]+direction[1])
        # If destination already exists
        if destKey in self:
            keys = self.__moveCell(destKey,keys,direction)
        # copy contents and pop key from self and tmp keylist
        self[destKey] = self[srcKey]
        self.pop(srcKey)
        keys.pop(keys.index(srcKey))
        return keys


    def __str__(self):
        imgbuf = ""
        for r in range(self.maxRow+1):
            rowbuf = list()
            for c in range(self.maxCol+1):
                rowbuf.append(self[(r,c)])
            imgbuf+= "".join(rowbuf)+"\n"
        return imgbuf
