#!/usr/bin/env python

# Copyright (c) 2014 CNRS
# Author: Florent Lamiraux
#
# This file is part of hpp-manipulation-corba.
# hpp-manipulation-corba is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp-manipulation-corba is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-manipulation-corba.  If not, see
# <http://www.gnu.org/licenses/>.

from xml.dom.minidom import parse

def isFloat (string):
    try:
        float (string)
        return True
    except ValueError:
        return False

class Parser (object):
    """
    Parser of srdf file that looks for object handles.
    """
    def __init__ (self, client, filename):
        """
        Constructor takes a client to hpp-manipulation corba server.
        """
        self.filename
        self.client = client
        self.tree = parse (filename)

    def parseHandles (self):
        nodeList = self.tree.getElementsByTagName ('robot')
        if nodeList.length is 0:
            raise RuntimeError ("file " +
                                self.filename + ' has no node "robot"')
        robotElement = nodeList [0]
        handles = robotElement.getElementsByTagName ('handle')
        for h in handles:
            name = h.attributes ['name'].nodeValue
            localPositions = h.getElementsByTagName ('local_position')
            if not localPositions.length is 1:
                raise RuntimeError ('expected 1 tag "local_position", ' +
                                    'but found %i.'%localPositions.length)
            localPosition = localPositions [0]
            position = localPosition.childNodes [0].nodeValue
            localPosition = map (float, (filter (isFloat,
                                                 position.split (' '))))
            links = h.getElementsByTagName ('link')
            if not links.length is 1:
                raise RuntimeError ('expected 1 tag "link", ' +
                                    'but found %i.'%links.length)
            link = links [0]
            linkName = link.attributes ['name'].nodeValue

            client.addHandle (self.objectName, )
