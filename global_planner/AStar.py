class Point:
    """docstring for point"""

    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y


class Node:
    def __init__(self, point, g=0, h=0):
        self.point = point  # self site
        self.father = None  # father node 
        self.g = g  # g  value
        self.h = h  # h value

    """
     manhaton algorithm
     """

    def manhattan(self, endNode):
        self.h = (abs(endNode.point.x - self.point.x) +
                  abs(endNode.point.y - self.point.y))*10

    def setG(self, g):
        self.g = g

    def setFather(self, node):
        self.father = node


class AStar:
    """
    A* algorithm
    python 2.7 
    """

    def __init__(self, map2d, startNode, endNode):
        """ 
        map2d:        to find the way array 
        startNode:    to find the starting point
        endNode:      to find the finishing point
        """
        #
        self.openList = []
        # 
        self.closeList = []
        # 
        self.map2d = map2d
        # 
        self.startNode = startNode
        # 
        self.endNode = endNode
        # 
        self.currentNode = startNode
        # 
        self.pathlist = []
        return

    def getMinFNode(self):
        """ 
        get the node which the F value is the smallest in openlist 
        """
        nodeTemp = self.openList[0]
        for node in self.openList:
            if node.g + node.h < nodeTemp.g + nodeTemp.h:
                nodeTemp = node
        return nodeTemp

    def nodeInOpenlist(self, node):
        for nodeTmp in self.openList:
            if nodeTmp.point.x == node.point.x \
                    and nodeTmp.point.y == node.point.y:
                return True
        return False

    def nodeInCloselist(self, node):
        for nodeTmp in self.closeList:
            if nodeTmp.point.x == node.point.x \
                    and nodeTmp.point.y == node.point.y:
                return True
        return False

    def endNodeInOpenList(self):
        for nodeTmp in self.openList:
            if nodeTmp.point.x == self.endNode.point.x \
                    and nodeTmp.point.y == self.endNode.point.y:
                return True
        return False

    def getNodeFromOpenList(self, node):
        for nodeTmp in self.openList:
            if nodeTmp.point.x == node.point.x \
                    and nodeTmp.point.y == node.point.y:
                return nodeTmp
        return None

    def searchOneNode(self, node):
        """ 
         search a node
         x is row
         y is column
        """
        # ignore the obstacle
        if self.map2d.isPass(node.point) != True:
            return
        # ignore the list
        if self.nodeInCloselist(node):
            return
        # G value calculation
        if abs(node.point.x - self.currentNode.point.x) == 1 and abs(node.point.y - self.currentNode.point.y) == 1:
            gTemp = 14
        else:
            gTemp = 10

        # If not in openlist, just add in openlist
        if self.nodeInOpenlist(node) == False:
            node.setG(gTemp)
            # H value calculation
            node.manhattan(self.endNode)
            self.openList.append(node)
            node.father = self.currentNode
        #  If in openlist, judge "currentNode" to current node's "G" smaller or not
        #  If smaller, recalculate g value, and change the father
        else:
            nodeTmp = self.getNodeFromOpenList(node)
            if self.currentNode.g + gTemp < nodeTmp.g:
                nodeTmp.g = self.currentNode.g + gTemp
                nodeTmp.father = self.currentNode
        return

    def searchNear(self):
        """ 
        search the node around
        search step by eight direction
        can't reach the corner directly
        (x-1,y-1)(x-1,y)(x-1,y+1)
        (x  ,y-1)(x  ,y)(x  ,y+1)
        (x+1,y-1)(x+1,y)(x+1,y+1)
        """
        if self.map2d.isPass(Point(self.currentNode.point.x - 1, self.currentNode.point.y)) and \
                self.map2d.isPass(Point(self.currentNode.point.x, self.currentNode.point.y - 1)):
            self.searchOneNode(
                Node(Point(self.currentNode.point.x - 1, self.currentNode.point.y - 1)))

        self.searchOneNode(
            Node(Point(self.currentNode.point.x - 1, self.currentNode.point.y)))

        if self.map2d.isPass(Point(self.currentNode.point.x - 1, self.currentNode.point.y)) and \
                self.map2d.isPass(Point(self.currentNode.point.x, self.currentNode.point.y + 1)):
            self.searchOneNode(
                Node(Point(self.currentNode.point.x - 1, self.currentNode.point.y + 1)))

        self.searchOneNode(
            Node(Point(self.currentNode.point.x, self.currentNode.point.y - 1)))
        self.searchOneNode(
            Node(Point(self.currentNode.point.x, self.currentNode.point.y + 1)))

        if self.map2d.isPass(Point(self.currentNode.point.x, self.currentNode.point.y - 1)) and \
                self.map2d.isPass(Point(self.currentNode.point.x + 1, self.currentNode.point.y)):
            self.searchOneNode(
                Node(Point(self.currentNode.point.x + 1, self.currentNode.point.y - 1)))

        self.searchOneNode(
            Node(Point(self.currentNode.point.x + 1, self.currentNode.point.y)))

        if self.map2d.isPass(Point(self.currentNode.point.x + 1, self.currentNode.point.y)) and \
                self.map2d.isPass(Point(self.currentNode.point.x, self.currentNode.point.y + 1)):
            self.searchOneNode(
                Node(Point(self.currentNode.point.x + 1, self.currentNode.point.y + 1)))
        return

    def start(self):
        ''''' 
        start
        '''
        # put the startnode in openlist
        self.startNode.manhattan(self.endNode)
        self.startNode.setG(0)
        self.openList.append(self.startNode)

        while True:
            # get the node which the F value is the smallest in current openlist
            # put it in closelist and remove from openlist
            self.currentNode = self.getMinFNode()
            self.closeList.append(self.currentNode)
            self.openList.remove(self.currentNode)

            self.searchNear()

            # finish or not?
            if self.endNodeInOpenList():
                nodeTmp = self.getNodeFromOpenList(self.endNode)
                while True:
                    self.pathlist.append(nodeTmp)
                    if nodeTmp.father != None:
                        nodeTmp = nodeTmp.father
                    else:
                        return True
            elif len(self.openList) == 0:
                return False
        return True

    def setMap(self):
        for node in self.pathlist:
            self.map2d.setMap(node.point)

        return
