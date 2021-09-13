class Point:
    """docstring for point"""

    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y


class Node:
    def __init__(self, point, g=0, h=0):
        self.point = point  # 自己的座標
        self.father = None  # 父節點
        self.g = g  # g值
        self.h = h  # h值

    """
    估價公式：曼哈頓演算法
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
    A* 演算法 
    python 2.7 
    """

    def __init__(self, map2d, startNode, endNode):
        """ 
        map2d:      尋路陣列 
        startNode:  尋路起點 
        endNode:    尋路終點 
        """
        # 開放列表
        self.openList = []
        # 封閉列表
        self.closeList = []
        # 地圖資料
        self.map2d = map2d
        # 起點
        self.startNode = startNode
        # 終點
        self.endNode = endNode
        # 當前處理的節點
        self.currentNode = startNode
        # 最後生成的路徑
        self.pathlist = []
        return

    def getMinFNode(self):
        """ 
        獲得openlist中F值最小的節點 
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
        搜尋一個節點
        x為是行座標
        y為是列座標
        """
        # 忽略障礙
        if self.map2d.isPass(node.point) != True:
            return
        # 忽略封閉列表
        if self.nodeInCloselist(node):
            return
        # G值計算
        if abs(node.point.x - self.currentNode.point.x) == 1 and abs(node.point.y - self.currentNode.point.y) == 1:
            gTemp = 14
        else:
            gTemp = 10

        # 如果不再openList中，就加入openlist
        if self.nodeInOpenlist(node) == False:
            node.setG(gTemp)
            # H值計算
            node.manhattan(self.endNode)
            self.openList.append(node)
            node.father = self.currentNode
        # 如果在openList中，判斷currentNode到當前點的G是否更小
        # 如果更小，就重新計算g值，並且改變father
        else:
            nodeTmp = self.getNodeFromOpenList(node)
            if self.currentNode.g + gTemp < nodeTmp.g:
                nodeTmp.g = self.currentNode.g + gTemp
                nodeTmp.father = self.currentNode
        return

    def searchNear(self):
        """ 
        搜尋節點周圍的點 
        按照八個方位搜尋
        拐角處無法直接到達
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
        開始尋路 
        '''
        # 將初始節點加入開放列表
        self.startNode.manhattan(self.endNode)
        self.startNode.setG(0)
        self.openList.append(self.startNode)

        while True:
            # 獲取當前開放列表裡F值最小的節點
            # 並把它新增到封閉列表，從開發列表刪除它
            self.currentNode = self.getMinFNode()
            self.closeList.append(self.currentNode)
            self.openList.remove(self.currentNode)

            self.searchNear()

            # 檢驗是否結束
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
