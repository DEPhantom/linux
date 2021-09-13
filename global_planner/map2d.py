from __future__ import print_function
import copy


class map2d:
    """
    地圖資料
    """

    def __init__(self):
        self.data = []
        self.w = -1
        self.h = -1
        self.origin_x = -1
        self.origin_y = -1
        self.passTag = '*'
        self.pathTag = 'o'
        self.pathway = []

    def getMap(self, filename ):
        test = []
        f = open(filename)
        line = f.readline()
        print(line)
        line2 = f.readline()
        line2 = line2.replace('\n', '')
        temp = line2.split(' ')
        self.w = int(temp[0])
        self.h = int(temp[1])
        print(line2)
        line3 = f.readline()
        print(line3)
        for line in f.readlines():
            lie = line[:len(line)-2]
            s = lie.split(' ')
            # print(len(s))
            self.data.append(s)

    def showMap(self):
        for x in range(0, self.h):
            for y in range(0, self.w):
                print(self.data[x][y], end='')
            print(" ")
        return

    def setMap(self, point):
        self.data[point.x][point.y] = self.pathTag
        return

    def isPass(self, point):
        if (point.x < 0 or point.x > self.h - 1) or (point.y < 0 or point.y > self.w - 1):
            return False

        if self.data[point.x][point.y] == self.passTag:
            return True

    def get_path(self, path):
        for i in range(len(path)):
            temp = copy.deepcopy(
                path[len(path) - i - 1].point)
            self.pathway.append(temp)
        for i in range(len(self.pathway)):
            x = self.pathway[i].x-self.origin_x
            y = self.pathway[i].y-self.origin_y
            self.pathway[i].x = -x
            self.pathway[i].y = -y
        return

    def get_origin(self):
        for i in range(len(self.data)):
            for j in range(len(self.data[i])):
                if self.data[i][j] == "@":
                    self.origin_x = i
                    self.origin_y = j
        return
