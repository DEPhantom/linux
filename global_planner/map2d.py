from __future__ import print_function
import math
import copy


class map_data:  # struct
    def __init__(self):
        self.origin = ''
        self.change = '*'
        self.new_object = '*'  # laser_scan的東西
    # end __init__

# end class


class ang_data:
    def __init__(self):
        self.lenth = -1.0
        self.x = 0
        self.y = 0
    # end def

# end class


class map2d:
    """
    map data
    """

    def __init__(self):
        self.origin_data = []
        self.data = []
        self.w = -1
        self.h = -1
        self.origin_x = -1
        self.origin_y = -1
        self.passTag = '*'
        self.pathTag = 'o'
        self.pathway = []

    def getMap(self, filename):
        test = []
        f = open(filename)
        line = f.readline()
        # print(line)
        line2 = f.readline()
        line2 = line2.replace('\n', '')
        temp = line2.split(' ')
        self.w = int(temp[0])
        self.h = int(temp[1])
        # print(line2)
        line3 = f.readline()
        # print(line3)
        for line in f.readlines():
            lie = line[:len(line)-2]
            s = lie.split(' ')
            # print(len(s))
            self.data.append(s)
        self.origin_data = self.data
        self.origin_data = copy.copy(self.data)

    def showMap(self):
        for x in range(0, self.h):
            for y in range(0, self.w):
                print(self.data[x][y], end='')
            print(" ")

    def setMap(self, point):
        self.data[point.x][point.y] = self.pathTag

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

    def get_origin(self):
        for i in range(len(self.data)):
            for j in range(len(self.data[i])):
                if self.data[i][j] == "@":
                    self.origin_x = i
                    self.origin_y = j
                    self.data[i][j] = "*"

    def clear_path(self):
        del self.pathway[:]

    def clear_map(self):
        for i in range(len(self.data)):
            for j in range(len(self.data[i])):
                if self.data[i][j] == "o":
                    self.data[i][j] = "*"

    def data_set(self, data):
        for i in range(0, 359):
            if data[i].lenth != -1:
                data[i].x = round(data[i].lenth * math.cos(i*math.pi/180))
                data[i].y = round(data[i].lenth * math.sin(i*math.pi/180))

    def new_obstacle(self, new_map, new_obs, y, x):
        for i in range(0, 359):
            position_x = -new_obs[i].x - y + self.origin_x
            position_y = -new_obs[i].y - x + self.origin_y
            #position_y = -(new_obs[i].x + x) + self.origin_x
            #position_x = -(new_obs[i].y + y) + self.origin_y
            new_map[position_x][position_y].new_object = '#'
            # print(position_x, end=" ")
            # print(position_y)

    def obs_increase(self, new_map):

        cols = 122
        rows = 129
        # print(new_map[65][10].new_object)
        for j in range(0, cols):
            for i in range(0, rows):
                new_map[i][j].change = new_map[i][j].origin

        # end for
        for j in range(0, cols):  # 增厚
            for i in range(0, rows):
                if new_map[i][j].origin == '*' and new_map[i][j].new_object == '#':
                    if new_map[i][j].new_object == '#':
                        if i-1 >= 0 and j-1 >= 0:
                            new_map[i-1][j-1].change = '#'
                        if i >= 0 and j-1 >= 0:
                            new_map[i][j-1].change = '#'
                        if i+1 < rows and j-1 >= 0:
                            new_map[i+1][j-1].change = '#'
                        if i-1 >= 0 and j >= 0:
                            new_map[i-1][j].change = '#'
                        if i >= 0 and j >= 0:
                            new_map[i][j].change = '#'
                        if i+1 < rows and j >= 0:
                            new_map[i+1][j].change = '#'
                        if i-1 >= 0 and j+1 < cols:
                            new_map[i-1][j+1].change = '#'
                        if i >= 0 and j+1 < cols:
                            new_map[i][j+1].change = '#'
                        if i+1 < rows and j+1 < cols:
                            new_map[i+1][j+1].change = '#'
                    elif new_map[i][j].origin == '@':
                        new_map[i][j].change = new_map[i][j].origin

     #       for j in range(0, cols):
      #          for i in range(0, rows):
       #             new_map[i][j].change = new_map[i][j].new_object

    # end obs_increase

    def showObj(self, yu_data):
        for i in range(0, len(yu_data)):
            for j in range(0, len(yu_data[i])):
                print(yu_data[i][j].new_object, end='')
            # end for

            print(" ")

        # end for

    # end showObj

    def update_test(self):
        list1 = []
        temp = ang_data()
        temp.lenth = 5

    # 讀入data
        for i in range(0, 359):
            b = copy.copy(temp)
            list1.append(b)

        return list1

    # end update_test()

    def update_map(self):
        fake_lidar = self.update_test()
        self.data_set(fake_lidar)
        yu_data = []
        temp_list = []
        for i in range(len(self.data)):
            for j in range(len(self.data[i])):
                temp = map_data()
                temp.origin = self.data[i][j]
                temp_list.append(temp)

            # end for

            yu_data.append(temp_list)
            temp_list = []

        # end for

        self.new_obstacle(yu_data, fake_lidar, 20, -20)  # 20 100
        self.showObj(yu_data)
        for i in range(3):
            self.obs_increase(yu_data)
        # end for

        for i in range(len(yu_data)):
            for j in range(len(yu_data[i])):
                self.data[i][j] = yu_data[i][j].change
            # end for
        # end for
