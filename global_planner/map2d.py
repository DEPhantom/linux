from __future__ import print_function
import math
import copy


class map_data:  # struct
    def __init__(self):
        self.origin = ''
        self.change = '*'
        self.new_object = '*'  # laser_scan thing
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
        if ( self.data[point.x][point.y] != '#' ) :
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

    def clear_update_map(self):
        seld.data = copy.copy(self.origin_data)

    def clear_map(self):
        for i in range(len(self.data)):
            for j in range(len(self.data[i])):
                if self.data[i][j] == "o":
                    self.data[i][j] = "*"

    def data_set(self, data, degree ):
        for count in range(0, 720):
            i = count/2.0 -90+ degree
            i = -i + 360
            if ( i >= 360 ):
                i = i-360
            # end if
            elif ( i < 0 ) :
               i = i+360
            # end elif()

        #    print( len(data) ) # test
            if data[count].lenth != -1:
                data[count].x = round(data[count].lenth*100/5 * math.cos(i*math.pi/180))
                data[count].y = round(data[count].lenth*100/5 * math.sin(i*math.pi/180))


    # end data_set()

    def new_obstacle(self, new_map, new_obs, y, x):
        for i in range(0, 720):
            position_x = - int( new_obs[i].y ) - y + self.origin_x
            position_y = - int( new_obs[i].x ) - x + self.origin_y
            #position_y = -(new_obs[i].x + x) + self.origin_x
            #position_x = -(new_obs[i].y + y) + self.origin_y
            if ( new_obs[i].x != 0 or new_obs[i].y != 0 ) :
              if ( position_x < len( new_map ) and position_y < len( new_map[0] ) ) :
                new_map[position_x][position_y].new_object = '#'
              # end if
              else :
                print( "limit x is %d limit y is %d" %( len( new_map ), len( new_map[0] ) ) )
                print( "error x is %d error y is %d" %( position_x, position_y ) )
                print( " lidar degree: %d" %( i ) )
              # end else
            # print(position_x, end=" ")
            # print(position_y)

    def obs_increase(self, new_map):

        cols = self.w
        rows = self.h
        # print(new_map[65][10].new_object)
        for j in range(0, cols):
            for i in range(0, rows):
                new_map[i][j].change = new_map[i][j].origin

        # end for
        for j in range(0, cols):  # increase
            for i in range(0, rows):
                if new_map[i][j].origin == '*' and new_map[i][j].new_object == '#':
                    if new_map[i][j].new_object == '#':
                        if i-3 >= 0 and j-3 >= 0:   # left top
                            new_map[i-1][j-1].change = '#'
                            new_map[i-2][j-1].change = '#'
                            new_map[i-3][j-1].change = '#'
                            new_map[i-1][j-2].change = '#'
                            new_map[i-2][j-2].change = '#'
                            new_map[i-3][j-2].change = '#'
                            new_map[i-1][j-3].change = '#'
                            new_map[i-2][j-3].change = '#'
                            new_map[i-3][j-3].change = '#'
                        if i >= 0 and j-3 >= 0:    # top
                            new_map[i][j-1].change = '#'
                            new_map[i][j-2].change = '#'
                            new_map[i][j-3].change = '#'
                        if i+3 < rows and j-3 >= 0:  #right top
                            new_map[i+1][j-1].change = '#'
                            new_map[i+2][j-1].change = '#'
                            new_map[i+3][j-1].change = '#'
                            new_map[i+1][j-2].change = '#'
                            new_map[i+2][j-2].change = '#'
                            new_map[i+3][j-2].change = '#'
                            new_map[i+1][j-3].change = '#'
                            new_map[i+2][j-3].change = '#'
                            new_map[i+3][j-3].change = '#'
                        if i-3 >= 0 and j >= 0:  #left
                            new_map[i-1][j].change = '#'
                            new_map[i-2][j].change = '#'
                            new_map[i-3][j].change = '#'
                        if i >= 0 and j >= 0:
                            new_map[i][j].change = '#'
                        if i+3 < rows and j >= 0:   #right
                            new_map[i+1][j].change = '#'
                            new_map[i+2][j].change = '#'
                            new_map[i+3][j].change = '#'
                        if i-3 >= 0 and j+3 < cols:  #bot left
                            new_map[i-1][j+1].change = '#'
                            new_map[i-2][j+1].change = '#'
                            new_map[i-3][j+1].change = '#'
                            new_map[i-1][j+2].change = '#'
                            new_map[i-2][j+2].change = '#'
                            new_map[i-3][j+2].change = '#'
                            new_map[i-1][j+3].change = '#'
                            new_map[i-2][j+3].change = '#'
                            new_map[i-3][j+3].change = '#'
                        if i >= 0 and j+3 < cols:  #bot
                            new_map[i][j+1].change = '#'
                            new_map[i][j+2].change = '#'
                            new_map[i][j+3].change = '#'
                        if i+1 < rows and j+1 < cols:  # right bot
                            new_map[i+1][j+1].change = '#'
                            new_map[i+2][j+1].change = '#'
                            new_map[i+3][j+1].change = '#'
                            new_map[i+1][j+2].change = '#'
                            new_map[i+2][j+2].change = '#'
                            new_map[i+3][j+2].change = '#'
                            new_map[i+1][j+3].change = '#'
                            new_map[i+2][j+3].change = '#'
                            new_map[i+3][j+3].change = '#'
                    elif new_map[i][j].origin == '@':
                        new_map[i][j].change = new_map[i][j].origin

          #  for j in range(0, cols):
           #     for i in range(0, rows):
            #        new_map[i][j].change = new_map[i][j].new_object

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

    # read data
        for i in range(0, 359):
            b = copy.copy(temp)
            list1.append(b)

        return list1

    # end update_test()

    def update_map( self, lidar, current_x, current_y, current_degree ):
        lidar2 = []
        for i in range( len(lidar) ) :
          temp = ang_data()
          temp.lenth = lidar[i]
          lidar2.append( temp )
        # end for

        self.data_set( lidar2, current_degree )
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

        self.new_obstacle(yu_data, lidar2, current_x, current_y )  # 20 100
        self.showObj(yu_data)
        for i in range(1):
            self.obs_increase(yu_data)
        # end for

        for i in range(len(yu_data)):
            for j in range(len(yu_data[i])):
                self.data[i][j] = yu_data[i][j].change
            # end for

        # end for

    # end update_map()




