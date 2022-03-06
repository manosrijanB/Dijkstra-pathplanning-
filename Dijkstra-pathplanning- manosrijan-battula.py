import numpy as np
import cv2 as cv


# Vistualizing the workspace with left top of image as corner

map = np.zeros((250,400,3), np.uint8)

hex_img = np.array([[240, 129.79], [240, 170.20], [200, 195.41],
                [160, 170.20], [160, 129.79], [200, 104.58]], np.int32)


poly_img = np.array([[31,65], [115,35], [85, 70], [105,155]], np.int32)
x_grid = np.arange(400)
y_grid = np.arange(250)
# creating map with the obstacles 
cv.circle(map, (300, 65), 45, (255, 255, 255), thickness = -1)
cv.fillPoly(map, [hex_img],(255,255,255))
cv.fillPoly(map, [poly_img], (255,255,255))


#  new co ordinates for from origin as left down corrner
hex = np.array([[240, 80], [240, 120], [200, 145],
                [160, 120], [160, 80], [200, 55], [240, 80]], np.int32)
hex_center = np.array([200, 100])

#  spliting the polygon along the center line to create two triangles
triangle_1 = np.array([[31,185], [105, 95], [85,180], [31,185]])

triangle_1_center = np.mean(triangle_1[:-1], axis = 0)

triangle_2 = np.array([[31, 185], [115,215], [85, 180], [31,185]])

triangle_2_center = np.mean(triangle_2[:-1], axis = 0)

# 

def ccw(A,B,C):
    return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])

# Return true if line segments AB and CD intersect
def intersect(A,B,C,D):
    return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)


def obstacle(pt):
    # for circle 
    x, y = pt
    if np.sqrt((x - 300)**2 + (y - 185)**2) <= 45 :
        return True
    #  for hexagon
    ret = False
    for i in range(len(hex) - 1):
        ret = ret or intersect(pt, hex_center, hex[i], hex[i+1])
    if not ret: 
        return True
#  for triangle 1 for polygon
    ret = False
    for i in range(len(triangle_1) - 1):
        ret = ret or intersect(pt, triangle_1_center, triangle_1[i], triangle_1[i+1])
    if not ret: 
        return True
  # for trianlge 2 of polygon
    ret = False
    for i in range(len(triangle_2) - 1):
        ret = ret or intersect(pt, triangle_2_center, triangle_2[i], triangle_2[i+1])
    if not ret: 
        return True    
    return False

# defining actions for movement 

def left (pos):
    i, j = pos
    cost = 1
    if i > 0 and not (obstacle(pos) ):
       
        pos = (i - 1 , j)
        return pos, cost

    else:
        return None

def right (pos):
      i, j = pos
      cost = 1
      if i < 399 and not (obstacle(pos) ):
      
          pos= (i + 1 , j)
          return pos, cost

      else:
          return None


def up (pos):
      i, j = pos
      cost = 1
      if j > 0 and not (obstacle(pos) ):
      
          pos= (i  , j - 1)
          return pos, cost

      else:
          return None

def down (pos):
      i, j = pos
      cost = 1
      if j < 249 and not (obstacle(pos) ):
        #   state = state.copy()
          
          pos = (i, j + 1)
          return pos, cost

      else:
          return None


def up_left (pos):
      i, j = pos
      cost = 1.4
      if i > 0 and j > 0 and not (obstacle(pos) ):
        
          pos = (i - 1, j - 1)
          return pos, cost

      else:
          return None


def up_right (pos):
      i, j = pos
      cost = 1.4
      if i < 399 and j > 0 and not (obstacle(pos) ):
                    
       
          pos = (i + 1, j - 1)
          return pos, cost

      else:
          return None

def down_left (pos):
      i, j = pos
      cost = 1.4
      if i > 0 and j < 249 and not (obstacle(pos) ):
       
          pos= (i - 1, j + 1)
          return pos, cost

      else:
          return None

def down_right (pos):
      i, j = pos
      cost = 1.4
      if i < 399 and j < 249 and not (obstacle(pos) ):
       
          pos = (i + 1, j + 1)
          return pos, cost

      else:
          return None

all_actions = [up, down, left, right, down_left, down_right, up_left, up_right]

# containing in a dictionary
def get_node( pos, parent, cost):
    Node  = {'pos': pos, 
             'parent': parent, 
             'cost': cost}
    return Node


def init_nodes(start_pos):

#   x_grid = np.arange(400)
#   y_grid = np.arange(250)
  open_dict = {}

  for x in x_grid:
    for y in y_grid:    
        open_dict[(x, y)] = get_node((x,y), None, np.inf)
        
  open_dict[start_pos]['cost'] = 0
  return open_dict


def Dijkstra (start_pos = (10, 10), goal_pos = (390, 240)):

    All_nodes = init_nodes(start_pos)
    open_dict = {start_pos: 0}
    closed_lis = {start_pos}
    explored = [start_pos]

    while len(open_dict):
        min_pos = min(open_dict, key = open_dict.get)
        closed_lis.add(min_pos)
        open_dict.pop(min_pos)
        min_node = All_nodes[min_pos]
        # print(min_pos)
        for action in all_actions:
            act_output = action(min_pos)
            if act_output is not None:
                pos, cost = act_output
                if pos not in closed_lis:
                    child_node = All_nodes[pos]
                    new_cost = min_node['cost'] + cost
                    if new_cost < child_node['cost']:
                        child_node['cost'] = new_cost
                        child_node['parent'] = min_pos
                        open_dict[pos] = new_cost
                        explored.append(child_node['pos'])
                    if pos == goal_pos: 
                        print("solution found")
                        return backtrack(All_nodes, pos), explored

#BackTracking

def backtrack(All_nodes, pos):
    print("Tracking Back")
    p = []
    while All_nodes[pos]['parent'] is not None:
        p.append(pos)
        pos = All_nodes[pos]['parent']
    p.reverse()
    return p


path, explored = Dijkstra()

#  visulizing the path explored nodes
def visualize(path, explored):
    ''' Visualise the exploration and the recently found path
    '''
    img = map
    h, w, _ = img.shape
    out = cv.VideoWriter('outpy.mp4',cv.VideoWriter_fourcc(*'mp4v'), 60.0, (w, h))
    
    for i in range(len(explored)):
        pos = (249 - explored[i][1], explored[i][0])
        img[pos] = [0, 255, 0]
        if i%100 == 0:
            out.write(img)
            cv.imshow('hi', img)
            cv.waitKey(1)
    for pos in path:
        pos = (249 - pos[1], pos[0])
        img[pos] = [0, 0, 255]
    for i in range(50): 
        out.write(img)
    out.release()
    cv.imshow('hi', img)
    cv.waitKey(0)

visualize(path, explored)



cv.imshow('map', map)
cv.waitKey(0)
