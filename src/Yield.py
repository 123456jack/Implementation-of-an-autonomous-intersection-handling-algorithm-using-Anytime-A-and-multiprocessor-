# -*- coding: utf-8 -*-
"""
Created on Sat Apr 28 14:08:42 2018

@author: bhaga
"""
#CHANGE VARIABLE yieldl to to activate the yield function on green light

#BEHAVIORAL AND MULTITHREAD MODEL IS DEFINED IN THE MAIN FUNCTION
import matplotlib.pyplot as plt
import math
from copy import deepcopy
import threading
import queue



class Node:

    def __init__(self, x, y, cost, pind):
        self.x = x
        self.y = y
        self.cost = cost
        self.pind = pind

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.pind)


def calc_fianl_path(ngoal, closedset):
    # generate final course
    rx, ry = [ngoal.x ], [ngoal.y ]
    pind = ngoal.pind
    while pind != -1:
        n = closedset[pind]
        rx.append(n.x )
        ry.append(n.y )
        pind = n.pind

    return rx, ry


def a_star_planning(q,sx, sy, gx, gy, ox, oy, rr,d):
    """
    gx: goal x position [m]
    gx: goal x position [m]
    ox: x position list of Obstacles [m]
    oy: y position list of Obstacles [m]
    rr: robot radius[m]
    """

    nstart = Node(round(sx ), round(sy), 0.0, -1)
    ngoal = Node(round(gx), round(gy), 0.0, -1)
    ox = [iox  for iox in ox]
    oy = [ioy for ioy in oy]

    obmap, minx, miny, maxx, maxy, xw, yw = calc_obstacle_map(ox, oy, rr)

    motion = get_motion_model()

    openset, closedset = dict(), dict()
    openset[calc_index(nstart, xw, minx, miny)] = nstart

    while 1:
        c_id = min(
            openset, key=lambda o: openset[o].cost + d*calc_h(ngoal, openset[o].x, openset[o].y))
        current = openset[c_id]
      
        if current.x == ngoal.x and current.y == ngoal.y:
            print("Graph Search")
            ngoal.pind = current.pind
            ngoal.cost = current.cost
            break

        # Remove the item from the open set
        del openset[c_id]
        # Add it to the closed set
        closedset[c_id] = current

        # Expand search grid based on motion model
        for i in range(len(motion)):
            node = Node(current.x + motion[i][0], current.y + motion[i][1],
                        current.cost + motion[i][2], c_id)
            n_id = calc_index(node, xw, minx, miny)

            if not verify_node(node, obmap, minx, miny, maxx, maxy):
                continue

            if n_id in closedset:
                continue
            # Otherwise if it is already in the open set
            if n_id in openset:
                if openset[n_id].cost > node.cost:
                    openset[n_id].cost = node.cost
                    openset[n_id].pind = c_id
            else:
                openset[n_id] = node

    rx, ry = calc_fianl_path(ngoal, closedset)
    q.put((rx,ry))
    return 


def calc_h(ngoal, x, y):
    w = 10.0  # weight of heuristic
    d = w * math.sqrt((ngoal.x - x)**2 + (ngoal.y - y)**2)
    return d


def verify_node(node, obmap, minx, miny, maxx, maxy):

    if node.x < minx:
        return False
    elif node.y < miny:
        return False
    elif node.x >= maxx:
        return False
    elif node.y >= maxy:
        return False

    if obmap[node.x][node.y]:
        return False

    return True


def calc_obstacle_map(ox, oy, vr):

    minx = round(min(ox))
    miny = round(min(oy))
    maxx = round(max(ox))
    maxy = round(max(oy))
  

    xwidth = 60 # Limitation of the grid
    ywidth = 60 #Limitation of the grid

    # obstacle map generation
    obmap = [[False for i in range(xwidth)] for i in range(ywidth)]
    for ix in range(xwidth):
        x = ix + minx
        for iy in range(ywidth):
            y = iy + miny
            #  print(x, y)
            for iox, ioy in zip(ox, oy):
                d = math.sqrt((iox - x)**2 + (ioy - y)**2)
                if d <= vr :
                    obmap[ix][iy] = True
                    break

    return obmap, minx, miny, maxx, maxy, xwidth, ywidth


def calc_index(node, xwidth, xmin, ymin):
    return (node.y - ymin) * xwidth + (node.x - xmin)


def get_motion_model():
    # dx, dy, cost
    motion = [[1, 0, 0.1],
              [0, 1, 0.1],
              [-1, 0, 10],
              [0, -1, 0.1]]    #removed addn states


    return motion

def obstacle_detect(colx, rx, ry, dobx, doby, doby_middle, ox, oy, steps_taken, speed_obs):    #added

    halfcar_len = 2
    safe_marg = 1

    if colx in rx:
        ind = rx.index(colx)
        coly = ry[ind]
        steps_car = len(rx) - ind - (steps_taken + 1)
        steps_dobj = (doby_middle - coly)/speed_obs  # no of steps

        if abs(steps_car - steps_dobj)< (halfcar_len + safe_marg)/speed_obs:

            doby_pred = [x - speed_obs*steps_dobj for x in doby]
            ox.extend(dobx)
            oy.extend(doby_pred)

    return ox, oy

def main():
    print(__file__ + " start!!")

    # start and goal position
    sx = 33.0  # [m]
    sy = 2.0  # [m]
 
    gx = 0.0  # [m]
    gy = 33.0  # [m]
   
   
    robot_size = 1.0  # [m]
    
    dobx = []
    doby=[]
    for i in range(5):             #Coordinates for the obstacle
        dobx.append(26.0)
        doby.append(50.0 +i)
        
    for i in range(5):
        dobx.append(26.0 +i)
        doby.append(54.0)           #53
        
    for i in range(5):
        dobx.append(30.0)           #31.0
        doby.append(50.0 +i)
        
    for i in range(5):
        dobx.append(26.0 +i)
        doby.append(50.0)

    doby_min = min(doby)   
    dobx_max = max(dobx)   
    doby_middle = 52.0      
    speed_obs = .5
    ox, oy = [], []
    sobx = 7
    soby = 33


    for i in range(25):   #Coordinates of teh intersection
        ox.append(25.0)
        oy.append(i)
    for i in range(25):
        ox.append(35.0)
        oy.append(60 - i)
    for i in range(25):
        ox.append(35.0)
        oy.append(i)
    for i in range(25):
        ox.append(25.0)
        oy.append(60 - i)
    for i in range(26):
        ox.append(i)
        oy.append(25)
    for i in range(35,60):
        ox.append(i)
        oy.append(25)
    for i in range(26):
        ox.append(i)
        oy.append(35)
    for i in range(35,60):
        ox.append(i)
        oy.append(35)
    for i in range(25):
        ox.append(31)
        oy.append(i)
    obstaticx=deepcopy(ox)
    q = queue.Queue()
    t = threading.Thread(target=a_star_planning,args=(q,sx, sy, gx, gy, ox, oy, robot_size,1))#First Call to Astar
    t.start()
    (rx,ry)=q.get()
    i = 0; 
    hmult = 3
    j=0
    f=0
    do = 0
    stopx = 36
    stopy = 27
    yieldl = 1
    yieldf = 0
    stop = 0
    stopc=0
    stopf = 0
    yieldj=0
    while [sx,sy]!=[gx,gy] or stop ==1 : #Main while loop for the motion model 
        if stop ==1 and math.sqrt((sx-stopx)**2+(sy-stopy)**2)<15 : #For the stop sign
            gy = 25
            gx = 33
            if stopf ==0:
               stopf=1
            else:
                stopf=2
            if sy==gy:
                stopc =stopc+1
        elif yieldl ==1 and math.sqrt((sx-stopx)**2+(sy-stopy)**2)<15and yieldf==0 :
            yieldf = 1
            
        if stopc==40 :
            if stopf==2:
                stopf=1
            else:
                stopf=3    
            stop=0
            gy=33
            gx=0.0
            
        if len(ox)!=len(obstaticx) or stopf==1:#Detection of the object
            print(len(ox),len(obstaticx))
            print(do)
            if do == 1:
                hmult = 3
            else:
                hmult = 2
            if stop==1 or stopf==1:
                hmult=1
                
            while math.sqrt((sx-sobx)**2+(sy-soby)**2)>5  : #Planning is done only till the inner horizon is hit
                if do==1 and yieldf==1:
                    obstaticx = deepcopy(ox)
                    break
                t1 = threading.Thread(target =  a_star_planning, args = (q,sx, sy, gx, gy, ox, oy, robot_size,hmult))#Threading for the simulating multiple controller     
                t1.start()
                j=i
                while q.empty():#While the astar is calculating, the vehicle continues to move
                     if len(rx)<(i+2):
                        sx=sx
                        sy=sy
                     else:
                         sx= rx[-2-i]
                         sy =ry[-2-i] 
                     plt.cla()
                     plt.plot(ox, oy, ".k")
                     plt.plot(sx, sy, "xr")
                     plt.plot(gx, gy, "xb")
                     plt.plot(dobx,doby,".k")
                     plt.grid(True)
                     plt.axis("equal")
                     plt.plot(sobx,soby,"xg")
                     
                     plt.plot(rx, ry, "-r")
                     plt.show()
                     fig = plt.gcf()
                     ax = fig.gca()
                     circle1 = plt.Circle((sx, sy), 15, color='r',fill=False)  
                     ax.add_artist(circle1)
                     circle2 = plt.Circle((sx, sy), 5, color='g',fill=False)  
                     ax.add_artist(circle2)
                     circle3 = plt.Circle((sx, sy), 30, color='b',fill=False)  
                     ax.add_artist(circle3)
                     if stop ==1:
                         circle3 = plt.Circle((stopx, stopy), 1, color='r',fill=True)  
                         ax.add_artist(circle3)
                     elif yieldl==1:
                         circle4 = plt.Circle((stopx, stopy), 1, color='g',fill=True)  
                         ax.add_artist(circle4)
                     
                     else:
                         arrow = plt.arrow(stopx, stopy, -2, 0, color='g',width=0.5)
                         ax.add_artist(arrow)
                     plt.pause(0.001)
                     i=i+1
                     if min(doby)>0:#for the motion of the dynamic obstacle
                         doby = [x-speed_obs for x in doby]   
                         doby_min = doby_min - speed_obs   
                         doby_middle = doby_middle -speed_obs 
                     
                     
                (rx,ry) = q.get()
                i = i-j
                if hmult>1:
                    hmult = hmult-1
                else:
                    break
            obstaticx = deepcopy(ox)   
            
            
        if len(rx)<(i+2):
            sx=sx
            sy=sy
        elif yieldf==1 and sx==33 and sy ==25:
            yieldj=yieldj+1
            sx=sx
            sy=sy
        else:      
            sx= rx[-2-i]
            sy =ry[-2-i]
        if doby_min<sy and yieldf==1:
            yieldf=2
            do=0
            i=i-yieldj
        if math.sqrt((sx - dobx_max) ** 2 + (sy - doby_min) ** 2) < 30 and dobx_max - sx < 5 and stop==0:  #Detection of the dynamic obstacle
                do=1
                (ox, oy) = obstacle_detect(32.0, rx, ry, dobx, doby, doby_middle, ox, oy, i, speed_obs)
                (ox, oy) = obstacle_detect(30.0, rx, ry, dobx, doby, doby_middle, ox, oy, i, speed_obs)
                (ox, oy) = obstacle_detect(28.0, rx, ry, dobx, doby, doby_middle, ox, oy, i, speed_obs)
             
        plt.ion()
        
        plt.cla()
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "xr")
        plt.plot(gx, gy, "xb")
        plt.plot(dobx,doby,".k")
        plt.grid(True)
        plt.axis("equal")
        plt.plot(sobx,soby,"xg")
        
        plt.plot(rx, ry, "-r")
        plt.show()
        fig = plt.gcf()
        ax = fig.gca()
        circle1 = plt.Circle((sx, sy), 15, color='r',fill=False)  
        ax.add_artist(circle1)
        circle2 = plt.Circle((sx, sy), 5, color='g',fill=False)  
        ax.add_artist(circle2)
        circle3 = plt.Circle((sx, sy), 30, color='b',fill=False)  
        ax.add_artist(circle3)
        if stop ==1:
            circle3 = plt.Circle((stopx, stopy), 1, color='r',fill=True)  
            ax.add_artist(circle3)
        elif yieldl==1:
            circle4 = plt.Circle((stopx, stopy), 1, color='g',fill=True)  
            ax.add_artist(circle4)    
        else:
            arrow = plt.arrow(stopx, stopy, -2, 0, color='g',width=0.5)
            ax.add_artist(arrow)
        
        plt.pause(0.15)

        if math.sqrt((sx-sobx)**2+(sy-soby)**2)<15 and f==0:
           ox.append(sobx)
           oy.append(soby)
           do=0
           f=1


        if min(doby)>0:
            doby = [x-speed_obs for x in doby]   #added and changed
            doby_min = doby_min - speed_obs   #added
            doby_middle = doby_middle -speed_obs  #added

        i = i+1
        plt.ioff()
if __name__ == '__main__':
    main()