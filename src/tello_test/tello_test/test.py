from imutils.video import VideoStream
import argparse
import imutils
import time
import cv2
import sys
import numpy as np
import tello_drone as bello
import math
from djitellopy import Tello
tello=Tello()

host = ''
port = 9000
local_address = (host, port)

drone = bello.Tello(host, port, is_dummy=False)

frame_read = drone.get_frame_read()

markerlist=[]

cnt40=0

global found_fid

found_fid=0



#############################

########################################################################

def search_all_four_fids(fid):
    # print("found fid, searching all four fids")

    if fid!=[1,2,3,4]:
        
        if fid==[]:

            drone.rotate_cw(30)
            print('drone.rotate_cw(30)')
            time.sleep(0.05)

        if fid==[1]:
            print('found fid 1')
            drone.rotate_cw(5)

        if fid==[2]:
            drone.rotate_ccw(5)

        if fid==[3]:
            drone.move_up(40)
            print('drone.move_up(20)')

        if fid==[4]:
            drone.move_down(20)
            print('drone.move_down(20)')
            # 4132

        ################### too near?
        if fid==[1,2]:
            drone.move_backward(20)
            print('drone.move_backward(20)')
        if fid==[3,4]:
            drone.move_backward(20)
            print('drone.move_backward(20)')
        ################### left down
        if fid==[1,3]:
            drone.rotate_cw(5)
            print('drone.rotate_cw(10)')
            # drone.move_backward(20)
            # print('drone.move_backward(20)')
            drone.move_up(40)
            print('drone.move_up(20)')
        ################### left up
        if fid==[1,4]:
            drone.rotate_cw(5)
            print('drone.rotate_cw(10)')
            drone.move_down(20)
            print('drone.move_down(20)')
        ################### right down
        if fid==[2,3]:
            drone.rotate_ccw(10)
            print('23 drone.rotate_ccw(10)')
            # drone.move_backward(20)
            # print('drone.move_backward(20)')
            drone.move_up(40)
            print('drone.move_up(20)')

        if fid==[2,4]:
            drone.rotate_ccw(10)
            print('24 drone.rotate_ccw(10)')
            # drone.move_backward(20)
            # print('drone.move_backward(20)')
            drone.move_down(20)
            print('drone.move_down(20)')
        ###################################### found 3
        if fid==[1,2,3]:
            # drone.move_backward(20)
            # print('drone.move_backward(20)')
            drone.move_up(40)
            print('drone.move_up(20)')
        if fid==[1,2,4]:
            drone.move_down(20)
            print('drone.move_down(20)')
            # drone.move_backward(20)
            # print('drone.move_backward(20)')
        if fid==[1,3,4]:
            # drone.move_backward(20)
            # print('drone.move_backward(20)')
            drone.rotate_cw(5)
            print('drone.rotate_cw(5)')        
        if fid==[2,3,4]:
            # drone.move_backward(20)
            # print('drone.move_backward(20)')
            drone.rotate_ccw(5)
            print('234 drone.rotate_ccw(10)')  
 

    else:
 
        return True
       


########################################## to avoid other num appearing besides 1234
def washmarkerlist(mark):

    markerlist_temp=[]

    for i in range(len(mark)):

        if(mark[i]==1 or mark[i]==2 or mark[i]==3 or mark[i]==4):
            markerlist_temp.append(mark[i])
    markerlist_temp=list(set(markerlist_temp))
    return markerlist_temp

#########################################
def correct_initial_angle(a,b,c,d):

    print('correcting initial angle')
    vertical_mid= (c+d)/2

    print ("portion is ",abs(a-vertical_mid)/abs(b-vertical_mid))
    cv2.putText(frame, str(abs(a-vertical_mid)/abs(b-vertical_mid)), (100,100), cv2.FONT_HERSHEY_SIMPLEX, 4, (0, 255, 0), 2)
    
    if abs(a-vertical_mid)/abs(b-vertical_mid)<0.8:
        print('too right!!! <0.8')
        drone.move_left(20)
        # drone.move_forward(20)
        drone.rotate_cw(40)
        
        
        
    if abs(a-vertical_mid)/abs(b-vertical_mid)>1.25:
        print('too left!!! >1.25')
        
        drone.move_right(20) 
        # drone.move_forward(20)
        drone.rotate_ccw(40)
        

#######################################

def determine_path_length(a,b):
    distance_to_go=abs(a-b)/20
    print("corresponding distance is",distance_to_go)
    if distance_to_go>5:
        distance_to_go=5
    print("corresponding distance is",distance_to_go)
    
    return (distance_to_go*100) 
################################################################## 
ball_color = 'green'

color_dist = {'red': {'Lower': np.array([0, 60, 60]), 'Upper': np.array([6, 255, 255])},
              'blue': {'Lower': np.array([100, 80, 46]), 'Upper': np.array([124, 255, 255])},
              #   'green': {'Lower': np.array([35, 43, 35]), 'Upper': np.array([90, 255, 255])},
              # caliberate width
            #   'green': {'Lower': np.array([25, 100, 35]), 'Upper': np.array([50, 120, 155])},
            #   'green': {'Lower': np.array([25, 100, 50]), 'Upper': np.array([50, 120, 255])},
            'green': {'Lower': np.array([25, 100, 35]), 'Upper': np.array([50, 255, 255])},##########ORIGINAL
                # 'green': {'Lower': np.array([25, 100, 55]), 'Upper': np.array([50, 255, 255])},
              }


########################################################### get_triangle_area and quadrilateral area
def get_triangle_area(point_a, point_b, point_c):
    """

    求三角形面积;
    一切都是基于三角形！！！！
    原文链接：https: // blog.csdn.net / hjq376247328 / article / details / 49465137

    :param point_a、point_b、point_c: 数据类型为list,二维坐标形式[x、y]或三维坐标形式[x、y、z]
    :return: 三角形面积，返回 - 1 为不能组成三角形;
    """
    area = -1
    side = [0, 0, 0]  # 储存三条边的长度;
    a_x, b_x, c_x = point_a[0], point_b[0], point_c[0]
    a_y, b_y, c_y = point_a[1], point_b[1], point_c[1]

    if len(point_a) == len(point_b) == len(point_c) == 3:
        # print("坐标点为3维坐标形式")
        a_z, b_z, c_z = point_a[2], point_b[2], point_c[2]
    else:
        a_z, b_z, c_z = 0, 0, 0
        # print("坐标点为2维坐标形式，z 坐标默认值设为0")

    side_ab = math.sqrt(pow(a_x - b_x, 2) +
                        pow(a_y - b_y, 2) + pow(a_z - b_z, 2))
    side_ac = math.sqrt(pow(a_x - c_x, 2) +
                        pow(a_y - c_y, 2) + pow(a_z - c_z, 2))
    side_cb = math.sqrt(pow(c_x - b_x, 2) +
                        pow(c_y - b_y, 2) + pow(c_z - b_z, 2))

    # 不能构成三角形;
    if side_ab + side_ac <= side_cb or side_ab + side_cb <= side_ac or side_ac + side_cb <= side_ab:
        return area

    # 利用海伦公式。s = sqr(p * (p - a)(p - b)(p - c));
    p = (side_ab + side_ac + side_cb) / 2  # 半周长
    area = math.sqrt(p * (p - side_ab) * (p - side_ac) * (p - side_cb))

    return area


def get_quadrilateral_area(point_a, point_b, point_c, point_d):
    """
    本算法可以计算凸四边形，也可以计算凹四边形
    本算法用于计算已经按顺时针或者逆时针排序过的点
    四边形可以分解为两个三角形
    :param point_a, point_b, point_c, point_d: 数据类型为list,二维坐标形式[x、y]或三维坐标形式[x、y、z]
    :return: 四边形面积
    """

    # 对于凸四边形，以任一点分割成两个三角形均可
    # 1，以ac线分割
    area_11 = get_triangle_area(point_a, point_b, point_c)
    area_12 = get_triangle_area(point_a, point_b, point_d)
    area_13 = get_triangle_area(point_a, point_c, point_d)
    area_14 = get_triangle_area(point_b, point_c, point_d)

    return (area_11+area_12+area_13+area_14)/2


def get_area(a):

    return get_quadrilateral_area(np.int0(a)[0], np.int0(a)[1], np.int0(a)[2], np.int0(a)[3])

#############################################
def look_into_cnt0(a):
    cnt1 = []
    l1 = []
    rect1 = []
    box1 = []

    cnt2 = []
    l2 = []
    rect2 = []
    box2 = []
    cnt3 = []
    l3 = []
    rect3 = []
    box3 = []
    l4 = []
    rect4 = []
    box4 = []
    cnt = 0

    if len(a) >= 4:
        l1 = max(a, key=cv2.contourArea)
        if len(l1) > 0:
            # print ('l1 is ',l1)
            rect1 = cv2.minAreaRect(l1)
            box1 = cv2.boxPoints(rect1)
            # print(box1)
            if get_area(box1) > 1000:
                cnt += 1
                cv2.drawContours(frame, [np.int0(box1)], 0, (0, 255, 255), 2)

            cnt1 = [x for x in a if x not in l1]
            if len(cnt1) > 0:
                l2 = max(cnt1, key=cv2.contourArea)
                # print ('l2 is ',l2)
                rect2 = cv2.minAreaRect(l2)
                box2 = cv2.boxPoints(rect2)
                # print(box2)
                if get_area(box2) > 1000:
                    cnt += 1
                    cv2.drawContours(frame, [np.int0(box2)], 0, (0, 255, 255), 2)

                cnt2 = [x for x in cnt1 if x not in l2]
                if len(cnt2) > 0:
                    l3 = max(cnt2, key=cv2.contourArea)
                    # print ('l3 is',l3)
                    rect3 = cv2.minAreaRect(l3)
                    box3 = cv2.boxPoints(rect3)
                    # print(box3)
                    if get_area(box3) > 1000:
                        cnt += 1
                        cv2.drawContours(
                            frame, [np.int0(box3)], 0, (0, 255, 255), 2)

                    cnt3 = [x for x in cnt2 if x not in l3]

                    if len(cnt3) > 0:
                        l4 = max(cnt3, key=cv2.contourArea)
                        # print ('l4 is',l4)
                        rect4 = cv2.minAreaRect(l4)
                        box4 = cv2.boxPoints(rect4)
                        # print(box4)
                        if get_area(box4) > 1000:
                            cnt += 1
                            cv2.drawContours(
                                frame, [np.int0(box4)], 0, (0, 255, 255), 2)
                            cv2.putText(
                                frame, 'found4', (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

                            mid = find_middle_of_all_boxes(
                                box1, box2, box3, box4)

                            return mid

                        else:
                            return [0]
                    else:
                        return [0]
                else:
                    return[0]
            else:
                return [0]
        else:
            return [0]
    else:
        return [0]


##########################################################################
def find_middle(a):
    # print('find middle',a)
    frame_mid_x = (a[0][0]+a[1][0]+a[2][0]+a[3][0])/4
    frame_mid_y = (a[0][1]+a[1][1]+a[2][1]+a[3][1])/4

    return int(frame_mid_x), int(frame_mid_y)



def find_middle_of_all_boxes(box1, box2, box3, box4):
    mid1 = find_middle(box1)
    cv2.circle(frame, mid1, 2, (0, 0, 255), 10)
    mid2 = find_middle(box2)
    cv2.circle(frame, mid2, 2, (0, 0, 255), 10)
    mid3 = find_middle(box3)
    cv2.circle(frame, mid3, 2, (0, 0, 255), 10)
    mid4 = find_middle(box4)
    cv2.circle(frame, mid4, 2, (0, 0, 255), 10)

    mid = find_middle([mid1, mid2, mid3, mid4])
    cv2.circle(frame, mid, 2, (0, 0, 255), 10)

    return mid

#############################################

def frame_get_to_position(box):
    (x, y), radius = cv2.minEnclosingCircle(box)
    center = (int(x), int(y))
    radius = int(radius)
    cv2.circle(frame, center, radius, (0, 0, 255), 4)
    cv2.circle(frame, center, 4,(0, 0, 255), 3)
    
    if center[1]-radius<0 and center[1]+radius>720:
        print('too far')
        drone.move_backward(25)
        time.sleep(0.02)

    if center[0]-radius<0 and center[0]+radius>960:
        print('too far')
        drone.move_backward(25)
        time.sleep(0.02)

    if center[1]-radius<0:
        print('move up 20')
        drone.move_up(20)

        time.sleep(0.02)

    if center[1]+radius>720:
        print('move down 20')
        drone.move_down(20)
        time.sleep(0.02)



    if center[0]-radius<0:
        print('move left 20')
        drone.move_left(20)
        time.sleep(0.02)
        
    if center[0]+radius>960:
        print('move right 20')
        drone.move_right(20)
        time.sleep(0.02)    
    
    if 0<=center[1]-radius and center[1]+radius<720 and center[0]-radius>0 and center[0]+radius<960:
        cv2.putText(frame, 'found whole circle', (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
        if corner_aim_mid(center) == True:
            print ("gogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogo")

            tello.send_rc_control(0,50,-30,0)
            time.sleep(3.5)
            tello.send_rc_control(0,0,0,0)
            time.sleep(1)
##################################################################
def start_frame(img,cnt0):
    l1 = max(cnt0, key=cv2.contourArea)
    rect1 = cv2.minAreaRect(l1)
    box1 = cv2.boxPoints(rect1)
    cv2.drawContours(img, [np.int0(box1)], 0, (0, 255, 255), 2)
    
    if get_area(box1) > 10000:
        print('frame system on')
        frame_get_to_position(box1)
###########################################


def corner_aim_mid(a):

    if 470 <= a[0] <= 510:

        if 340 <= a[1] <= 380:

            return True

        elif a[1] > 380:
            drone.move_down(20)
            time.sleep(0.01)
            print('drone.move down')
            return False
        elif a[1] < 340:
            drone.move_up(20)
            time.sleep(0.01)
            print('drone.move up')
            return False

    elif a[0] < 470:
        
        drone.move_left(20)
        time.sleep(0.01)
        print('drone.move_left(20)')
        return False

    elif a[0] > 510:
        drone.move_right(20)
        
        time.sleep(0.01)
        print('drone.move_right(20)')
        return False

########################################################################
def number_of_valid_corner(cnt):
    count=0

    l1 = max(cnt, key=cv2.contourArea)
    if len(l1) > 0:
        rect1 = cv2.minAreaRect(l1)
        box1 = cv2.boxPoints(rect1)
        if get_area(box1) > 1000:
            count += 1


        cnt1 = [x for x in cnt if x not in l1]
        if len(cnt1) > 0:
            l2 = max(cnt1, key=cv2.contourArea)
            rect2 = cv2.minAreaRect(l2)
            box2 = cv2.boxPoints(rect2)
            # print(box2)
            if get_area(box2) > 1000:
                count += 1

            cnt2 = [x for x in cnt1 if x not in l2]

            if len(cnt2) > 0:
                l3 = max(cnt2, key=cv2.contourArea)
                rect3 = cv2.minAreaRect(l3)
                box3 = cv2.boxPoints(rect3)
                if get_area(box3) > 1000:
                    count += 1

                cnt3 = [x for x in cnt2 if x not in l3]

                if len(cnt3) > 0:
                    l4 = max(cnt3, key=cv2.contourArea)
                    rect4 = cv2.minAreaRect(l4)
                    box4 = cv2.boxPoints(rect4)

                    if get_area(box4) > 1000:
                        count += 1
    return count
###########################################
def start_corner(img,cnt0):
    if number_of_valid_corner(cnt0) > 0:

        box_cood = look_into_cnt0(cnt0)  # come up with 4 coordinates

    if box_cood == [0]:

        drone.move_up(20)
        time.sleep(0.05)

    elif corner_aim_mid(box_cood) == True:
        print("just forward")
        print ("gogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogo")

        tello.send_rc_control(0,50,-40,0)
        time.sleep(3.5)
        tello.send_rc_control(0,0,0,0)
        time.sleep(1)    
 
######################################
######################################
######################################
######################################

ap = argparse.ArgumentParser()
ap.add_argument("-t", "--type", type=str, default="DICT_4X4_100", help="type of ArUCo tag to detect")
args = vars(ap.parse_args())

ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000
}

desired_aruco_dictionary = "DICT_4X4_100"

print("[INFO] loading image...")
image = cv2.imread("/home/chopin/Desktop/RAS/marker.png")
image = imutils.resize(image, width=600)

#if ARUCO_DICT.get(desired_aruco_dictionary, None) is None:
#	print("[INFO] ArUCo tag of '{}' is not supported".format(desired_aruco_dictionary))
#	sys.exit(0)

if ARUCO_DICT.get(args["type"], None) is None:
	print("[INFO] ArUCo tag of '{}' is not supported".format(args["type"]))
	sys.exit(0)

#print("[INFO] detecting '{}' tags...".format(desired_aruco_dictionary))
#arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[desired_aruco_dictionary])
#arucoParams = cv2.aruco.DetectorParameters_create()

print("[INFO] detecting '{}' tags...".format(args["type"]))
arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[args["type"]])
arucoParams = cv2.aruco.DetectorParameters_create()

# (corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)

print("[INFO] starting video stream...")
vs = VideoStream(src=0).start()

one_x=[]
one_y=[]


two_x=[]
two_y=[]


three_x=[]
three_y=[]

four_x=[]
four_y=[]


main_cnt=0 




######################################################################################
tello.send_rc_control(0,0,50,0)
time.sleep(2)
######################################################################################
aimtopLeft=(475,340)
aimtopRight=(505,340)
aimbottomRight=(505,380)
aimbottomLeft=(475,380)
######################################################################################

while True:
    
    frame = frame_read.frame

    cap = drone.get_video_capture()

    (corners, ids, rejected) = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)
    
    gs_frame = cv2.GaussianBlur(frame, (5, 5), 0)

    hsv = cv2.cvtColor(gs_frame, cv2.COLOR_BGR2HSV)

    inRange_hsv = cv2.inRange(
        hsv, color_dist[ball_color]['Lower'], color_dist[ball_color]['Upper'])

    cnt0 = cv2.findContours(
        inRange_hsv.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

    cv2.line(frame, aimtopLeft, aimtopRight, (0, 255, 0), 3)
    cv2.line(frame, aimtopRight, aimbottomRight, (0, 255, 0), 3)
    cv2.line(frame, aimbottomRight, aimbottomLeft, (0, 255, 0), 3)
    cv2.line(frame, aimbottomLeft, aimtopLeft, (0, 255, 0), 3)
                    
    if len(corners) > 0:
      
        # flatten the ArUco IDs list
        ids = ids.flatten()
        # loop over the detected ArUCo corners
        for (markerCorner, markerID) in zip(corners, ids):
            # extract the marker corners (which are always returned in
            # top-left, top-right, bottom-right, and bottom-left order)
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            # convert each of the (x, y)-coordinate pairs to integers

            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))

            
            # draw the bounding box of the ArUCo detection
            cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
            cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)
            # compute and draw the center (x, y)-coordinates of the ArUco
            # marker
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)
            # draw the ArUco marker ID on the image
            cv2.putText(frame, str(markerID), (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            # print("[INFO] ArUco marker ID: {}".format(markerID))
            markerlist.append(markerID)





            if found_fid==1:

                # print("found it!!","main cnt 次数为",main_cnt)

                if main_cnt<40:
                    if markerID==1:
                        one_x.append(cX)
                        one_y.append(cY)

                    if markerID==2:
                        two_x.append(cX)
                        two_y.append(cY)

                    if markerID==3:
                        three_x.append(cX)
                        three_y.append(cY)

                    if markerID==4:
                        four_x.append(cX)
                        four_y.append(cY)

                    main_cnt+=1

                if main_cnt==40:

                    # print("main cnt 运行40次")

                    main_cnt=0


                    one_mid_x=(np.mean(one_x)+np.mean(two_x)+np.mean(three_x)+np.mean(four_x))/4
                    one_mid_y=(np.mean(one_y)+np.mean(two_y)+np.mean(three_y)+np.mean(four_y))/4
                     

                    if one_mid_x>0:

                        if one_mid_y>0:

                            cv2.circle(frame, (int(one_mid_x),int(one_mid_y)), 4, (0, 0, 255), 3)

                    

                    ###### if middle of fiducials dqqqeviate from mid of screen:
                    print("if middle of fiducials deviate from mid of screen")
                    # print("midx midy is ",one_mid_x,one_mid_y)

                    correct_initial_angle(np.mean(one_x),np.mean(two_x),np.mean(three_x),np.mean(four_x))



                    ################## draw aiming area
                    
                    if 465<=one_mid_x<=515:
                        
                        if 340<=one_mid_y<=380:
                            
                          
                            
                            print ("gogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogogo")

                            # determine path length
                            # dist=determine_path_length(np.mean(one_x),np.mean(two_x))
                            tello.send_rc_control(0,50,-35,0)
                            time.sleep(7)
                            tello.send_rc_control(0,0,0,0)
                            time.sleep(1)
                            # print("move forward",dist)

                          

                        elif 380 < one_mid_y:

                            drone.move_down(20)
                            
                            print("drone.move_down(16)")  

                        elif one_mid_y < 340:

                            drone.move_up(20) 

                            print("drone.move_up(16) " )


                    elif 510<one_mid_x:


                        drone.move_right(20)
                        time.sleep(0.05)
                        # print("midx!!!!!!!  tello.send_rc_control(20,0,0,0)")
                        print('drone.move_right(20)')
                    elif one_mid_x<470:

                        # tello.send_rc_control(-20,0,0,0)
                        # time.sleep(0.05)
                        # tello.send_rc_control(0,0,0,0)
                        # time.sleep(0.05)
                        drone.move_left(20)
                        time.sleep(0.05)
                        print('drone.move_left(20)')
                        # print("midx!!!!!!!  tello.send_rc_control(-20,0,0,0)")

                        
                       

                    one_x=[]
                    one_y=[]
                    two_x=[]
                    two_y=[]
                    three_x=[]
                    three_y=[]
                    four_x=[]
                    four_y=[]


    if len(corners) == 0:
        time.sleep(0.01)
        if len(corners) == 0:
            if(len(cnt0))!= 0:  # found something 
                l1 = max(cnt0, key=cv2.contourArea)
                rect1 = cv2.minAreaRect(l1)
                box1 = cv2.boxPoints(rect1)

                novc=number_of_valid_corner(cnt0)

                cv2.putText(frame,  f'[{novc}]' , (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
            
                
                if novc==1:
                    if get_area(box1) > 10000:

                        start_frame(frame,cnt0)################################## Enter frame Mode

                if novc>1:
                        start_corner(frame,cnt0)

                    
                    
                        

    if cnt40==40:# calculate within 40 loops
        
        # print('cnt 运行40次')
        markerlist=washmarkerlist(markerlist)

        print(' final markerlist is ',markerlist)
        
        ######### when final marker is not 1,2,3,4


        if search_all_four_fids(markerlist)==True:

            found_fid=1

        else :
           
            found_fid=0

        
    

        markerlist=[]

        cnt40=0


    else:
        cnt40+=1





    # show live frame
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1)
    if key == ord("q"):
        print('landing')
        tello.land()
        break
        
    
        
 
cv2.destroyAllWindows()
vs.stop()

