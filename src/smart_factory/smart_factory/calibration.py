import cv2
import numpy as np
import os
import glob
import matplotlib.pyplot as plt



import numpy as np
import cv2 as cv
import glob

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((9*6,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.


# For mean value calculation
mtx_list = []
dist_list = []


images = glob.glob('/home/yms/rokey_week8_ws/src/smart_factory/smart_factory/cali_data3/*')
print(images)

for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (9,6), None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)

        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        cv.drawChessboardCorners(img, (9,6), corners2, ret)
        #cv.imshow('img', img)
        #cv.waitKey(200)

        ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

        print(f'mtx',mtx)
        print(f'dist',dist)
        
        # Store results for averaging
        mtx_list.append(mtx)
        dist_list.append(dist)

cv.destroyAllWindows()

# Calculate and print the mean values
avg_mtx = np.mean(mtx_list, axis=0) if mtx_list else None
avg_dist = np.mean(dist_list, axis=0) if dist_list else None

print(f'avg_mtx',avg_mtx)
print(f'avg_dist',avg_dist)


# CHECKERBOARD = (9,6)
# criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# objpoints = []
# imgpoints = []

# mtxs = []
# dists= []

# objp = np.zeros((1,CHECKERBOARD[0] * CHECKERBOARD[1],3), np.float32)
# objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1,2)

# images = glob.glob('/home/yms/rokey_week8_ws/src/smart_factory/smart_factory/cali_data3/*')
# #images = glob.glob('/home/yms/rokey_week8_ws/src/smart_factory/smart_factory/FisheyeDataSet/test_images/Fisheye1_*.jpg')
# print(images)

# fig = plt.figure(figsize=(10,10))

# for fname in images:
#     img = cv2.imread(fname)

#     gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#     ret, corners = cv2.findChessboardCorners(gray,
#                                              CHECKERBOARD,
#                                              None)#cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
    
#     if ret == True:
#         objpoints.append(objp)

#         corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
        
#         imgpoints.append(corners2)

#         img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)

#     #cv2.imshow('img',img)
#     #cv2.waitKey(0)
    
#     #cv2.destroyAllWindows()

#     ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)

#     mtxs.append(mtx)
#     dists.append(dist)


# mtx= np.mean(mtxs,axis=0)
# dist = np.mean(dists,axis=0)

np.save('/home/yms/rokey_week8_ws/src/smart_factory/smart_factory/mtx720_12301.npy', avg_mtx)
np.save('/home/yms/rokey_week8_ws/src/smart_factory/smart_factory/dist720_12301.npy', avg_dist)

#print(mtx_avg)
#print(dist_avg)

#mtx = np.load('/home/yms/rokey_week8_ws/src/smart_factory/smart_factory/mtx720.npy')
#dist = np.load('/home/yms/rokey_week8_ws/src/smart_factory/smart_factory/dist720.npy')



# for fname in images:
#     img = cv2.imread(fname)

#     gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#     img1 = cv2.cvtColor(gray, cv2.COLOR_BGR2RGB)
#     #cv2.imshow('img',gray)
#     #cv2.waitKey(0)

#     ax1 = plt.subplot(1,2,1)
#     ax1.imshow(img1)

#     h, w = gray.shape[:2]
#     print(gray.shape)

#     newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h),1,(w, h))

#     dst = cv2.undistort(gray, mtx, dist, None, newcameramtx)

#     x,y,w,h = roi

#     dst = dst[y:y+h, x:x+w]

#     dst = cv2.cvtColor(dst, cv2.COLOR_BGR2RGB)

#     ax1 = plt.subplot(1,2,2)
#     ax1.imshow(dst)
#     plt.show()

# cv2.destroyAllWindows()





# from __future__ import print_function
# import cv2
# import numpy as np
# import sys



# desired_aruco_dictionary = "DICT_5X5_100"


# ARUCO_DICT = {
#   "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
#   "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
#   "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
#   "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
#   "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
#   "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
#   "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
#   "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
#   "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
#   "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
#   "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
#   "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
#   "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
#   "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
#   "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
#   "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
#   "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
# }


# def main():

#     if ARUCO_DICT.get(desired_aruco_dictionary, None) is None:
#         print("ERROR")
#         sys.exit(0)

#     this_aruco_dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[desired_aruco_dictionary])
#     this_aruco_parameters = cv2.aruco.DetectorParameters_create()

#     k = np.load('/home/yms/rokey_week8_ws/src/smart_factory/smart_factory/calibration_matrix_480p.npy')
#     d = np.load('/home/yms/rokey_week8_ws/src/smart_factory/smart_factory/distortion_coefficients_480p.npy')

#     cap = cv2.VideoCapture(3)

#     if not cap.isOpened():
#         print("Failed to open camera!")
#         return

#     while(True):

#         ret,frame = cap.read()

#         (corners, ids, rejected) = cv2.aruco.detectMarkers(
#             frame, this_aruco_dictionary, parameters=this_aruco_parameters
#         )

#         if len(corners) > 0:
#             ids = ids.flatten()

#             for (marker_corner, marker_id) in zip(corners, ids):

#                 corners = marker_corner.reshape((4,2))
#                 (top_left, top_right, bottom_right, bottom_left) = corners

#                 top_right = (int(top_right[0]), int(top_right[1]))
#                 bottom_right = (int(bottom_right[0]),int(bottom_right[1]))
#                 bottom_left = (int(bottom_left[0]),int(bottom_left[1]))
#                 top_left = (int(top_left[0]),int(top_left[1]))

#                 cv2.line(frame, top_left, top_right, (0,255,0),2)
#                 cv2.line(frame, top_right, bottom_right, (0,255,0),2)
#                 cv2.line(frame, bottom_right, bottom_left, (0,255,0),2)
#                 cv2.line(frame, bottom_left, top_left, (0,255,0),2)


#                 center_x = int((top_left[0] + bottom_right[0])/ 2.0)
#                 center_y = int((top_left[1] + bottom_right[1])/ 2.0)

#                 cv2.circle(frame, (center_x,center_y),4,(0,0,255), -1)

#                 cv2.putText(frame, str(marker_id),
#                             (top_left[0], top_left[1] -15),
#                             cv2.FONT_HERSHEY_SIMPLEX,
#                             0.5,(0,255,0),2)

#                 rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(marker_corner, 0.02,k,d)
#                 cv2.drawFrameAxes(frame, k,d, rvec, tvec, 0.01)
#                 print(rvec)
#                 ###################### tvec : 좌표 (월드기준)
#                 ###################### rvec : 방향 (월드기준)
#         cv2.imshow('frame', frame)

#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break
                
#     cap.release()
#     cv2.destroyAllWindows()

# if __name__ == '__main__':
#     print(__doc__)
#     main()