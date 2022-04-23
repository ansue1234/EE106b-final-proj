import cv2
import numpy as np
import matplotlib.pyplot as plt
from grip import GripPipeline

# define a video capture object
vid = cv2.VideoCapture(0)
processor = GripPipeline()

size_to_dist = lambda a: 0.01714102+15.74211494/a
# x = np.array([175000, 100000, 46550, 10500, 4500, 2300, 1400])
# y = np.array([0.07, 0.1, 0.15, 0.3, 0.45, 0.6, 0.75])
# x = 1/np.sqrt(x/np.pi)
# x = np.vstack((np.ones(len(x)), x))
# print(x.T)

# print(np.linalg.lstsq(x.T, y))
# plt.plot(x, y)
# plt.show()
while(True):
      
    # Capture the video frame
    # by frame
    ret, frame = vid.read()
  
    # Display the resulting frame
    
    processor.process(frame)
    origin = np.array([frame.shape[0], frame.shap[1]])

    keypoints = [max(processor.find_blobs_output, key =lambda a: a.size)] if len(processor.find_blobs_output) else []
    im_with_keypoints = cv2.drawKeypoints(frame, keypoints, np.array([]), (0,255,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    # the 'q' button is set as the
    for kp in keypoints:
        ball_xy = np.array([kp.pt[0], kp.pt[1]])
        diff = origin - ball_xy
        
        dist = size_to_dist(kp.size/2)
        print(dist)
    cv2.imshow('frame', im_with_keypoints)
    
    # quitting button you may use any
    # desired button of your choice
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
  
# After the loop release the cap object
vid.release()
# Destroy all the windows
cv2.destroyAllWindows()

# plt.plot(np.arange(len(area)), area)
# plt.show()

