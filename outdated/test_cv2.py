import cv2
print('cv2 version', cv2.__version__)
cap=cv2.VideoCapture(1)
print('cap opened', cap.isOpened())
cap.set(cv2.CAP_PROP_FRAME_WIDTH,320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,240)
print('width',cap.get(cv2.CAP_PROP_FRAME_WIDTH),'height',cap.get(cv2.CAP_PROP_FRAME_HEIGHT),'fps',cap.get(cv2.CAP_PROP_FPS))
cap.release()
print('done')
