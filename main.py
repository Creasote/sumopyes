import numpy as np
import cv2 as cv

# // Open camera
# // Get image feed
cap = cv.VideoCapture(0)
if not cap.isOpened():
    print("Cannot open camera")
    exit()

zeros = np.zeros([480,640], dtype="uint8")
while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    # if frame is read correctly ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    # Our operations on the frame come here
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    (B,G,R) = cv.split(frame)
    #print(G.size())
    # Display the resulting frame
    cv.imshow('frame', gray)
    cv.imshow("Red", cv.merge([zeros,zeros,R]))
    cv.imshow("Green", cv.merge([zeros,G,zeros]))
    cv.imshow("Blue", cv.merge([B,zeros,zeros]))
    if cv.waitKey(1) == ord('q'):
        break
# When everything done, release the capture
cap.release()
cv.destroyAllWindows()






# // Convert to mat

# // Create three copies (r, g, b)
# // Combine (g,b), look for orientation by centre of mass

# // Find target (r)

# // Output instructions to robot - how?