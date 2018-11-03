import cv2

# Create a VideoCapture object
cap = cv2.VideoCapture(1)

# Check if camera opened successfully
if (cap.isOpened() == False):
    print("Unable to read camera feed")


while (True):
    ret, frame = cap.read()

    if ret == True:

        ret=int(cap.set(3,800))
        ret=int(cap.set(4,448))

        # Display the resulting frame
        cv2.imshow('frame', frame)

        # Press Q on keyboard to stop recording
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Break the loop
    else:
        break

        # When everything done, release the video capture and video write objects
cap.release()

# Closes all the frames
cv2.destroyAllWindows()
