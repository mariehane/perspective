import cv2

WINDOW_NAME = "Camera"

def show_camera():
    cap = cv2.VideoCapture(0) # Simply open the first camera
    if cap.isOpened():
        window_handle = cv2.namedWindow(WINDOW_NAME, cv2.WND_PROP_FULLSCREEN)#cv2.WINDOW_AUTOSIZE)
        cv2.setWindowProperty(WINDOW_NAME, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

        # Window
        while cv2.getWindowProperty(WINDOW_NAME, 0) >= 0:
            ret_val, img = cap.read()
            cv2.imshow(WINDOW_NAME, img)

            keyCode = cv2.waitKey(30) & 0xFF
            # Stop the program on the ESC key
            if keyCode == 27:
                break
        cap.release()
        cv2.destroyAllWindows()
    else:
        print("Unable to open camera")


if __name__ == "__main__":
    show_camera()
