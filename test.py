import cv2

window_title = "Frame"

showingHsv = False

mouseX = 0
mouseY = 0


def mouse_event(event, x, y, flags, param):
    global mouseX, mouseY
    if event == cv2.EVENT_MOUSEMOVE:
        mouseX, mouseY = x, y


cv2.namedWindow(window_title)
cv2.setMouseCallback(window_title, mouse_event)


def rtext(img, text, org):
    cv2.putText(
        img,
        text,
        org,
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (255, 255, 255),
        4,
        cv2.LINE_AA,
    )
    cv2.putText(
        img,
        text,
        org,
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (0, 0, 0),
        2,
        cv2.LINE_AA,
    )


while True:
    k = cv2.waitKey(1)

    if k == ord("h"):
        showingHsv = True
    if k == ord("g"):
        showingHsv = False

    imgRaw = cv2.imread("test.png")

    img = imgRaw if showingHsv else imgRaw

    rtext(img, "HSV" if showingHsv else "RGB", (5, 30))
    rtext(
        img,
        f"X:{mouseX:04d}, Y:{mouseY:04d}",
        (5, 60),
    )
    # pixel is in BGR!
    pixel = img[mouseY, mouseX]
    rtext(
        img,
        f"{pixel[2]:03d},{pixel[1]:03d},{pixel[0]:03d}",
        (5, 90),
    )

    cv2.imshow(window_title, img)

    if k == ord(" ") or k == ord("q"):
        break


# import numpy as np
# import cv2 as cv

# # cap = cv.VideoCapture(0)
# # if not cap.isOpened():
# #     print("Cannot open camera")
# # exit()

# while True:
#     # Capture frame-by-frame
#     # ret, frame = cap.read()
#     ret = True
#     frame = cv.imread("test.png")

#     # if frame is read correctly ret is True
#     if not ret:
#         print("Can't receive frame (stream end?). Exiting ...")
#         break
#     # Our operations on the frame come here
#     gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
#     # Display the resulting frame
#     cv.imshow("frame", gray)
#     if cv.waitKey(1) == ord("q"):
#         break

# # When everything done, release the capture
# # cap.release()
# cv.destroyAllWindows()
