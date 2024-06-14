import cv2 as cv

window_title = "Frame"

currentlyShowing = 0

mouseX = 0
mouseY = 0


def mouse_event(event, x, y, flags, param):
    global mouseX, mouseY
    if event == cv.EVENT_MOUSEMOVE:
        mouseX, mouseY = x, y


cv.namedWindow(window_title)
cv.setMouseCallback(window_title, mouse_event)


def rtext(img, text, org):
    cv.putText(
        img,
        text,
        org,
        cv.FONT_HERSHEY_SIMPLEX,
        1,
        (255, 255, 255),
        4,
        cv.LINE_AA,
    )
    cv.putText(
        img,
        text,
        org,
        cv.FONT_HERSHEY_SIMPLEX,
        1,
        (0, 0, 0),
        2,
        cv.LINE_AA,
    )


cap = cv.VideoCapture(0)
if not cap.isOpened():
    print("Cannot open camera")
    exit()

while True:
    k = cv.waitKey(1)

    if k == ord("g"):
        currentlyShowing = 0
    if k == ord("h"):
        currentlyShowing = 1
    if k == ord("s"):
        currentlyShowing = 2
    if k == ord("v"):
        currentlyShowing = 3
    if k == ord("H"):
        currentlyShowing = 4

    # imgRaw = cv.imread("test.png")
    ret, rawImg = cap.read()
    if ret == False:
        break

    hsvImg = cv.cvtColor(rawImg, cv.COLOR_BGR2HSV_FULL)
    hsvHueOnly = hsvImg.copy()
    hsvHueOnly[:, :, 1] = 255
    hsvHueOnly[:, :, 2] = 255
    hsvHueOnly = cv.cvtColor(hsvHueOnly, cv.COLOR_HSV2BGR_FULL)
    hsvSatOnly = cv.cvtColor(hsvImg[:, :, 1], cv.COLOR_GRAY2BGR)
    hsvValOnly = cv.cvtColor(hsvImg[:, :, 2], cv.COLOR_GRAY2BGR)

    # pixel is in BGR!
    rgbPixel = rawImg[mouseY, mouseX]
    hsvPixel = hsvImg[mouseY, mouseX]

    showFrame = rawImg
    txt = "RGB"
    if currentlyShowing == 1:
        showFrame = hsvHueOnly
        txt = "Hue"
    elif currentlyShowing == 2:
        showFrame = hsvSatOnly
        txt = "Sat"
    elif currentlyShowing == 3:
        showFrame = hsvValOnly
        txt = "Val"
    elif currentlyShowing == 4:
        showFrame = hsvImg
        txt = "HSV"
    showFrame = cv.flip(showFrame, 1)
    rtext(showFrame, txt, (5, 30))
    rtext(
        showFrame,
        f"X:{mouseX:04d}, Y:{mouseY:04d}",
        (5, 60),
    )
    rtext(
        showFrame,
        f"R{rgbPixel[2]:03d} G{rgbPixel[1]:03d} B{rgbPixel[0]:03d}",
        (5, 90),
    )
    rtext(
        showFrame,
        f"H{hsvPixel[0]:03d} S{hsvPixel[1]:03d} V{hsvPixel[2]:03d}",
        (5, 120),
    )
    cv.imshow(window_title, showFrame)

    if k == ord(" ") or k == ord("q"):
        break

cap.release()
cv.destroyAllWindows()

# import numpy as np
# import cv as cv


# while True:
#     # Capture frame-by-frame
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
