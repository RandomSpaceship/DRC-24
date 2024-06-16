import cv2 as cv
import numpy as np
import time

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


blueLow = (100, 200, 0)
blueHigh = (200, 255, 255)
yellowLow = (0, 200, 0)
yellowHigh = (80, 255, 190)

# BGR edge fills so Voronoi always has a boundary
blueFill = (160, 90, 6)
yellowFill = (30, 170, 170)

img = np.zeros((1024, 1024), np.uint8)

# Fill it with some white pixels
img[10, 10] = 255
img[20, 1000] = 255
img[:, 800:] = 255

morphKernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (5, 5))

cap = cv.VideoCapture(0)
if not cap.isOpened():
    print("Cannot open camera")
    exit()

while True:
    k = cv.waitKey(1)

    if k == ord("1"):
        currentlyShowing = -1
    if k == ord("2"):
        currentlyShowing = -2
    if k == ord("3"):
        currentlyShowing = -3
    if k == ord("4"):
        currentlyShowing = -4
    if k == ord("5"):
        currentlyShowing = -5
    if k == ord("6"):
        currentlyShowing = -6
    if k == ord("7"):
        currentlyShowing = -7
    if k == ord("8"):
        currentlyShowing = -8
    if k == ord("9"):
        currentlyShowing = -9
    if k == ord("0"):
        currentlyShowing = -10
    if k == ord("g"):
        currentlyShowing = 0
    if k == ord("H"):
        currentlyShowing = 1
    if k == ord("s"):
        currentlyShowing = 2
    if k == ord("v"):
        currentlyShowing = 3
    if k == ord("h"):
        currentlyShowing = 4

    start_time = time.time()
    ret = True
    inputImg = cv.imread("test2.png")

    # inputImg = cv.cvtColor(img, cv.COLOR_GRAY2BGR)
    # ret, inputImg = cap.read()
    # if ret == False:
    # break

    height, width, channels = inputImg.shape
    inputImg[:, 0:5] = blueFill
    inputImg[:, width - 5 : width] = yellowFill
    # inputImg = cv.flip(inputImg, 1)  # TODO REMOVE ON FINAL

    hsvImg = cv.cvtColor(inputImg, cv.COLOR_BGR2HSV_FULL)
    hsvHueOnly = hsvImg.copy()
    hsvHueOnly[:, :, 1] = 255
    hsvHueOnly[:, :, 2] = 255
    hsvHueOnly = cv.cvtColor(hsvHueOnly, cv.COLOR_HSV2BGR_FULL)
    hsvSatOnly = cv.cvtColor(hsvImg[:, :, 1], cv.COLOR_GRAY2BGR)
    hsvValOnly = cv.cvtColor(hsvImg[:, :, 2], cv.COLOR_GRAY2BGR)

    # pixel is in BGR!
    rgbPixel = inputImg[mouseY, mouseX]
    hsvPixel = hsvImg[mouseY, mouseX]

    blueThresh = cv.inRange(hsvImg, blueLow, blueHigh)
    blueThresh = cv.morphologyEx(blueThresh, cv.MORPH_OPEN, morphKernel)
    yellowThresh = cv.inRange(hsvImg, yellowLow, yellowHigh)
    yellowThresh = cv.morphologyEx(yellowThresh, cv.MORPH_OPEN, morphKernel)
    combinedThresh = cv.bitwise_xor(yellowThresh, blueThresh)
    end_time = time.time()

    blueDist = cv.normalize(
        cv.distanceTransform(cv.bitwise_not(blueThresh), cv.DIST_L2, 5),
        None,
        0,
        255,
        cv.NORM_MINMAX,
        cv.CV_8UC1,
    )
    yellowDist = cv.normalize(
        cv.distanceTransform(cv.bitwise_not(yellowThresh), cv.DIST_L2, 5),
        None,
        0,
        255,
        cv.NORM_MINMAX,
        cv.CV_8UC1,
    )
    combinedDist = cv.normalize(
        cv.distanceTransform(cv.bitwise_not(combinedThresh), cv.DIST_L2, 5),
        None,
        0,
        255,
        cv.NORM_MINMAX,
        cv.CV_8UC1,
    )

    diff = cv.absdiff(blueDist, yellowDist)
    ret, potPath = cv.threshold(diff, 1, 255, cv.THRESH_BINARY_INV)
    sobel = cv.normalize(
        cv.Sobel(combinedDist, cv.CV_64F, 2, 0, ksize=7),
        None,
        0,
        255,
        cv.NORM_MINMAX,
        cv.CV_8UC1,
    )
    ret, cpath = cv.threshold(sobel, 90, 255, cv.THRESH_BINARY_INV)
    # sobel = cv.GaussianBlur(sobel, (21, 21), 0)

    match currentlyShowing:
        case -1:
            txt = "BLU"
            showFrame = cv.cvtColor(blueThresh, cv.COLOR_GRAY2BGR)
        case -2:
            txt = "YLW"
            showFrame = cv.cvtColor(yellowThresh, cv.COLOR_GRAY2BGR)
        case -3:
            txt = "BDST"
            showFrame = cv.cvtColor(blueDist, cv.COLOR_GRAY2BGR)
        case -4:
            txt = "YDST"
            showFrame = cv.cvtColor(yellowDist, cv.COLOR_GRAY2BGR)
        case -5:
            txt = "ADIF"
            showFrame = cv.cvtColor(diff, cv.COLOR_GRAY2BGR)
        case -6:
            txt = "THR"
            showFrame = cv.cvtColor(potPath, cv.COLOR_GRAY2BGR)
        case -7:
            txt = "CMB"
            showFrame = cv.cvtColor(combinedThresh, cv.COLOR_GRAY2BGR)
        case -8:
            txt = "CDST"
            showFrame = cv.cvtColor(combinedDist, cv.COLOR_GRAY2BGR)
        case -9:
            txt = "HSBL"
            showFrame = cv.cvtColor(sobel, cv.COLOR_GRAY2BGR)
        case -10:
            txt = "TST"
            showFrame = cv.cvtColor(cpath, cv.COLOR_GRAY2BGR)
        case 0:
            txt = "RGB"
            showFrame = inputImg
        case 1:
            txt = "Hue"
            showFrame = hsvHueOnly
        case 2:
            txt = "Sat"
            showFrame = hsvSatOnly
        case 3:
            txt = "Val"
            showFrame = hsvValOnly
        case 4:
            txt = "HSV"
            showFrame = hsvImg
        case _:
            txt = "Dflt"
            showFrame = inputImg

    samplePixel = showFrame[mouseY, mouseX]
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
    rtext(
        showFrame,
        f"R{samplePixel[2]:03d} G{samplePixel[1]:03d} B{samplePixel[0]:03d}",
        (5, 210),
    )
    dt = end_time - start_time
    time_ms = dt * 1000
    fps = 1 / dt
    rtext(showFrame, f"T:{time_ms:03.0f}ms FPS:{fps:03.0f}", (5, 150))
    rtext(showFrame, f"W:{width} H:{height}", (5, 180))
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
