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
blueHigh = (180, 255, 255)
yellowLow = (0, 200, 0)
yellowHigh = (80, 255, 190)
magentaLow = (200, 200, 0)
magentaHigh = (255, 255, 255)

# BGR color fills
blueFill = (160, 90, 6)
yellowFill = (30, 170, 170)

img = np.zeros((1024, 1024), np.uint8)

# Fill it with some white pixels
img[10, 10] = 255
img[20, 1000] = 255
img[:, 800:] = 255

colourKernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))

pathOpenKernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (5, 5))
pathCloseKernel = cv.getStructuringElement(cv.MORPH_RECT, (9, 9))
pathDilateKernel = cv.getStructuringElement(cv.MORPH_RECT, (3, 11))

derivativeKernelSize = 21
pathThresholdVal = 90
horizCutoffDist = 100

cap = cv.VideoCapture(0)
if not cap.isOpened():
    print("Cannot open camera")
    exit()

picIn = cv.imread("test4.png")  # TODO TESTING ONLY
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

    # IMAGE INPUT
    # ret = True
    # inputImg = picIn.copy()
    # inputImg = cv.cvtColor(img, cv.COLOR_GRAY2BGR)
    ret, inputImg = cap.read()
    if ret == False:
        break

    # TODO never changes
    rows, cols, channels = inputImg.shape
    # fill the edges so voronoi will always have a boundary
    # inputImg[:, 0:5] = blueFill
    # inputImg[:, cols - 5 : cols] = yellowFill
    # inputImg = cv.flip(inputImg, 1)  # TODO ONLY FOR LAPTOP TEST

    # calculate thresholding for obstacles
    hsvImg = cv.cvtColor(inputImg, cv.COLOR_BGR2HSV_FULL)
    blueThresh = cv.inRange(hsvImg, blueLow, blueHigh)
    yellowThresh = cv.inRange(hsvImg, yellowLow, yellowHigh)
    magentaThresh = cv.inRange(hsvImg, magentaLow, magentaHigh)
    combinedThresh = cv.bitwise_xor(yellowThresh, blueThresh)
    combinedThresh = cv.bitwise_xor(combinedThresh, magentaThresh)
    combinedThresh = cv.morphologyEx(combinedThresh, cv.MORPH_OPEN, colourKernel)

    combinedDist = cv.distanceTransform(cv.bitwise_not(combinedThresh), cv.DIST_L2, 5)

    # TODO This never changes, can be computed at startup
    bottomRowsMask = np.zeros_like(combinedDist, np.uint8)
    bottomRowsMask[rows - 5 : rows, :] = 1

    rawSobel = cv.Sobel(combinedDist, cv.CV_32F, 2, 0, ksize=derivativeKernelSize)
    # rawSobel = np.uint8(rawSobel)

    sobel = cv.normalize(rawSobel, None, 0, 255, cv.NORM_MINMAX, cv.CV_8UC1)
    derivativeMin, derivativeMax, derivativeMinLoc, derivativeMaxLoc = cv.minMaxLoc(
        sobel, bottomRowsMask
    )
    ret, rawPaths = cv.threshold(
        sobel, derivativeMin + pathThresholdVal, 255, cv.THRESH_BINARY_INV
    )
    denoisedPaths = cv.morphologyEx(rawPaths, cv.MORPH_OPEN, pathOpenKernel)
    finalPaths = cv.dilate(denoisedPaths, pathDilateKernel)

    contours, hierarchy = cv.findContours(
        finalPaths, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE
    )

    end_time = time.time()

    # RENDERING
    match currentlyShowing:
        case -1:
            txt = "BLU"
            showFrame = cv.cvtColor(blueThresh, cv.COLOR_GRAY2BGR)
        case -2:
            txt = "YLW"
            showFrame = cv.cvtColor(yellowThresh, cv.COLOR_GRAY2BGR)
        case -3:
            txt = "MAG"
            showFrame = cv.cvtColor(magentaThresh, cv.COLOR_GRAY2BGR)
        case -4:
            txt = "CMB"
            showFrame = cv.cvtColor(combinedThresh, cv.COLOR_GRAY2BGR)
        case -5:
            txt = "DIST"
            combinedDistFrame = cv.normalize(
                combinedDist,
                None,
                0,
                255,
                cv.NORM_MINMAX,
                cv.CV_8UC1,
            )
            showFrame = cv.cvtColor(combinedDistFrame, cv.COLOR_GRAY2BGR)
        case -6:
            txt = "HSBL"
            showFrame = cv.cvtColor(sobel, cv.COLOR_GRAY2BGR)
        case -7:
            txt = "RPTH"
            showFrame = cv.cvtColor(rawPaths, cv.COLOR_GRAY2BGR)
        case -8:
            txt = "DNPT"
            showFrame = cv.cvtColor(denoisedPaths, cv.COLOR_GRAY2BGR)
        case -9:
            txt = "FPTH"
            showFrame = cv.cvtColor(finalPaths, cv.COLOR_GRAY2BGR)
        case -10:
            txt = "CNTR"
            showFrame = cv.cvtColor(finalPaths, cv.COLOR_GRAY2BGR)
            cv.drawContours(showFrame, contours, -1, (0, 255, 0), 2)
        case 0:
            txt = "RGB"
            showFrame = inputImg
        case 1:
            txt = "Hue"
            hsvHueOnly = hsvImg.copy()
            hsvHueOnly[:, :, 1] = 255
            hsvHueOnly[:, :, 2] = 255
            hsvHueOnly = cv.cvtColor(hsvHueOnly, cv.COLOR_HSV2BGR_FULL)
            showFrame = hsvHueOnly
        case 2:
            txt = "Sat"
            showFrame = cv.cvtColor(hsvImg[:, :, 1], cv.COLOR_GRAY2BGR)
        case 3:
            txt = "Val"
            showFrame = cv.cvtColor(hsvImg[:, :, 2], cv.COLOR_GRAY2BGR)
        case 4:
            txt = "HSV"
            showFrame = hsvImg
        case _:
            txt = "Dflt"
            showFrame = inputImg

    # pixel is in BGR!
    mouseX = min(max(mouseX, 0), cols)
    mouseY = min(max(mouseY, 0), rows)
    rgbPixel = inputImg[mouseY, mouseX]
    hsvPixel = hsvImg[mouseY, mouseX]
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
    rtext(showFrame, f"W:{cols} H:{rows}", (5, 180))
    cv.imshow(window_title, showFrame)

    if k == ord(" ") or k == ord("q"):
        break

cap.release()
cv.destroyAllWindows()
