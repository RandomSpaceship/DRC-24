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
pathDilateKernel = cv.getStructuringElement(cv.MORPH_RECT, (3, 11))

derivativeKernelSize = 21
pathThresholdVal = 90
pathMinArea = 15
horizCutoffDist = 250
bottomRowsPathCheck = 5

# cap = cv.VideoCapture(0)
# if not cap.isOpened():
#     print("Cannot open camera")
#     exit()

picIn = cv.imread("test2.png")  # TODO TESTING ONLY
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
    if k == ord("-"):
        currentlyShowing = -11
    if k == ord("="):
        currentlyShowing = -12
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
    inputImg = picIn.copy()
    # inputImg = cv.cvtColor(img, cv.COLOR_GRAY2BGR)
    # ret, inputImg = cap.read()
    # if ret == False:
    #     break

    # TODO never changes
    rows, cols, channels = inputImg.shape
    centre_x = cols / 2
    centre_y = rows / 2
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
    bottomRowsMask[rows - bottomRowsPathCheck : rows, :] = 1

    rawSobel = cv.Sobel(combinedDist, cv.CV_32F, 2, 0, ksize=derivativeKernelSize)
    # rawSobel = np.uint8(rawSobel)

    sobel = cv.normalize(rawSobel, None, 0, 255, cv.NORM_MINMAX, cv.CV_8U)
    derivativeMin, derivativeMax, derivativeMinLoc, derivativeMaxLoc = cv.minMaxLoc(
        sobel, bottomRowsMask
    )
    rawPaths = cv.inRange(sobel, 0, derivativeMin + pathThresholdVal)
    denoisedPaths = cv.morphologyEx(rawPaths, cv.MORPH_OPEN, pathOpenKernel)
    finalPaths = cv.dilate(denoisedPaths, pathDilateKernel)

    contours, hierarchy = cv.findContours(
        finalPaths, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE
    )

    contour_moments = [None] * len(contours)
    centroids = [None] * len(contours)
    bounding_boxes = [None] * len(contours)
    valid_path_contour_indices = []
    nearby_path_contour_indices = []

    for i in range(len(contours)):
        contour = contours[i]
        contour_moments[i] = cv.moments(contour, True)
        # add 1e-5 to avoid division by zero
        centroid_x = int(contour_moments[i]["m10"] / (contour_moments[i]["m00"] + 1e-5))
        centroid_y = int(contour_moments[i]["m01"] / (contour_moments[i]["m00"] + 1e-5))
        centroids[i] = (centroid_x, centroid_y)

        bounding_box = cv.boundingRect(contour)
        bounding_boxes[i] = bounding_box
        tr_x, tr_y, w, h = bounding_box
        bl_x = tr_x + w
        bl_y = tr_y + h
        if bl_y >= rows - bottomRowsPathCheck and cv.contourArea(contour) > pathMinArea:
            valid_path_contour_indices.append(i)
            if (
                abs(centre_x - bl_x) < horizCutoffDist
                or abs(centre_x - tr_x) < horizCutoffDist
                or abs(centre_x - centroid_x) < horizCutoffDist
            ):
                nearby_path_contour_indices.append(i)

    test = np.zeros_like(finalPaths)
    for i in nearby_path_contour_indices:
        cv.drawContours(test, contours, i, (255, 255, 255), cv.FILLED)
    # lines = cv.HoughLinesP(finalPaths, 1, np.pi / 180, 30, None, 50, 10)
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
            txt = "SBL"
            showFrame = cv.cvtColor(sobel, cv.COLOR_GRAY2BGR)
        case -7:
            txt = "PTHS"
            showFrame = cv.cvtColor(test, cv.COLOR_GRAY2BGR)
        case -8:
            txt = "DNPT"
            showFrame = cv.cvtColor(denoisedPaths, cv.COLOR_GRAY2BGR)
        case -9:
            txt = "FPTH"
            showFrame = cv.cvtColor(finalPaths, cv.COLOR_GRAY2BGR)
        case -10:
            txt = "CNTR"
            showFrame = cv.cvtColor(finalPaths, cv.COLOR_GRAY2BGR)
            cv.drawContours(showFrame, contours, -1, (0, 255, 0), 1, cv.LINE_AA)
            for centroid in centroids:
                cv.drawMarker(showFrame, centroid, (255, 0, 0), cv.MARKER_CROSS, 11, 3)
            for x, y, w, h in bounding_boxes:
                cv.rectangle(
                    showFrame, (x, y), (x + w, y + h), (0, 0, 255), 2, cv.LINE_AA
                )
        case -11:
            txt = "BCTR"
            showFrame = cv.cvtColor(finalPaths, cv.COLOR_GRAY2BGR)
            for i in valid_path_contour_indices:
                cv.drawContours(showFrame, contours, i, (255, 255, 0), -1, cv.LINE_AA)
            for i in nearby_path_contour_indices:
                cv.drawContours(
                    showFrame,
                    contours,
                    i,
                    (0, 0, 255),
                    2,
                    cv.LINE_AA,
                )
        # case -12:
        #     txt = "PATH"
        #     showFrame = np.zeros_like(inputImg)
        #     for tmp in lines:
        #         line = tmp[0]
        #         p1 = (line[0], line[1])
        #         p2 = (line[2], line[3])
        #         cv.line(showFrame, p1, p2, (0, 0, 255), 2, cv.LINE_AA)
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

    cv.line(showFrame, (int(cols / 2), 0), (int(cols / 2), rows), (255, 0, 255), 2)
    cv.line(
        showFrame,
        (int(cols / 2) - horizCutoffDist, rows),
        (int(cols / 2) + horizCutoffDist, rows),
        (255, 0, 255),
        2,
    )
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
        f"R{samplePixel[2]:03.0f} G{samplePixel[1]:03.0f} B{samplePixel[0]:03.0f}",
        (5, 210),
    )
    dt = end_time - start_time
    time_ms = dt * 1000
    fps = 1 / dt
    rtext(showFrame, f"T:{time_ms:03.0f}ms FPS:{fps:03.0f}", (5, 150))
    rtext(showFrame, f"W:{cols} H:{rows}", (5, 180))
    cv.imshow(window_title, showFrame)

    if k == ord("q"):
        break

cv.destroyAllWindows()
cap.release()
