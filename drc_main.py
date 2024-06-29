import cv2 as cv
import numpy as np
import time
import math
import os

remote_display = True
# display_scale = 0.5
display_scale = 2

os.environ["DISPLAY"] = "mcrn-tachi.local:0" if remote_display else ":0"

window_title = "DRC Pathfinder"


def mouse_event(event, x, y, flags, param):
    global mouseX, mouseY
    if event == cv.EVENT_MOUSEMOVE:
        mouseX, mouseY = int(x), int(y)


def process_key(key):
    global currentlyShowing

    if key == ord("1"):
        currentlyShowing = -1
    if key == ord("2"):
        currentlyShowing = -2
    if key == ord("3"):
        currentlyShowing = -3
    if key == ord("4"):
        currentlyShowing = -4
    if key == ord("5"):
        currentlyShowing = -5
    if key == ord("6"):
        currentlyShowing = -6
    if key == ord("7"):
        currentlyShowing = -7
    if key == ord("8"):
        currentlyShowing = -8
    if key == ord("9"):
        currentlyShowing = -9
    if key == ord("0"):
        currentlyShowing = -10
    if key == ord("-"):
        currentlyShowing = -11
    if key == ord("="):
        currentlyShowing = -12
    if key == ord("g"):
        currentlyShowing = 0
    if key == ord("n"):
        currentlyShowing = 1
    if key == ord("s"):
        currentlyShowing = 2
    if key == ord("v"):
        currentlyShowing = 3
    if key == ord("h"):
        currentlyShowing = 4


def update_pcx(val):
    global pathfinding_centre_x
    global image_centre_x
    pathfinding_centre_x = image_centre_x + (val - 200)


def rtext(img, text, org, col=(0, 0, 0), border=(255, 255, 255), scale=1):
    cv.putText(
        img,
        text,
        org,
        cv.FONT_HERSHEY_SIMPLEX,
        scale,
        border,
        4 * scale,
        cv.LINE_AA,
    )
    cv.putText(
        img,
        text,
        org,
        cv.FONT_HERSHEY_SIMPLEX,
        scale,
        col,
        2 * scale,
        cv.LINE_AA,
    )


# DEBUGGING + DISPLAY
currentlyShowing = 0

mouseX = 0
mouseY = 0

# THRESHOLDING
blu_hsv = (150, 220, 170)
ylw_hsv = (38, 98, 200)
mgnta_hsv = (0, 0, 0)
red_hsv = (0, 0, 0)

blu_hsv_thresh_range = (20, 50, 100)
ylw_hsv_thresh_range = (20, 50, 50)
mgnta_hsv_thresh_range = (0, 0, 0)
red_hsv_thresh_range = (0, 0, 0)

# BGR color fills for image edge
blu_fill_col = blu_hsv
ylw_fill_col = ylw_hsv
col_denoise_kernel_rad = 3

# DENOISING/CLEANDING KERNELS
colour_denoise_kernel = cv.getStructuringElement(
    cv.MORPH_RECT, ((col_denoise_kernel_rad * 2) - 1, (col_denoise_kernel_rad * 2) - 1)
)

path_open_kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (5, 5))
# path_dilate_kernel = cv.getStructuringElement(cv.MORPH_RECT, (3, 11))
path_dilate_kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))

# PARAMETERS
derivative_kernel_size = 21
path_threshold_val = 90
path_slice_height = 5

initial_blur_size = 31

# more image proportions
# if absolute error is les
path_error_switch_start = 0.5

# filtering/slicing image proportions
horizontal_cutoff_dist = 0.5
future_path_height = 0.35
path_min_area_proportion = 0.02 * 0.1

# robot config parameters
path_failsafe_time = 0.5  # TODO

Kp = 1
Ki = 0
Kd = 0

setpoint = 0

cap = cv.VideoCapture(0)
if not cap.isOpened():
    print("Cannot open camera")
    exit()

# picIn = cv.imread("test4.png")  # TODO TESTING ONLY
_, picIn = cap.read()

rows, cols, channels = picIn.shape
image_centre_x = int(cols / 2)
image_centre_y = int(rows / 2)
pathfinding_centre_x = image_centre_x
horizontal_cutoff_dist_px = int(horizontal_cutoff_dist * cols / 2)
future_path_offset_px = int(rows * future_path_height)
path_minimum_area = int(path_min_area_proportion * rows * cols)

bottom_slice_mask = np.zeros((rows, cols), np.uint8)
bottom_slice_mask[rows - path_slice_height : rows, :] = 1
path_cutoff_height = rows - path_slice_height

setpoint_px = int(image_centre_x + (setpoint * cols / 2))

blu_hsv_low = np.array(
    [pair[0] - pair[1] for pair in zip(blu_hsv, blu_hsv_thresh_range)]
)
blu_hsv_high = np.array(
    [pair[0] + pair[1] for pair in zip(blu_hsv, blu_hsv_thresh_range)]
)

ylw_hsv_low = np.array(
    [pair[0] - pair[1] for pair in zip(ylw_hsv, ylw_hsv_thresh_range)]
)
ylw_hsv_high = np.array(
    [pair[0] + pair[1] for pair in zip(ylw_hsv, ylw_hsv_thresh_range)]
)

mgnta_hsv_low = np.array(
    [pair[0] - pair[1] for pair in zip(mgnta_hsv, mgnta_hsv_thresh_range)]
)
mgnta_hsv_high = np.array(
    [pair[0] + pair[1] for pair in zip(mgnta_hsv, mgnta_hsv_thresh_range)]
)

red_hsv_low = np.array(
    [pair[0] - pair[1] for pair in zip(red_hsv, red_hsv_thresh_range)]
)
red_hsv_high = np.array(
    [pair[0] + pair[1] for pair in zip(red_hsv, red_hsv_thresh_range)]
)

path_max_y_check = int(rows * 0.49)
path_mask_widen_end_y = int(rows * 0.5)
path_mask_widen_start_y = int(rows * 0.95)
path_mask = np.zeros((rows, cols), np.uint8)
path_mask_contour = np.array(
    [
        # [0, 0],
        # [cols - 1, 0],
        # [cols - 1, path_mask_y2],
        # [0, path_mask_y1],
        [pathfinding_centre_x - horizontal_cutoff_dist_px, rows - 1],
        [pathfinding_centre_x - horizontal_cutoff_dist_px, path_mask_widen_start_y],
        [0, path_mask_widen_end_y],
        [0, path_max_y_check],
        [cols - 1, path_max_y_check],
        [cols - 1, path_mask_widen_end_y],
        [pathfinding_centre_x + horizontal_cutoff_dist_px, path_mask_widen_start_y],
        [pathfinding_centre_x + horizontal_cutoff_dist_px, rows - 1],
    ],
    dtype=np.int32,
)
path_mask_contour = path_mask_contour.reshape((-1, 1, 2))
# cv.polylines(path_mask, [path_mask_contour], True, 255)
cv.drawContours(path_mask, [path_mask_contour], 0, 255, -1)

# FAILSAFE
last_time_path_seen = time.time()

# OPENCV WINDOW
cv.namedWindow(window_title, cv.WINDOW_GUI_NORMAL)
cv.resizeWindow(window_title, int(cols * display_scale), int(rows * display_scale))
if not remote_display:
    cv.moveWindow(window_title, 0, -20)
cv.setMouseCallback(window_title, mouse_event)
cv.createTrackbar("Path Xo", window_title, 200, 400, update_pcx)

while True:
    key = cv.waitKey(1)
    process_key(key)
    if key == ord("q"):
        break

    start_time = time.time()
    # IMAGE INPUT
    # input_frame = picIn.copy()
    ret, input_frame = cap.read()
    if ret == False:
        break
    capture_time = time.time()
    # input_frame = cv.blur(input_frame, (initial_blur_size, initial_blur_size))
    input_frame = cv.GaussianBlur(
        input_frame, (initial_blur_size, initial_blur_size), 0
    )

    hsvImg = cv.cvtColor(input_frame, cv.COLOR_BGR2HSV_FULL)
    hsvImg[:, 0:col_denoise_kernel_rad] = blu_fill_col
    hsvImg[:, cols - col_denoise_kernel_rad : cols] = ylw_fill_col

    # calculate thresholded masks for various colours
    blu_mask = cv.inRange(hsvImg, blu_hsv_low, blu_hsv_high)
    ylw_mask = cv.inRange(hsvImg, ylw_hsv_low, ylw_hsv_high)
    mgnta_mask = cv.inRange(hsvImg, mgnta_hsv_low, mgnta_hsv_high)
    # combine the masks
    track_boundaries_mask = cv.bitwise_xor(ylw_mask, blu_mask)
    avoid_mask = cv.bitwise_xor(track_boundaries_mask, mgnta_mask)
    # and denoise
    avoid_mask = cv.morphologyEx(avoid_mask, cv.MORPH_OPEN, colour_denoise_kernel)

    # distanceTransform gives distance from nearest *zero pixel*, not from nearest *white* pixel, so it needs to be inverted
    # ksize 3 gives really bad results for some reason even though it is slightly faster
    distance_plot = cv.distanceTransform(cv.bitwise_not(avoid_mask), cv.DIST_L2, 5)

    # take second-order horizontal Sobel derivative of the image
    # This converts the "peaks" in the Voronoi diagram into significantly negative regions
    # or, after normalisation, local (and global!) minima
    # raw_derivative = cv.Sobel(
    #     distance_plot, cv.CV_32F, 2, 0, ksize=derivative_kernel_size
    # )
    # the Laplacian being the sum of horiz + vertical 2nd derivatives *looks* useful,
    # but is in fact quite slow and breaks a lot of stuff due to top and bottom edges
    raw_derivative = cv.Laplacian(
        distance_plot, cv.CV_32F, ksize=derivative_kernel_size
    )

    # normalise the insane values that the derivative produces to u8 range
    normalised_horiz_derivative = cv.normalize(
        raw_derivative, None, 0, 255, cv.NORM_MINMAX, cv.CV_8UC1
    )
    # only the minimum is needed, but maybe in the future use the coords output to auto-choose the best path?
    # would mean no more weighting though...
    # bottom slice just used to prevent any weirdness in the top part of the image from throwing the results
    (
        minimum_derivative,
        maximum_derivative,
        minimum_derivative_coords,
        maximum_derivative_coords,
    ) = cv.minMaxLoc(normalised_horiz_derivative, bottom_slice_mask)

    # threshold based on the minimum found gets us the "ridgelines" in the distance plot
    raw_paths_binary = cv.inRange(normalised_horiz_derivative, 0, path_threshold_val)
    # opening (erode/dilate) removes the "strings" produced by the diagonal lines
    # denoised_paths_mask = cv.morphologyEx(
    #     raw_paths_mask, cv.MORPH_OPEN, path_open_kernel
    # )
    # finally a mostly-vertical dilation re-joins paths that sometimes split after the open operation
    # final_paths_mask = cv.dilate(denoised_paths_mask, path_dilate_kernel)
    final_paths_binary = cv.dilate(raw_paths_binary, path_dilate_kernel)
    final_paths_binary = cv.bitwise_and(final_paths_binary, path_mask)
    final_paths_binary[rows - 5 : rows, :] = final_paths_binary[rows - 6, :]

    # find all the contours - since they should all be separate lines and only one is chosen,
    # heirarchy can get thrown away
    contours, _ = cv.findContours(
        final_paths_binary, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE
    )

    # centroids = [None] * len(contours)
    bounding_boxes = [None] * len(contours)
    potential_path_contours = []

    chosen_path_idx = -1
    closest_path_dist = 9999999

    # TODO maybe could cut this down to only the bottom/middle slices?
    # might interfere with bounding boxes but could be decent optimisation

    # locate valid potential path contours
    for i in range(len(contours)):
        contour = contours[i]
        # calculate x/y centre of contour using moments
        contour_moment = cv.moments(contour, True)
        # add 1e-5 to avoid division by zero
        centroid_x = int(contour_moment["m10"] / (contour_moment["m00"] + 1e-5))
        # centroid_y = int(contour_moment["m01"] / (contour_moment["m00"] + 1e-5))
        # centroids[i] = (centroid_x, centroid_y)

        # calculate bounding box coordinates
        bounding_box = cv.boundingRect(contour)
        bounding_boxes[i] = bounding_box
        tr_x, tr_y, w, h = bounding_box
        bl_x = tr_x + w
        bl_y = tr_y + h

        path_dist = min(
            abs(pathfinding_centre_x - bl_x),
            abs(pathfinding_centre_x - tr_x),
            abs(pathfinding_centre_x - centroid_x),
        )
        # if the path starts in the bottom rows,
        # AND if any of the contour corners or its centroid are
        # within range of the path choice centreline
        # AND it's not tiny
        if (
            bl_y >= path_cutoff_height
            and path_dist < horizontal_cutoff_dist_px
            and cv.contourArea(contour) > path_minimum_area
        ):
            # then it's an option for a path
            potential_path_contours.append(i)
            # want to choose the path closest to the current path centreline
            # if it's closer, update the new distance and index
            if path_dist < closest_path_dist:
                closest_path_dist = path_dist
                chosen_path_idx = i

    # create a mask image with only the selected path in it
    chosen_path_binary = np.zeros_like(final_paths_binary)

    # pathfinding output vars
    current_path_x = pathfinding_centre_x
    future_path_x = current_path_x
    path_lost = False
    short_path_warn = True

    # robot failsafe stop
    should_stop = False

    # failsafe if no path detected
    if chosen_path_idx < 0:
        if (time.time() - last_time_path_seen) > path_failsafe_time:
            should_stop = True
        path_lost = True
    else:
        last_time_path_seen = time.time()
        cv.drawContours(chosen_path_binary, contours, chosen_path_idx, 255, cv.FILLED)

        # and then slice that and use moments to get the x coordinates of the start and middle of the path
        current_path_slice = chosen_path_binary[(rows - path_slice_height) : rows, :]
        current_path_moment = cv.moments(current_path_slice, True)
        current_path_x = int(
            current_path_moment["m10"] / (current_path_moment["m00"] + 1e-5)
        )
        future_path_slice_start = rows - future_path_offset_px
        future_path_slice = chosen_path_binary[
            future_path_slice_start : (future_path_slice_start + path_slice_height), :
        ]
        future_path_moment = cv.moments(
            future_path_slice,
            True,
        )
        future_path_x = current_path_x
        if future_path_moment["m10"] > 1:
            future_path_x = int(
                future_path_moment["m10"] / (future_path_moment["m00"] + 1e-5)
            )
            short_path_warn = False

    end_time = time.time()

    # RENDERING
    match currentlyShowing:
        case -1:
            txt = "BLU"
            display_frame = cv.cvtColor(blu_mask, cv.COLOR_GRAY2BGR)
        case -2:
            txt = "YLW"
            display_frame = cv.cvtColor(ylw_mask, cv.COLOR_GRAY2BGR)
        case -3:
            txt = "MAG"
            display_frame = cv.cvtColor(mgnta_mask, cv.COLOR_GRAY2BGR)
        case -4:
            txt = "CMB"
            display_frame = cv.cvtColor(avoid_mask, cv.COLOR_GRAY2BGR)
        case -5:
            txt = "DIST"
            combinedDistFrame = cv.normalize(
                distance_plot,
                None,
                0,
                255,
                cv.NORM_MINMAX,
                cv.CV_8UC1,
            )
            display_frame = cv.cvtColor(combinedDistFrame, cv.COLOR_GRAY2BGR)
        case -6:
            txt = "DERV"
            display_frame = cv.cvtColor(normalised_horiz_derivative, cv.COLOR_GRAY2BGR)
        case -7:
            txt = "CPATH"
            display_frame = cv.cvtColor(chosen_path_binary, cv.COLOR_GRAY2BGR)
        case -8:
            txt = "RPTM"
            display_frame = cv.cvtColor(raw_paths_binary, cv.COLOR_GRAY2BGR)
        case -9:
            txt = "FPTM"
            display_frame = cv.cvtColor(final_paths_binary, cv.COLOR_GRAY2BGR)
        case -10:
            txt = "ALCTR"
            display_frame = cv.cvtColor(final_paths_binary, cv.COLOR_GRAY2BGR)
            cv.drawContours(display_frame, contours, -1, (0, 255, 0), 1, cv.LINE_AA)
            # for centroid in centroids:
            #     cv.drawMarker(
            #         display_frame, centroid, (255, 0, 0), cv.MARKER_CROSS, 11, 3
            #     )
            for x, y, w, h in bounding_boxes:
                cv.rectangle(
                    display_frame, (x, y), (x + w, y + h), (0, 0, 255), 2, cv.LINE_AA
                )
        case -11:
            txt = "MASK"
            display_frame = cv.cvtColor(path_mask, cv.COLOR_GRAY2BGR)
        case -12:
            txt = "PTCTR"
            display_frame = cv.cvtColor(final_paths_binary, cv.COLOR_GRAY2BGR)

            cv.drawContours(
                display_frame,
                contours,
                chosen_path_idx,
                (255, 255),
                -1,
                cv.LINE_AA,
            )
            for i in potential_path_contours:
                cv.drawContours(
                    display_frame,
                    contours,
                    i,
                    (0, 0, 255),
                    2,
                    cv.LINE_AA,
                )
        case 0:
            txt = "RGB"
            display_frame = input_frame
        case 1:
            txt = "Hue"
            hsvHueOnly = hsvImg.copy()
            hsvHueOnly[:, :, 1] = 255
            hsvHueOnly[:, :, 2] = 255
            hsvHueOnly = cv.cvtColor(hsvHueOnly, cv.COLOR_HSV2BGR_FULL)
            display_frame = hsvHueOnly
        case 2:
            txt = "Sat"
            display_frame = cv.cvtColor(hsvImg[:, :, 1], cv.COLOR_GRAY2BGR)
        case 3:
            txt = "Val"
            display_frame = cv.cvtColor(hsvImg[:, :, 2], cv.COLOR_GRAY2BGR)
        case 4:
            txt = "HSV"
            display_frame = hsvImg
        case _:
            txt = "Dflt"
            display_frame = input_frame

    main_path_error_px = current_path_x - setpoint_px
    future_path_error_px = future_path_x - current_path_x

    # draw calculated pathfinding markers on final image
    cv.line(
        display_frame,
        (pathfinding_centre_x, 0),
        (pathfinding_centre_x, rows),
        (255, 0, 255),
        2,
    )
    cv.line(
        display_frame,
        (pathfinding_centre_x - horizontal_cutoff_dist_px, rows - 1),
        (pathfinding_centre_x + horizontal_cutoff_dist_px, rows - 1),
        (255, 0, 255),
        4,
    )
    cv.drawMarker(
        display_frame,
        (current_path_x, rows - 10),
        (255, 0, 0),
        cv.MARKER_DIAMOND,
        11,
        3,
    )
    cv.drawMarker(
        display_frame,
        (future_path_x, rows - future_path_offset_px),
        (255, 0, 0),
        cv.MARKER_DIAMOND,
        11,
        3,
    )

    # pixel is in BGR!
    mouseX = min(max(mouseX, 0), cols - 1)
    mouseY = min(max(mouseY, 0), rows - 1)

    # show some pixel info on mouse hover for debugging
    rgbPixel = input_frame[mouseY, mouseX]
    hsvPixel = hsvImg[mouseY, mouseX]
    samplePixel = display_frame[mouseY, mouseX]

    # render debug info
    rtext(display_frame, txt, (5, 30))
    rtext(
        display_frame,
        f"X:{mouseX:04d}, Y:{mouseY:04d}",
        (5, 60),
    )
    rtext(
        display_frame,
        f"R{rgbPixel[2]:03d} G{rgbPixel[1]:03d} B{rgbPixel[0]:03d}",
        (5, 90),
    )
    rtext(
        display_frame,
        f"H{hsvPixel[0]:03d} S{hsvPixel[1]:03d} V{hsvPixel[2]:03d}",
        (5, 120),
    )
    rtext(display_frame, f"W:{cols} H:{rows}", (5, 180))
    rtext(
        display_frame,
        f"R{samplePixel[2]:03.0f} G{samplePixel[1]:03.0f} B{samplePixel[0]:03.0f}",
        (5, 210),
    )
    rtext(
        display_frame,
        f"ERR: {main_path_error_px:+04.0f} {future_path_error_px:+04.0f}",
        (5, 240),
    )

    warn_text = ""
    if short_path_warn:
        warn_text = "NO FUTURE PATH"

    if len(warn_text) > 0:
        rtext(
            display_frame,
            f"WARN: {warn_text.upper()}",
            (5, 270),
            (0, 100, 255),
            (0, 0, 0),
        )

    if should_stop:
        rtext(
            display_frame,
            f"PATH LOSS TIMEOUT - STOP",
            (5, image_centre_y),
            (0, 0, 255),
            (0, 0, 0),
            2,
        )
    elif path_lost:
        rtext(
            display_frame,
            f"PATH LOST",
            (5, image_centre_y),
            (0, 180, 255),
            (0, 0, 0),
            2,
        )

    proc_dt = end_time - capture_time
    full_dt = end_time - start_time
    proc_time_ms = proc_dt * 1000
    full_time_ms = proc_dt * 1000
    fps = 1 / proc_dt
    rtext(
        display_frame,
        # f"P,F:{proc_time_ms:03.0f},{full_time_ms:03.0f} FPS:{fps:03.0f}",
        f"dt:{full_time_ms:03.0f}ms FPS:{fps:03.0f}",
        (5, 150),
        col=((0, 0, 0) if fps > 25 else (0, 0, 255)),
    )

    # display_frame = cv.resize(
    #     display_frame, (int(cols * display_scale), int(rows * display_scale))
    # )
    cv.imshow(window_title, display_frame)

cv.destroyAllWindows()
cap.release()
