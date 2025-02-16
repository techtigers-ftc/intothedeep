import cv2
import numpy as np
import math
import time
from collections import defaultdict


# Track OpenCV function calls and timing
opencv_stats = defaultdict(lambda: {"count": 0, "total_time": 0})


DEBUG = False


def log(*args):
    if DEBUG:
        print(*args)


def track_opencv(func_name):
    def decorator(func):
        def wrapper(*args, **kwargs):
            start = time.time()
            result = func(*args, **kwargs)
            end = time.time()
            opencv_stats[func_name]["count"] += 1
            opencv_stats[func_name]["total_time"] += end - start
            return result

        return wrapper

    return decorator


# Wrap commonly used OpenCV functions
cv2.split = track_opencv("split")(cv2.split)
cv2.cvtColor = track_opencv("cvtColor")(cv2.cvtColor)
cv2.inRange = track_opencv("inRange")(cv2.inRange)
cv2.bitwise_and = track_opencv("bitwise_and")(cv2.bitwise_and)
cv2.bitwise_or = track_opencv("bitwise_or")(cv2.bitwise_or)
cv2.bitwise_not = track_opencv("bitwise_not")(cv2.bitwise_not)
cv2.morphologyEx = track_opencv("morphologyEx")(cv2.morphologyEx)
cv2.GaussianBlur = track_opencv("GaussianBlur")(cv2.GaussianBlur)
cv2.Sobel = track_opencv("Sobel")(cv2.Sobel)
cv2.Canny = track_opencv("Canny")(cv2.Canny)
cv2.findContours = track_opencv("findContours")(cv2.findContours)
cv2.drawContours = track_opencv("drawContours")(cv2.drawContours)
cv2.bilateralFilter = track_opencv("bilateralFilter")(cv2.bilateralFilter)
cv2.normalize = track_opencv("normalize")(cv2.normalize)
cv2.dilate = track_opencv("dilate")(cv2.dilate)
cv2.contourArea = track_opencv("contourArea")(cv2.contourArea)


# Edge detection parameters - initial values
BLUR_SIZE = 13
SOBEL_KERNEL = 7


GAUSSIAN_BLUR_KERNEL_SIZE = (5, 5)
MORPHOLOGY_KERNEL = np.ones((5, 5), np.uint8)
DILATE_KERNEL = np.ones((3, 3), np.uint8)


# Color detection ranges for different color spaces
HSV_BLUE_RANGE = ([90, 70, 20], [140, 255, 255])
HSV_RED_RANGE_1 = ([0, 70, 20], [20, 255, 255])  # Red wraps around in HSV
HSV_RED_RANGE_2 = ([150, 70, 20], [180, 255, 255])
HSV_YELLOW_RANGE = ([10, 70, 150], [30, 255, 255])


# Constants for filtering contours
SMALL_CONTOUR_AREA_FINE = 12000
SMALL_CONTOUR_AREA_COARSE = 200


# Minimum average brightness threshold (0-255)
MIN_BRIGHTNESS_THRESHOLD = 20


# Drawing color
FONT_NAME = cv2.FONT_HERSHEY_SIMPLEX
FONT_SIZE = 0.25
FONT_THICKNESS = 1


COLOR_RED = (0, 0, 255)
COLOR_GREEN = (0, 255, 0)
COLOR_INFO = (255, 255, 0)
COLOR_ERROR = (0, 0, 255)


def calculate_angle(contour):
    if len(contour) < 5:
        return 0
    (x, y), (MA, ma), angle = cv2.fitEllipse(contour)
    return angle - 90


def draw_info(image, color, angle, center, index, area):
    x = center[0]
    y = center[1]
    rotated_angle = math.radians(angle)
    angle_x = x + int(50 * math.cos(rotated_angle))
    angle_y = y + int(50 * math.sin(rotated_angle))

    cv2.putText(
        image,
        f"Angle: {angle:.2f}",
        (x - 40, y - 40),
        FONT_NAME,
        FONT_SIZE,
        COLOR_INFO,
        FONT_THICKNESS,
    )
    cv2.putText(
        image,
        f"Area: {area:.2f}",
        (x - 40, y - 20),
        FONT_NAME,
        FONT_SIZE,
        COLOR_INFO,
        FONT_THICKNESS,
    )
    cv2.circle(image, center, 5, COLOR_INFO, -1)
    cv2.line(image, center, (angle_x, angle_y), COLOR_INFO, 2)


def explore_touching_contours(frame, contour, min_area_ratio=0.15):
    x, y, w, h = cv2.boundingRect(contour)
    mask = np.zeros((h, w), dtype=np.uint8)
    shifted_contour = contour - [x, y]
    cv2.drawContours(mask, [shifted_contour], -1, 255, -1)

    original_area = cv2.contourArea(contour)
    max_contours = []
    max_count = 1

    dist_transform = cv2.distanceTransform(mask, cv2.DIST_L2, 3)

    for threshold in np.linspace(0.1, 0.9, 9):
        _, thresh = cv2.threshold(
            dist_transform, threshold * dist_transform.max(), 255, 0
        )
        thresh = np.uint8(thresh)

        contours, _ = cv2.findContours(
            thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        print(f"Contour count = {len(contours)})")

    return frame


def separate_touching_contours(contour, min_area_ratio=0.15):
    x, y, w, h = cv2.boundingRect(contour)
    mask = np.zeros((h, w), dtype=np.uint8)
    shifted_contour = contour - [x, y]
    cv2.drawContours(mask, [shifted_contour], -1, 255, -1)

    original_area = cv2.contourArea(contour)
    max_contours = []
    max_count = 1

    dist_transform = cv2.distanceTransform(mask, cv2.DIST_L2, 3)

    for threshold in np.linspace(0.1, 0.9, 9):
        _, thresh = cv2.threshold(
            dist_transform, threshold * dist_transform.max(), 255, 0
        )
        thresh = np.uint8(thresh)

        contours, _ = cv2.findContours(
            thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        print(f"Contour count = {len(contours)})")

        valid_contours = [
            contour
            for contour in contours
            if cv2.contourArea(contour) > original_area * min_area_ratio
        ]

        if len(valid_contours) > max_count:
            max_count = len(valid_contours)
            max_contours = valid_contours

    if max_contours:
        return [contour + [x, y] for contour in max_contours]

    return [contour]


def runPipeline(frame, llrobot):
    llrobot[0] = 1
    llrobot[3] = 0
    try:
        usingYellow = llrobot[0] == 1
        usingRed = llrobot[1] == 1
        usingBlue = llrobot[2] == 1
        isFine = llrobot[3] == 0

        llpython = [0, 0, 0, 0, 0, 0, 0, 0]
        largest_contour = np.array([[]])

        # Convert to HSV and denoise
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Split and display HSV channels
        h, s, v = cv2.split(hsv)

        hsv_denoised = cv2.GaussianBlur(hsv, GAUSSIAN_BLUR_KERNEL_SIZE, 0)
        hsv_denoised = hsv

        # Create masks for each color
        blue_mask = cv2.inRange(
            hsv_denoised, np.array(HSV_BLUE_RANGE[0]), np.array(HSV_BLUE_RANGE[1])
        )

        red_mask1 = cv2.inRange(
            hsv_denoised, np.array(HSV_RED_RANGE_1[0]), np.array(HSV_RED_RANGE_1[1])
        )
        red_mask2 = cv2.inRange(
            hsv_denoised, np.array(HSV_RED_RANGE_2[0]), np.array(HSV_RED_RANGE_2[1])
        )
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)

        yellow_mask = cv2.inRange(
            hsv_denoised, np.array(HSV_YELLOW_RANGE[0]), np.array(HSV_YELLOW_RANGE[1])
        )

        # Combine all color masks
        height, width = blue_mask.shape
        combined_mask = np.zeros((height, width), dtype=np.uint8)
        if usingRed:
            combined_mask = cv2.bitwise_or(red_mask, combined_mask)
        if usingYellow:
            combined_mask = cv2.bitwise_or(yellow_mask, combined_mask)
        if usingBlue:
            combined_mask = cv2.bitwise_or(blue_mask, combined_mask)

        masked_frame = cv2.bitwise_and(frame, frame, mask=combined_mask)

        gray_masked = cv2.cvtColor(masked_frame, cv2.COLOR_BGR2GRAY)
        # Edge detection pipeline
        blurred = cv2.GaussianBlur(gray_masked, (BLUR_SIZE, BLUR_SIZE), 0)

        sobelx = cv2.Sobel(blurred, cv2.CV_32F, 1, 0, ksize=SOBEL_KERNEL)
        sobely = cv2.Sobel(blurred, cv2.CV_32F, 0, 1, ksize=SOBEL_KERNEL)

        magnitude = np.sqrt(sobelx**2 + sobely**2)
        magnitude = np.uint8(magnitude * 255 / np.max(magnitude))

        # Threshold the magnitude image
        _, edges = cv2.threshold(magnitude, 50, 255, cv2.THRESH_BINARY)

        edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, MORPHOLOGY_KERNEL)

        edges = cv2.dilate(edges, DILATE_KERNEL, iterations=1)

        edges = cv2.bitwise_not(edges)
        edges = cv2.bitwise_and(edges, edges, mask=combined_mask)

        contours, hierarchy = cv2.findContours(
            edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        contour_frame = frame.copy()

        game_pieces = []
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        contours_to_select_from = []

        for i, contour in enumerate(contours):
            small_contour_area = (
                SMALL_CONTOUR_AREA_FINE if isFine == 1 else SMALL_CONTOUR_AREA_COARSE
            )
            if cv2.contourArea(contour) < small_contour_area:
                continue

            frame = explore_touching_contours(frame, contour)
            for sep_contour in separate_touching_contours(contour):
                mask = np.zeros(gray.shape, dtype=np.uint8)
                cv2.drawContours(mask, [sep_contour], -1, 255, -1)

                if cv2.mean(gray, mask=mask)[0] < MIN_BRIGHTNESS_THRESHOLD:
                    continue

                angle = calculate_angle(sep_contour)
                M = cv2.moments(sep_contour)
                if M["m00"] != 0:
                    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                else:
                    continue

                area = cv2.contourArea(sep_contour)

                vertices = len(sep_contour)

                contours_to_select_from.append([sep_contour, center])

                color = COLOR_GREEN if hierarchy[0][i][3] == -1 else COLOR_RED

                cv2.drawContours(frame, [sep_contour], 0, color, 2)

                draw_info(frame, "Blue", angle, center, i, area)

                game_pieces.append(
                    {
                        "index": len(game_pieces) + 1,
                        "color": "Blue",
                        "position": center,
                        "angle": angle,
                        "area": area,
                        "hierarchy_level": "external"
                        if hierarchy[0][i][3] == -1
                        else "internal",
                    }
                )

        def dist_for_fine(center):
            return (width / 2 - center[0]) ** 2 + (height / 2 - center[1]) ** 2

        def dist_for_coarse(center):
            return height - center[1]

        if isFine:
            dist_func = dist_for_fine
        else:
            dist_func = dist_for_coarse

        min_dist = 10000000000
        for contour, center in contours_to_select_from:
            dist = dist_func(center)
            if dist < min_dist:
                min_dist = dist
                largest_contour = contour

        if len(game_pieces) > 0:
            llpython = [1, center[0], center[1], angle, 0, 0, 0, 0]

        return largest_contour, frame, llpython

    except Exception as e:
        print(e)
        cv2.putText(
            frame,
            f"Error: {str(e)}",
            (10, 30),
            FONT_NAME,
            FONT_SIZE,
            COLOR_ERROR,
            FONT_THICKNESS,
        )
        return np.array([[]]), frame, [0, 0, 0, 0, 0, 0, 0, 0]