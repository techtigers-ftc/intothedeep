import cv2
import numpy as np
import math

inited = False
bbox = None
hasBeenTracking = False
tracker = None
theta = 0
updating = 0
restriction = []

# the periodic function
def runPipeline(image, llrobot):
    if sum(llrobot) != 0:
        restriction = llrobot
    global inited
    global tracker
    global hasBeenTracking
    global bbox
    global theta
    global updating

    updating +=1

    # check whether the tracker has been initialized
    if not inited:
        res = get_best_box(image)
        # was the tracker tracking in the last frame and has something to track?
        if res and hasBeenTracking:
            x, y, w, h, theta = res
            print(x-w/2, y-h/2, w, h)
            # bounding box
            bbox = (x-w/2, y-h/2, h, w)
            bbox =  list(map(int, bbox))

            # object tracker
            tracker = cv2.TrackerCSRT_create()
            try:
                tracker.init(image, bbox)
                inited = True
            except:
                pass
        # we can track now
        elif res:
            hasBeenTracking = True
        # we are no longer tracking
        else:
            hasBeenTracking = False

        print("C-1: Not Inited", updating)
        return np.array([[[bbox]]]), image, [-1, -1, -1, -1, -1, updating]
    # if we do not have something to track, find the closest block
    else:
        success, bbox = tracker.update(image)
        if not success:
            inited = False
            print("C-1: Inited, Image Absent", updating)
            return np.array([[[]]]), image, [-1, -1, -1, -1, -1, updating]
        else:

            x, y, w, h = map(int, bbox)
            # [x, y, x + w, y + h]
            res = get_best_box(image)
            if res:
                _, _, _, _, theta = res
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            print("Inited, Image Found", x, y, w, h, theta, updating)
            return np.array([[bbox]]), image, [x, y, w, h, theta, updating]

# get the bounding box of the best block to select
def get_best_box(image):
    global restriction
    # if restriction_provided != None:
    #     (x1, y1, x2, y2) = restriction_provided
    #     image = image[x1:x2][y1:y2]
    if restriction != []:
        (x1, y1, x2, y2) = restriction
        image = image[x1:x2][y1:y2]
    # convert to csv
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # color masking
    lower_bound = np.array([8, 160, 120])
    upper_bound = np.array([40, 255, 255])
    mask = cv2.inRange(hsv, lower_bound, upper_bound)

    # blur + erode
    kernel = np.ones((3, 3), np.uint8)
    opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)

    sure_bg = cv2.dilate(opening, kernel, iterations=3)
    dist_transform = cv2.distanceTransform(opening, cv2.DIST_L2, 5)

    # split blocks that are close together
    _, sure_fg = cv2.threshold(dist_transform, 0.47 * dist_transform.max(), 255, 0)
    sure_fg = np.uint8(sure_fg)

    unknown = cv2.subtract(sure_bg, sure_fg)

    _, markers = cv2.connectedComponents(sure_fg)

    markers = markers + 1
    markers[unknown == 0] = 0

    # perform watershed transform to not have contours that contain multiple blocks
    markers = cv2.watershed(image, markers)
    image[markers == -1] = [0, 0, 255]

    contours, _ = cv2.findContours(sure_fg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # get the properties of each block
    for contour in contours:
        rect = cv2.minAreaRect(contour)
        (center, dims, angle) = rect
        area = dims[0] * dims[1]
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        top = max(box, key=lambda x: x[1])
        right  = max(box, key=lambda x: x[0])
        angle = math.atan((top[1]-right[1])/(top[0] - right[0]))
        cv2.drawContours(image, [box], 0, (0, 255, 0), 2)

    # distance on the frame of the contour to the place where the extension will reach
    def blockDist(contour):
            center = cv2.minAreaRect(contour)[0]
            return math.sqrt((center[0]-320)**2 + (center[1]-400)**2)


    largestContour = min(contours, key=blockDist) if contours else None
    # print(type(largestContour))


    if type(largestContour) != np.ndarray:
        return False

    # get pose from a contour
    def getPose(contour):
        rect = cv2.minAreaRect(contour)
        (center, dims, angle) = rect
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        # steps to get the angle
        top = max(box, key=lambda x: x[1])
        right  = max(box, key=lambda x: x[0])
        left = min(box, key=lambda x: x[0])
        angle = math.degrees(math.atan((right[1]-top[1])/(right[0] - top[0])))
        if dist(top, right) > dist(top, left):
            angle=-angle
        else:
            angle=90-angle
        return [*center, *dims, angle]
    (center, dims, angle) = cv2.minAreaRect(largestContour)
    if dims[0] * dims[1] < 200:
        return False
    return getPose(largestContour)

# euclidean distance
def dist(p1, p2):
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)