# Copyright 2024 - Andrew Kwok Fai LUI, 
# Robotics and Autonomous Systems Group, REF, RI
# and the Queensland University of Technology

__author__ = 'Andrew Lui'
__copyright__ = 'Copyright 2024'
__license__ = 'GPL'
__version__ = '1.0'
__email__ = 'ak.lui@qut.edu.au'
__status__ = 'Development'

from io import BytesIO
from collections import namedtuple
import cv2
import numpy as np

# -------------------------------
def resize_to_height(image, new_height):
    """
Args:
    image: the image to be resize
    new_height: the new height of the resized image
Returns:
    the resized image    
"""
    if image is None:
        return None
    height = image.shape[0]
    new_width = int(image.shape[1] * (new_height / height))
    dim = (new_width, new_height)
    resized = cv2.resize(image, dim, interpolation = cv2.INTER_AREA)
    return resized
# -------------------------------
def extract_masked_points(image_mask, mask_value=255):
    """
Args:
    image_mask (nparray or cvimage): 2d nparray of integers 
    mask_value (bool): the value representing the mask
Returns:
    list: a list of points (tuples of (row, col)) of the mask
"""
    #data = np.array(image_mask)[:,:,0]
    wpoint = np.where(image_mask == mask_value)
    points = set((row, col) for row, col in zip(*wpoint)) # (row, col)
    return points
# --- INTERNAL USE: generating the index of the 8 neighours of a pixel
def generate_neighbours(point):
    neighbours = [ (1, -1), (1, 0),(1, 1),(0, -1), (0, 1), (1, -1), (1, 0),(-1, 1) ]
    for neigh in neighbours:
        yield tuple(map(sum, zip(point, neigh)))
# ---- extract the connected region starting from a seed point
def extract_region(seed, points): 
    """Gather a connected region from the points of mask, starting from the seed
Args:
    image_mask (nparray or cvimage): 2d nparray of integers 
    mask_value (bool): the value representing the mask
Returns:
    list: a list of points (tuples of (row, col)) of the mask
"""
    region_points = []   # in (row, col)
    seen_points = set()
    the_seeds = [seed]
    while len(the_seeds) > 0:
        point = the_seeds.pop()
        if point not in seen_points:
            seen_points.add(point)
            if point in points:
                region_points.append(point)               
                points.remove(point)
                for n in generate_neighbours(point):
                    the_seeds.append(n)
    region_points = np.asarray(region_points)
    min_point = np.min(region_points, axis=0)
    max_point = np.max(region_points, axis=0)
    bbox = np.hstack((min_point, max_point)) # bbox is a nparray
    return region_points, bbox
# ---- return the score of similarity between two masks
def compare_masks(mask1, mask2):
    drow, dcol = min(mask1.shape[0], mask2.shape[0]), min(mask1.shape[1], mask2.shape[1])
    score = (mask1[:drow, :dcol] == mask2[:drow, :dcol]).sum() / (drow * dcol)
    return score
# ---- create a new image of shape (row, col) and copy the mask referenced at the center
def copy_and_pad(mask1, row, col):
    assert(row >= mask1.shape[0] and col >= mask1.shape[1])
    r = (row - mask1.shape[0]) // 2
    c = (col - mask1.shape[1]) // 2 
    image = np.zeros(shape=(row, col))
    image[r:r+mask1.shape[0], c:c+mask1.shape[1]] = mask1
    return image.astype(np.bool)             
# ---- return the jaccard shape similarity between two masks 
def jaccard_masks(mask1, mask2):
    drow, dcol = max(mask1.shape[0], mask2.shape[0]), max(mask1.shape[1], mask2.shape[1])
    im1 = copy_and_pad(mask1, drow, dcol)
    im2 = copy_and_pad(mask2, drow, dcol)
    intersection = np.logical_and(im1, im2)
    union = np.logical_or(im1, im2)
    return intersection.sum() / float(union.sum())
# ---- return the jaccard location (overlap) similarity between two masks 
def jaccard_bbox(bbox1, bbox2):
    bbox1 = bbox1.flatten()
    bbox2 = bbox2.flatten()
    xA = max(bbox1[0], bbox2[0])
    yA = max(bbox1[1], bbox2[1])
    xB = min(bbox1[2], bbox2[2])
    yB = min(bbox1[3], bbox2[3])
    interArea = abs(max((xB - xA, 0)) * max((yB - yA), 0))
    if interArea == 0:
        return 0
    boxAArea = abs((bbox1[2] - bbox1[0]) * (bbox1[3] - bbox1[1]))
    boxBArea = abs((bbox2[2] - bbox2[0]) * (bbox2[3] - bbox2[1]))
    iou = interArea / float(boxAArea + boxBArea - interArea)
    return iou

# --- generates offset of positions that form a spiral out of the origin (0, 0)
def spiral_iterator(iteration_limit=999):
    x, y = 0, 0
    layer, leg, iteration = 1, 0, 0
    yield 0, 0
    while iteration < iteration_limit:
        iteration += 1
        if leg == 0:
            x += 1
            if (x == layer): leg += 1
        elif leg == 1:
            y += 1
            if (y == layer):  leg += 1
        elif leg == 2:
            x -= 1
            if -x == layer: leg += 1
        elif leg == 3:
            y -= 1
            if -y == layer:
                leg = 0
                layer += 1
        yield x, y
# ---- return a square bbox of a specified length in a masked region
#      that is nearest to the centre of mass
#      if no such bbox is found, the length is reduced and search again
#      mask: the mask array
def get_bbox_near_centre(mask, minlength):
    pass

# --- convert numpy arry to byte array
def array_to_bytes(x: np.ndarray) -> bytes:
    np_bytes = BytesIO()
    np.save(np_bytes, x, allow_pickle=True)
    return np_bytes.getvalue()

def bytes_to_array(b: bytes) -> np.ndarray:
    np_bytes = BytesIO(b)
    return np.load(np_bytes, allow_pickle=True)

# ---------------------------------------------------
# Template Matching Functions

TEMPLATE_MATCH_RESULT = namedtuple('TEMPLATE_MATCH_RESULT', ['bbox', 'center', 'score'])

def is_overlap_bbox(bbox1, bbox2):
    if (bbox1[0] >= bbox2[2]) or (bbox1[2] <= bbox2[0]) or (bbox1[3] <= bbox2[1]) or (bbox1[1] >= bbox2[3]):
        return False
    return True

def match_result_single(image_search, template, mode=cv2.TM_CCOEFF_NORMED):
    assert image_search is not None and len(image_search.shape) == 2, 'the parameter image_search should be greyscale'
    assert template is not None and len(template.shape) == 2, 'the parameter template should be greyscale'        
    res = cv2.matchTemplate(image_search, template, mode)
    w, h = template.shape[::-1]
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
    top_left = list(max_loc)
    #bottom_right = [top_left[0] + w, top_left[1] + h]
    bbox = (top_left[0], top_left[1], top_left[0] + w, top_left[1] + h)
    center = (bbox[0] + bbox[2]) // 2, (bbox[1] + bbox[3]) // 2
    max_val = max_val // (w * h)
    return TEMPLATE_MATCH_RESULT(bbox, center, max_val)

def search_overlap(results, bbox):
    for r in results:
        if is_overlap_bbox(r.bbox, bbox):
            return r
    return None

def match_result_multi(image_search, template, mode=cv2.TM_CCOEFF_NORMED, threshold=0.8):
    assert image_search is not None and len(image_search.shape) == 2, 'the parameter image_search should be greyscale'
    assert template is not None and len(template.shape) == 2, 'the parameter template should be greyscale'  
    res = cv2.matchTemplate(image_search, template, mode)
    w, h = template.shape[::-1]
    loc = np.where(res >= threshold)

    results = []
    for top_left in zip(*loc[::-1]):
        bbox = (top_left[0], top_left[1], top_left[0] + w, top_left[1] + h)
        center = (bbox[0] + bbox[2]) // 2, (bbox[1] + bbox[3]) // 2
        score = res[top_left[1], top_left[0]]
        found = search_overlap(results, bbox)
        if found is not None:
            if score > found.score:
                results.remove(found)
                results.append(TEMPLATE_MATCH_RESULT(bbox, center, score))
        else:
            results.append(TEMPLATE_MATCH_RESULT(bbox, center, score))
    return results

# ---------------------------------------------------
# Drawing Functions
# ---- draw a crisscross at center (y, x) or (row, col)
def draw_overlay_crisscross(overlay, center, color, size=5, thickness=4):
    # cv2.line uses (x, y) coordinates
    assert(overlay is not None)
    if (center[0]-size<0) or (center[1]-size<0) or (center[0]+size >= overlay.shape[0]) or(center[1]+size >= overlay.shape[1]):
        return overlay
    cy, cx = int(center[0]), int(center[1])
    overlay = cv2.line(overlay, (cx-size, cy-size), (cx+size, cy+size), color, thickness)
    overlay = cv2.line(overlay, (cx-size, cy+size), (cx+size, cy-size), color, thickness)
    return overlay
