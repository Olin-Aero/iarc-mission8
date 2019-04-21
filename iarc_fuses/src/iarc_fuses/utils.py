import numpy as np
import cv2

def rint(x):
    """Rounds a float to a 32 bit integer
    
    Args:
        x (float): Input value
    
    Returns:
        np.int32: Rounded int32
    """
    return np.int32(np.round(x))
def anorm(x):
    """Normalizes an angle to exist between -pi and pi
    
    Args:
        x (float): Angle in radians
    
    Returns:
        float: Angularly normalized angle
    """
    return (x+np.pi) % (2*np.pi) - np.pi

def adiff(a,b):
    """Differnece between two angles, a and b represented in radians
    
    Args:
        a (float): Angle a in radians
        b (float): Angle b in radians
    
    Returns:
        float: Normalized difference
    """
    return anorm(a-b)

def convert_to_relative(image_size, bounding_box):
    """Converts an absolute bounding box into a relative one given an image image_size
    
    Args:
        image_size (2-tuple): Size of form (width, height)
        bounding_box (4-tuple): Pixel defined bounding box of object, in 
                form (center_x, center_y, width, height)

    Returns:
        4-tuple(float): Returns a relative bounding box of type (center_x, center_y, width, height) 
    """
    print(image_size)
    print(bounding_box)
    return (bounding_box[0]/image_size[0], bounding_box[1]/image_size[1], 
            bounding_box[2]/image_size[0], bounding_box[3]/image_size[1])

def convert_to_pixels(image_size, bounding_box):
	"""Converts a relative bounding box into a bounding box defined using pixels
	given an image size
	
	Args:
	    image_size (2-tuple): Size of form (width, height)
        bounding_box (4-tuple): Relative defined bounding box of object, in 
                form (center_x, center_y, width, height)
	
	Returns:
	    4-tuple(float): Returns a pixel defined bounding box of type (center_x, center_y, width, height)
	"""
	return (int(bounding_box[0] * image_size[0]),
			int(bounding_box[1] * image_size[1]),
			int(bounding_box[2] * image_size[0]),
			int(bounding_box[3] * image_size[1]))

def draw_bbox(img, box, cls=None):
    """ Draw a yxyx-encoded box """
    h,w = img.shape[:2]
    yxyx = box
    yxyx = np.multiply(yxyx, [h,w,h,w])
    yxyx = np.round(yxyx).astype(np.int32)
    y0,x0,y1,x1 = yxyx
    cv2.rectangle(img, (x0,y0), (x1,y1), (255,0,0), thickness=2)
    if cls is not None:
        org = ( max(x0,0), min(y1,h) )
        cv2.putText(img, cls, org, 
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,0,255),
                1, cv2.LINE_AA
                )

def iarc_root():
    try:
        import rospkg
        root = rospkg.RosPack().get_path('iarc_fuses')
    except Exception:
        import os
        script_path = os.path.dirname(__file__)
        root = os.path.join(script_path, '../..')
        root = os.path.realpath(root)
    return root

def translate_contour(contour, (x, y)):
  return contour + [x,y]

def inRangeWrap(img, low, high):
  """
  Like cv2.inRange, but can do colors that wrap around the hue range, like reds.
  If low hue <= high hue, it takes everything in the range (low hue, high hue), like normal.
  If low hue > high hue, it takes everything *outside* the range (high hue, low hue).
  """
  if low[0] <= high[0]:
    return cv2.inRange(img, low, high)
  else:
    lowRange = cv2.inRange(img, [0, low[1], low[2]], high)
    highRange = cv2.inRange(img, low, [255, high[1], high[2]])
    return lowRange | highRange
