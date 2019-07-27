uint8 FMT_XYXY=0 #Left, Top, Right, Bottom
uint8 FMT_YXYX=1 #Top, Left, Bottom, Right
uint8 FMT_CCWH=2 #Center X, Center Y, Width, Height
uint8 FMT_XYWH=3 #Left, Top, Width, Height

uint8 format
float32[4] data
bool normalized # flag to indicate normalized box : 0.0-1.0
