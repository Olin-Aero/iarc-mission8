import numpy as np
import operator

# following functions ONLY used during BoxUtils initialization.
# to avoid circular dependencies.
def _pmap(m, p):
    return reduce(operator.or_, [m[k] for k in p])
def _pmap_i(m_i, p):
    return ''.join(s.lower() for (f,s) in m_i.items() if (f&p))

def _merge_format(fmt):
    return reduce(lambda x,y: ((x<<8)|y), fmt)
def _split_format(fmt):
    return [(fmt >> (8*i)) & 0b11111111 for i in reversed(range(4))]

def _encode_format(m, fmt):
    return _merge_format( [_pmap(m, p) for p in fmt] )
def _decode_format(m_i, fmt):
    return [_pmap_i(m_i,p) for p in _split_format(fmt)]

class BoxUtils(object):
    """
    Box Utilities for handling automatic conversions.
    This is to prevent any confusion regarding the box specifications.
    Note that this was mostly implemented as me having fun with encoding boxes efficiently.
    Incredibly over-engineered.
    """
    # L-CX-R, T-CY-B

    # Generalized Properties
    FMT_X  = 0b00000001 # x-property flag
    FMT_Y  = 0b00000010 # y-property flag
    FMT_0  = 0b00000100 # 0 (min-value) flag
    FMT_1  = 0b00001000 # 1 (max-value) flag
    FMT_C  = 0b00010000 # C (center-value) flag
    FMT_S  = 0b00100000 # S (scale-value) flag

    # global normalization flag, repeated for all box fields
    FMT_N  = _merge_format([0b01000000 for _ in range(4)])

    # Masks
    MSK_A = 0b00000011 # Axis Mask
    MSK_V = 0b00111100 # Value Mask

    # Property map
    PMAP={'n':FMT_N, 'x':FMT_X, 'y':FMT_Y, '0':FMT_0, '1':FMT_1, 'c':FMT_C, 's':FMT_S}
    PMAP.update({k.upper():v for (k,v) in PMAP.items()})
    PMAP_I={v:k for (k,v) in PMAP.items()}

    # Default X-Property
    FMT_X0 = FMT_X | FMT_0         # left-x
    FMT_XC = FMT_X | FMT_C         # center-x
    FMT_R = FMT_X1 = FMT_X | FMT_1 # right-x
    FMT_W = FMT_XS = FMT_X | FMT_S # width-x

    # Default Y-Property
    FMT_T = FMT_Y0 = FMT_Y | FMT_0 # top-y
    FMT_YC = FMT_Y | FMT_C         # center-y
    FMT_B = FMT_Y1 = FMT_Y | FMT_1 # bottom-y
    FMT_H = FMT_YS = FMT_Y | FMT_S # height-y

    # provide some popular configurations
    FMT_XYXY = _encode_format(PMAP, ['x0', 'y0', 'x1', 'y1'])
    FMT_YXYX = _encode_format(PMAP, ['y0', 'x0', 'y1', 'x1'])
    FMT_CCWH = _encode_format(PMAP, ['xc', 'yc', 'xs', 'ys'])
    FMT_XYWH = _encode_format(PMAP, ['x0', 'y0', 'xs', 'ys']) 

    # provide equivalent normalized configurations
    FMT_NXYXY = FMT_XYXY | FMT_N
    FMT_NYXYX = FMT_YXYX | FMT_N
    FMT_NCCWH = FMT_CCWH | FMT_N
    FMT_NXYWH = FMT_XYWH | FMT_N


    """ Box normalization """
    @staticmethod
    def normalize(box, fmt, img_shape):
        ss = []
        for f in _split_format(fmt):
            s = (img_shape[0] if (f & BoxUtils.FMT_Y) else img_shape[1])
            ss.append(s)
        return np.divide(box, ss)

    @staticmethod
    def unnormalize(box, fmt, img_shape):
        ss = []
        for f in _split_format(fmt):
            s = (img_shape[0] if (f & BoxUtils.FMT_Y) else img_shape[1])
            ss.append(s)
        return np.multiply(box, ss)

    """ Handle Value Format Properties """
    @staticmethod
    def pmap(p):
        return _pmap(BoxUtils.PMAP, p)
    @staticmethod
    def encode_format(fmt):
        return _encode_format(BoxUtils.PMAP, fmt)
    @staticmethod
    def decode_format(fmt):
        return _decode_format(BoxUtils.PMAP_I, fmt)

    """ Conversion Helpers """
    @staticmethod
    def _first(fs, default=None):
        for f in fs:
            try:
                return f()
            except:
                continue
        else:
            return default
    @staticmethod
    def _convert_1(d_in, fmt_out):
        fmts = [BoxUtils.FMT_0, BoxUtils.FMT_1, BoxUtils.FMT_C, BoxUtils.FMT_S]
        data = {}
        for fi, vi in d_in.items():
            for vmsk in [BoxUtils.FMT_0, BoxUtils.FMT_1, BoxUtils.FMT_C, BoxUtils.FMT_S]:
                if fi & vmsk:
                    data[vmsk] = vi

        fs = []
        if (fmt_out & BoxUtils.FMT_0):
            fs = [
                    lambda: data[BoxUtils.FMT_0],
                    lambda: data[BoxUtils.FMT_1] - data[BoxUtils.FMT_S],
                    lambda: data[BoxUtils.FMT_C] - 0.5*data[BoxUtils.FMT_S],
                    lambda: 2.0 * data[BoxUtils.FMT_C] - data[BoxUtils.FMT_1]
                    ]

        if (fmt_out & BoxUtils.FMT_1):
            fs = [
                    lambda: data[BoxUtils.FMT_1],
                    lambda: data[BoxUtils.FMT_0] + data[BoxUtils.FMT_S],
                    lambda: data[BoxUtils.FMT_C] + 0.5*data[BoxUtils.FMT_S],
                    lambda: 2.0 * data[BoxUtils.FMT_C] - data[BoxUtils.FMT_0]
                    ]

        if (fmt_out & BoxUtils.FMT_C):
            fs = [
                    lambda: data[BoxUtils.FMT_C],
                    lambda: 0.5 * (data[BoxUtils.FMT_0] + data[BoxUtils.FMT_1]),
                    lambda: data[BoxUtils.FMT_0] + 0.5*data[BoxUtils.FMT_S],
                    lambda: data[BoxUtils.FMT_1] - 0.5*data[BoxUtils.FMT_S],
                    ]

        if (fmt_out & BoxUtils.FMT_S):
            fs = [
                    lambda: data[BoxUtils.FMT_S],
                    lambda: data[BoxUtils.FMT_1] - data[BoxUtils.FMT_0],
                    lambda: 2.0 * (data[BoxUtils.FMT_1] - data[BoxUtils.FMT_C]),
                    lambda: 2.0 * (data[BoxUtils.FMT_C] - data[BoxUtils.FMT_0])
                    ]

        return BoxUtils._first(fs)

    @staticmethod
    def _convert(box_in, fmt_in, fmt_out):
        # populate input dict
        d_in  = {f:b for (f,b) in zip(_split_format(fmt_in), np.swapaxes(box_in, 0, -1))}
        d_out = dict(d_in) # copy to d_out

        vos = []
        for fo in _split_format(fmt_out):
            # select relevant properties
            d_sel = {fi:b for (fi, b) in d_in.items() if (fi & fo & BoxUtils.MSK_A)}

            # attempt conversion
            vo = BoxUtils._convert_1(d_sel, fo)
            if vo is None:
                return None

            # add to result
            vos.append(vo)

        return np.stack(vos, axis=-1)

    @staticmethod
    def convert(box_in, fmt_in, fmt_out, img_shape=None):
        """
        Handle Box conversions between multiple encoded formats.

        Arguments:
            box_in  : A(..., 4); array-like with 4 channels corresponding to fmt_in.
            fmt_in  : uint32; Input Box Format according to BoxUtils encoding.
            fmt_out : uint32; Output Box Format according to BoxUtils encoding.
        Returns:
            box_in : A(..., 4); array-like with 4 channels corresponding to fmt_out.
            (None if conversion is impossible)
        """
        # determine flag for normalization requirement
        need_shape = ((fmt_in & BoxUtils.FMT_N) ^ (fmt_out & BoxUtils.FMT_N))
        if need_shape and (img_shape is None):
            # TODO : Exceptions? or None?
            print('Cannot convert between normalization without image shape')
            return None
        
        box_out = BoxUtils._convert(box_in, fmt_in, fmt_out)

        # finalize output with normalization
        if (box_out is not None) and need_shape:
            if (fmt_out & BoxUtils.FMT_N):
                box_out = BoxUtils.normalize(box_out, fmt_out, img_shape)
            else:
                box_out = BoxUtils.unnormalize(box_out, fmt_out, img_shape)

        return box_out

def main():
    import time
    # test box utils
    fmts = [BoxUtils.FMT_XYXY, BoxUtils.FMT_YXYX,
            BoxUtils.FMT_CCWH, BoxUtils.FMT_XYWH,
            BoxUtils.FMT_NXYXY, BoxUtils.FMT_NYXYX,
            BoxUtils.FMT_NCCWH, BoxUtils.FMT_NXYWH]

    fis = [np.random.choice(fmts)]
    fos = [np.random.choice(fmts)]
    #fis = fmts
    #fos = fmts

    img_shape = (480,640,3)
    num_boxes = (400,400)

    for fmt_in in fis:
        for fmt_out in fos:
            box_in = np.random.uniform(size=num_boxes + (4,))

            if not (fmt_in & BoxUtils.FMT_N):
                BoxUtils.unnormalize(box_in, fmt_in, img_shape)

            t0 = time.time()
            box_out = BoxUtils.convert(box_in, fmt_in, fmt_out,
                    img_shape=img_shape
                    )

            box_in_r = BoxUtils.convert(box_out,
                    fmt_out, fmt_in,
                    img_shape=img_shape
                    )
            t1 = time.time()
            dt = t1 - t0
            print('From {} -> {}'.format(':'.join(BoxUtils.decode_format(fmt_in)), ':'.join(BoxUtils.decode_format(fmt_out))))
            print('Took {} seconds (= {} per box)'.format(dt, 4*dt/box_in.size ))
            print('Reconstruction Error', np.linalg.norm(box_in - box_in_r))

if __name__ == "__main__":
    main()
