__all__ = ['BoxUtils', 'draw_bbox', 'expand_box',
           'crop_box', 'inner_box', 'box_ixn', 'box_area', 'box_iou']

import numpy as np
import operator
import cv2
import functools

# following functions ONLY used during BoxUtils initialization.
# to avoid circular dependencies.


def _pmap(m, p):
    return functools.reduce(operator.or_, [m[k] for k in p])


def _pmap_i(m_i, p):
    return ''.join(s.lower() for (f, s) in m_i.items() if (f & p))


def _merge_format(fmt):
    return functools.reduce(lambda x, y: ((x << 8) | y), fmt)


def _split_format(fmt):
    return [(fmt >> (8 * i)) & 0b11111111 for i in reversed(range(4))]


def _encode_format(m, fmt):
    return _merge_format([_pmap(m, p) for p in fmt])


def _decode_format(m_i, fmt):
    return [_pmap_i(m_i, p) for p in _split_format(fmt)]


class BoxUtils(object):

    """
    Box Utilities for handling automatic conversions.
    This is to prevent any confusion regarding the box specifications.
    Note that this was mostly implemented as me having fun with encoding boxes efficiently.
    Incredibly over-engineered.
    """
    # L-CX-R, T-CY-B

    # Generalized Properties
    FMT_X = 0b00000001  # x-property flag
    FMT_Y = 0b00000010  # y-property flag
    FMT_0 = 0b00000100  # 0 (min-value) flag
    FMT_1 = 0b00001000  # 1 (max-value) flag
    FMT_C = 0b00010000  # C (center-value) flag
    FMT_S = 0b00100000  # S (scale-value) flag

    # global normalization flag, repeated for all box fields
    FMT_N = _merge_format([0b01000000 for _ in range(4)])

    # Masks
    MSK_A = 0b00000011  # Axis Mask
    MSK_V = 0b00111100  # Value Mask

    # Property map
    PMAP = {'n': FMT_N, 'x': FMT_X, 'y': FMT_Y,
            '0': FMT_0, '1': FMT_1, 'c': FMT_C, 's': FMT_S}
    PMAP.update({k.upper(): v for (k, v) in PMAP.items()})
    PMAP_I = {v: k for (k, v) in PMAP.items()}

    # Default X-Property
    FMT_X0 = FMT_X | FMT_0         # left-x
    FMT_XC = FMT_X | FMT_C         # center-x
    FMT_R = FMT_X1 = FMT_X | FMT_1  # right-x
    FMT_W = FMT_XS = FMT_X | FMT_S  # width-x

    # Default Y-Property
    FMT_T = FMT_Y0 = FMT_Y | FMT_0  # top-y
    FMT_YC = FMT_Y | FMT_C         # center-y
    FMT_B = FMT_Y1 = FMT_Y | FMT_1  # bottom-y
    FMT_H = FMT_YS = FMT_Y | FMT_S  # height-y

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
                    lambda: data[BoxUtils.FMT_C] - 0.5 * data[BoxUtils.FMT_S],
                    lambda: 2.0 * data[BoxUtils.FMT_C] - data[BoxUtils.FMT_1]
            ]

        if (fmt_out & BoxUtils.FMT_1):
            fs = [
                lambda: data[BoxUtils.FMT_1],
                    lambda: data[BoxUtils.FMT_0] + data[BoxUtils.FMT_S],
                    lambda: data[BoxUtils.FMT_C] + 0.5 * data[BoxUtils.FMT_S],
                    lambda: 2.0 * data[BoxUtils.FMT_C] - data[BoxUtils.FMT_0]
            ]

        if (fmt_out & BoxUtils.FMT_C):
            fs = [
                lambda: data[BoxUtils.FMT_C],
                    lambda: 0.5 *
                        (data[BoxUtils.FMT_0] + data[BoxUtils.FMT_1]),
                    lambda: data[BoxUtils.FMT_0] + 0.5 * data[BoxUtils.FMT_S],
                    lambda: data[BoxUtils.FMT_1] - 0.5 * data[BoxUtils.FMT_S],
            ]

        if (fmt_out & BoxUtils.FMT_S):
            fs = [
                lambda: data[BoxUtils.FMT_S],
                    lambda: data[BoxUtils.FMT_1] - data[BoxUtils.FMT_0],
                    lambda: 2.0 *
                        (data[BoxUtils.FMT_1] - data[BoxUtils.FMT_C]),
                    lambda: 2.0 * (data[BoxUtils.FMT_C] - data[BoxUtils.FMT_0])
            ]

        return BoxUtils._first(fs)

    @staticmethod
    def _convert(box_in, fmt_in, fmt_out):
        # populate input dict
        d_in = {f: b for (f, b) in zip(
            _split_format(fmt_in), np.swapaxes(box_in, 0, -1))}
        d_out = dict(d_in)  # copy to d_out

        vos = []
        for fo in _split_format(fmt_out):
            # select relevant properties
            d_sel = {
                fi: b for (fi, b) in d_in.items() if (fi & fo & BoxUtils.MSK_A)}

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
        if (fmt_in == fmt_out):
            return box_in

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


""" Additional Box Utilities """


def draw_bbox(img, box, fmt=BoxUtils.FMT_NYXYX, cls=None):
    """ Draw an encoded box """
    # convert to draw-able box
    box = BoxUtils.convert(box,
                           fmt,
                           BoxUtils.FMT_XYXY,
                           img.shape
                           )

    x0, y0, x1, y1 = [int(e) for e in box]
    cv2.rectangle(img, (x0, y0), (x1, y1), (255, 0, 0), thickness=2)
    if cls is not None:
        org = (max(x0, 0), min(y1, h))
        cv2.putText(img, cls, org,
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255),
                    1, cv2.LINE_AA
                    )


def expand_box(box, margin, fmt=BoxUtils.FMT_NYXYX):
    fmt_tmp = BoxUtils.FMT_CCWH | (fmt & BoxUtils.FMT_N)
    box = BoxUtils.convert(box, fmt, fmt_tmp)
    box[..., 2:] *= (1.0 + margin)
    return BoxUtils.convert(box, fmt_tmp, fmt)


def crop_box(img, box, margin=0.0, fmt=BoxUtils.FMT_NYXYX):
    h, w = img.shape[:2]
    if margin > 0:
        box = expand_box(box, margin)
    box = BoxUtils.convert(box, fmt, BoxUtils.FMT_NYXYX,
                           img_shape=img.shape)
    box = np.clip(box, 0.0, 1.0)
    box = BoxUtils.convert(box, BoxUtils.FMT_NYXYX, BoxUtils.FMT_YXYX)
    y0, x0, y1, x1 = box.astype(np.int32)
    return img[y0:y1, x0:x1], box


def inner_box(box, subbox,
              fmt=BoxUtils.FMT_NYXYX,
              fmt_sub=BoxUtils.FMT_NYXYX
              ):
    # format input
    box = BoxUtils.convert(box, fmt, BoxUtils.FMT_NYXYX)
    subbox = BoxUtils.convert(subbox, fmt, BoxUtils.FMT_NYXYX)

    # process
    y0, x0, y1, x1 = [box[..., i] for i in range(4)]
    bh = y1 - y0
    bw = x1 - x0
    o = np.stack([y0, x0, y0, x0], axis=-1)  # origin (top-left)
    s = np.stack([bh, bw, bh, bw], axis=-1)  # scale (hwhw)
    d = np.multiply(subbox, s)  # delta (bottom-right)
    box = o + d

    # format output
    return BoxUtils.convert(box, BoxUtils.FMT_NYXYX, fmt)


def box_ixn(box0, box1,
            fmt0=BoxUtils.FMT_NYXYX,
            fmt1=BoxUtils.FMT_NYXYX
            ):
    # format input
    fmt0_tmp = BoxUtils.FMT_YXYX | (fmt0 & BoxUtils.FMT_N)
    fmt1_tmp = BoxUtils.FMT_YXYX | (fmt1 & BoxUtils.FMT_N)
    box0 = BoxUtils.convert(box0, fmt0, fmt0_tmp)
    box1 = BoxUtils.convert(box1, fmt1, fmt1_tmp)

    # process
    ymin = max(box0[..., 0], box1[..., 0])
    xmin = max(box0[..., 1], box1[..., 1])
    ymax = min(box0[..., 2], box1[..., 2])
    xmax = min(box0[..., 3], box1[..., 3])
    box = np.stack([ymin, xmin, ymax, xmax], axis=-1)

    # format output
    return BoxUtils.convert(box,
                            BoxUtils.FMT_YXYX | (fmt0 & BoxUtils.FMT_N),
                            fmt0)


def box_area(box, fmt=BoxUtils.FMT_NYXYX):
    fmt_tmp = BoxUtils.FMT_XYWH | (fmt & BoxUtils.FMT_N)
    box = BoxUtils.convert(box, fmt, fmt_tmp)
    w = box[..., 2]
    h = box[..., 3]
    return np.abs(w * h)


def box_iou(box0, box1,
            fmt0=BoxUtils.FMT_NYXYX,
            fmt1=BoxUtils.FMT_NYXYX):
    box0 = np.asarray(box0)
    box1 = np.asarray(box1)
    ixn = box_area(box_ixn(box0, box1, fmt0, fmt1), fmt0)
    uxn = box_area(box0, fmt0) + box_area(box1, fmt1) - ixn
    return (ixn / np.float32(uxn))


def main():
    import time
    # test box utils
    fmts = [BoxUtils.FMT_XYXY, BoxUtils.FMT_YXYX,
            BoxUtils.FMT_CCWH, BoxUtils.FMT_XYWH,
            BoxUtils.FMT_NXYXY, BoxUtils.FMT_NYXYX,
            BoxUtils.FMT_NCCWH, BoxUtils.FMT_NXYWH]

    fis = [np.random.choice(fmts)]
    fos = [np.random.choice(fmts)]
    # fis = fmts
    # fos = fmts

    img_shape = (480, 640, 3)
    num_boxes = (400, 400)

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
            print('From {} -> {}'.format(
                ':'.join(BoxUtils.decode_format(fmt_in)), ':'.join(BoxUtils.decode_format(fmt_out))))
            print(
                'Took {} seconds (= {} per box)'.format(dt, 4 * dt / box_in.size))
            print('Reconstruction Error', np.linalg.norm(box_in - box_in_r))

if __name__ == "__main__":
    main()
