import numpy as np
import os
import six.moves.urllib as urllib
import tarfile
import tensorflow as tf
import cv2
import time

from iarc_fuses.utils import draw_bbox, iarc_root

class ObjectDetectorTF(object):
    """
    Thin wrapper around the Tensorflow Object Detection API.
    Heavily inspired by the [provided notebook][1].

    Note:
        [1]: https://github.com/tensorflow/models/blob/master/research/object_detection/object_detection_tutorial.ipynb
    """
    def __init__(self,
            root='/tmp',
            #model='ssd_mobilenet_v1_coco_11_06_2017',
            model='ssd_mobilenet_v1_ppn_shared_box_predictor_300x300_coco14_sync_2018_07_03',
            use_gpu=False,
            cmap=None,
            shape=(640,640,3)
            ):
        """
        Arguments:
            root(str): persistent data directory; override to avoid initialization overhead.
            model(str): model name; refer to the [model zoo][2].
            use_gpu(bool): Enable vision processing execution on the GPU.
            cmap(dict): Alternative class definitions; remap known classes for convenience.
            shape(tuple): (WxHx3) shape used for warmup() to pre-allocate tensor.

        Note:
            [2]: https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md
        """
        # hard-coded auxiliary model
        # downloadable from google drive
        self.aux_model_ = {
                'model2-drone-300x300' : '1voCDNghyKCNzk7Q9c4fzj23lW5jQtoTp'
                }

        # cache arguments
        self.root_  = root
        self.model_ = model
        self.use_gpu_ = use_gpu
        self.shape_ = shape

        # load model
        ckpt_file  = self._maybe_download_ckpt()
        self.graph_ = self._load_graph(ckpt_file)
        self.input_, self.output_ = self._build_pipeline(self.graph_)
        self.cmap_ = cmap

        self.initialize()

    def __call__(self, img):
        """
        Run object detection.

        Arguments:
            img(A(N?,W,H,3)): image to run inference on; may optionally be batched.
        Returns:
            output(dict): {'box':A(N,M,4), 'class':A(N,M), 'score':A(N,M)}
        """
        if img.ndim == 3:
            # single image configuration
            outputs = self.__call__(img[None,...])
            return {k:v[0] for k,v in outputs.iteritems()}
        outputs = self.run(self.output_, {self.input_:img})
        if self.cmap_ is not None:
            # map to alternative class definitions
            outputs['class'] = [[ str(self.cmap_[x] if (x in self.cmap_) else x) for x in xs] for xs in outputs['class']]
            outputs['class'] = np.array(outputs['class'], dtype=str)
        return outputs
    
    def _download_from_gdrive(self):
        try:
            from google_drive_downloader import GoogleDriveDownloader as gd
            gd.download_file_from_google_drive(
                    file_id=self.aux_model_[self.model_],
                    dest_path=os.path.join(self.root_, self.model_, 'frozen_inference_graph.pb'),
                    unzip=True)
        except Exception as e:
            print('Downloading From GDrive Failed : {}'.format(e))

    def _download_from_tfzoo(self,
            model_tar_basename,
            model_tar_fullpath
            ):
        # fetch from web if tar file does not exist
        if not os.path.exists(model_tar_fullpath):
            print('downloading model ...'.format(self.model_))
            download_base = 'http://download.tensorflow.org/models/object_detection/'
            opener = urllib.request.URLopener()
            opener.retrieve(download_base+model_tar_basename, model_tar_fullpath)

        # extract if graph does not exist
        tar_file = tarfile.open(model_tar_fullpath)
        for file in tar_file.getmembers():
            file_name = os.path.basename(file.name)
            if 'frozen_inference_graph.pb' in file_name:
                tar_file.extract(file, self.root_)

    def _maybe_download_ckpt(self):
        """
        WARN: internal.

        Check if model file exists; download if not available.
        Returns:
            ckpt_file(str): path to the checkpoint, from which to load graph.
        """
        ckpt_file = os.path.join(self.root_, self.model_,
                'frozen_inference_graph.pb')
        print('ckpt_file', ckpt_file)

        if not os.path.exists(ckpt_file):
            if self.model_ in self.aux_model_:
                # fetch from google drive
                self._download_from_gdrive()
            else:
                # fetch from tf zoo
                model_tar_basename = '{}.tar.gz'.format(self.model_)
                model_tar_fullpath = os.path.join(self.root_, model_tar_basename)
                self._download_from_tfzoo(
                        model_tar_basename,
                        model_tar_fullpath)
        return ckpt_file

    def _load_graph(self, ckpt_file):
        """
        WARN: internal.

        Load Graph from file.
        Arguments:
            ckpt_file(str): result from _maybe_download_ckpt()
        Returns:
            graph(tf.Graph): computational tensorflow Graph
        """
        detection_graph = tf.Graph()
        with detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(ckpt_file, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
        return detection_graph

    def _build_pipeline(self, graph):
        """
        WARN: internal.

        Parse graph into inputs and outputs.
        Arguments:
            graph(tf.Graph): result from _load_graph()
        Returns:
            x(tf.Placeholder): NHWC input image tensor (batched)
            y(dict): output(dict): {'box':A(N,M,4), 'class':A(N,M), 'score':A(N,M), 'num':M=A(N)}
        """
        x = None
        y = {}
        with graph.as_default():
            get = lambda s: graph.get_tensor_by_name(s)
            x = get('image_tensor:0')
            y['box'] = get('detection_boxes:0')
            y['score'] = get('detection_scores:0')
            y['class'] = get('detection_classes:0')
            y['num'] = get('num_detections:0')
        return x, y

    def warmup(self):
        """ Allocate resources on the GPU as a warm-up """
        self.__call__(np.zeros(shape=(1,)+self.shape_, dtype=np.uint8))

    def initialize(self):
        """ Create Session and warmup """
        with self.graph_.as_default():
            if self.use_gpu_:
                self.sess_ = tf.Session(graph=self.graph_)
            else:
                self.sess_ = tf.Session(
                        config=tf.ConfigProto(device_count={'GPU': 0}),
                        graph=self.graph_
                        )
        self.warmup()

    def run(self, *args, **kwargs):
        return self.sess_.run(*args, **kwargs)

def test_image():
    """
    Simple test script; requires /tmp/image1.jpg
    """
    app = ObjectDetectorTF(
            root=os.path.join(iarc_root(), 'data'),
            #model='model1-drone-640x640',
            model='model2-drone-300x300',
            #model='model3-drone-640x640',
            cmap={1:'DRONE'},
            use_gpu=False 
            )
    img = cv2.imread('/tmp/test.png')
    print 'img', img.shape
    h,w = img.shape[:2]
    res = app(img[...,::-1])
    msk = (res['score'] > 0.5)

    cls   = res['class'][msk]
    box   = res['box'][msk]
    score = res['score'][msk]
    print 'score', score

    for box_, cls_, val_ in zip(box, cls, score):
        #ry0,rx0,ry1,rx1 = box_ # relative
        print 'box', box
        draw_bbox(img, box_, '{}:{:2f}'.format(cls_, val_))

    cv2.imshow('win', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def box_ixn(box0, box1):
    # ensure ndarray
    box0, box1 = np.asarray(box0), np.asarray(box1)

    ymin = max(box0[..., 0], box1[..., 0])
    xmin = max(box0[..., 1], box1[..., 1])
    ymax = min(box0[..., 2], box1[..., 2])
    xmax = min(box0[..., 3], box1[..., 3])

    return np.stack([ymin,xmin,ymax,xmax], axis=-1)

def box_area(box):
    h = box[...,2] - box[...,0]
    w = box[...,3] - box[...,1]
    return np.abs(w*h)

def box_iou(box0, box1):
    #TRAINED_CKPT_PREFIX="/tmp/model.ckpt-38683"
    ixn = box_area(box_ixn(box0, box1))
    uxn = box_area(box0) + box_area(box1) - ixn
    return (ixn / np.float32(uxn))

def test_images():
    """
    Simple test script; requires /tmp/image1.jpg
    """
    
    app = ObjectDetectorTF(
            root=os.path.join(iarc_root(), 'data'),
            #model='model1-drone-640x640',
            model='model2-drone-300x300',
            #model='model3-drone-640x640',
            cmap={1:'DRONE'},
            use_gpu=False 
            )

    imgdir = os.path.expanduser(
            '~/Repos/drone-net/image'
            #'~/Pictures'
            #"/home/jamie/libs/yolo-9000/darknet/data"
            #'/tmp/simg'
            )

    cv2.namedWindow('win', cv2.WINDOW_NORMAL)

    fs = os.listdir(imgdir)
    np.random.shuffle(fs)

    fs = [os.path.expanduser('~/Pictures/swarm2.jpg')]
    
    for f in fs:
        f = os.path.join(imgdir, f)
        img = cv2.imread(f)
        if img is None:
            continue

        h,w = img.shape[:2]
        res = app(img[...,::-1])
        msk = (res['score'] > 0.5)
        print res['score']

        cls   = res['class'][msk]
        box   = res['box'][msk]
        score = res['score'][msk]

        for (box_, cls_, score_) in zip(box, cls, score):
            #ry0,rx0,ry1,rx1 = box_ # relative
            draw_bbox(img, box_, '{}:{:.2f}'.format(cls_, score_) )

        cv2.imshow('win', img)
        k = cv2.waitKey(0)
        if k in [27, ord('q')]:
            break

    #cv2.destroyWindow('win')
    cv2.destroyAllWindows()

def test_camera():
    """ Simple test srcipt; requires /dev/video0 """
    app = ObjectDetectorTF(use_gpu=False, cmap={1:'person'})

    cam = cv2.VideoCapture(0)

    fps = []

    while True:
        ret, img = cam.read()
        if not ret:
            print('camera capture failed')
            break
        h,w = img.shape[:2]
        t0 = time.time()
        res = app(img)
        t1 = time.time()
        fps.append(1.0 / (t1-t0+1e-9))
        msk = (res['score'] > 0.5)

        cls   = res['class'][msk]
        box   = res['box'][msk]
        score = res['score'][msk]
        for box_, cls_ in zip(box, cls):
            #ry0,rx0,ry1,rx1 = box_ # relative
            draw_bbox(img, box_, str(cls_))
        cv2.imshow('win', img)
        k = cv2.waitKey(1)
        if k in [ord('q'), 27]:
            print('quitting...')
            break
        print('average fps: {}'.format( np.mean(fps[-100:])) )

def main():
    #test_image()
    test_camera()
    #test_images()

if __name__ == "__main__":
    main()
