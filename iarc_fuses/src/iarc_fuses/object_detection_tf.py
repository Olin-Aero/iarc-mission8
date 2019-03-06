import numpy as np
import os
import six.moves.urllib as urllib
import tarfile
import tensorflow as tf
import cv2
import time

class ObjectDetectorTFSSD(object):
    def __init__(self,
            root='/tmp',
            #model='ssd_mobilenet_v1_coco_11_06_2017',
            model='ssd_mobilenet_v1_ppn_shared_box_predictor_300x300_coco14_sync_2018_07_03',
            use_gpu=False
            ):
        # cache arguments
        self.root_  = root
        self.model_ = model
        self.use_gpu_ = use_gpu

        # load model
        ckpt_file = self._maybe_download_ckpt()
        self.graph_ = self._load_graph(ckpt_file)
        self.input_, self.output_ = self._build_pipeline(self.graph_)

        label_file = os.path.join('data', 'mscoco_label_map.pbtxt')
        n_classes = 90

    def __call__(self, img):
        if img.ndim == 3:
            # single image configuration
            outputs = self.__call__(img[None,...])
            return {k:v[0] for k,v in outputs.iteritems()}
        outputs = self.run(self.output_, {self.input_:img})
        return outputs

    def _maybe_download_ckpt(self):
        ckpt_file = os.path.join(self.root_, self.model_,
                'frozen_inference_graph.pb')
        model_tar_basename = '{}.tar.gz'.format(self.model_)
        model_tar_fullpath = os.path.join(self.root_, model_tar_basename)

        if not os.path.exists(ckpt_file):
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
        return ckpt_file

    def _load_graph(self, ckpt_file):
        detection_graph = tf.Graph()
        with detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(ckpt_file, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
        return detection_graph

    def _build_pipeline(self, graph):
        x = None
        y = {}
        with graph.as_default():
            get = lambda s : graph.get_tensor_by_name(s)
            x = get('image_tensor:0')
            y['box'] = get('detection_boxes:0')
            y['score'] = get('detection_scores:0')
            y['class'] = get('detection_classes:0')
            y['num'] = get('num_detections:0')
        return x, y

    def warmup(self):
        self.__call__(np.zeros(shape=(1,480,640,3), dtype=np.uint8))

    def initialize(self):
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
    app = ObjectDetectorTFSSD()
    app.initialize()

    img = cv2.imread('/tmp/image1.jpg')
    h,w = img.shape[:2]
    res = app(img)
    msk = (res['score'] > 0.5)

    cls   = res['class'][msk]
    box   = res['box'][msk]
    score = res['score'][msk]

    for box_ in box:
        #ry0,rx0,ry1,rx1 = box_ # relative
        yxyx = box_
        yxyx = np.multiply(yxyx, [h,w,h,w])
        yxyx = np.round(yxyx).astype(np.int32)
        y0,x0,y1,x1 = yxyx
        cv2.rectangle(img, (x0,y0), (x1,y1), (255,0,0), thickness=2)

    cv2.imshow('win', img)
    cv2.waitKey(0)

def test_camera():
    app = ObjectDetectorTFSSD(use_gpu=False)
    app.initialize()

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
        for box_ in box:
            #ry0,rx0,ry1,rx1 = box_ # relative
            yxyx = box_
            yxyx = np.multiply(yxyx, [h,w,h,w])
            yxyx = np.round(yxyx).astype(np.int32)
            y0,x0,y1,x1 = yxyx
            cv2.rectangle(img, (x0,y0), (x1,y1), (255,0,0), thickness=2)
        cv2.imshow('win', img)
        k = cv2.waitKey(1)
        if k in [ord('q'), 27]:
            print('quitting...')
            break
        print('average fps : {}'.format( np.mean(fps[-100:])) )

def main():
    test_camera()

if __name__ == "__main__":
    main()
