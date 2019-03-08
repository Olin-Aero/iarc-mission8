import rospy
import tf
import numpy as np

def main():
    rospy.init_node('noisy_tf')
    #tfl = tf.TransformListener()
    tfb = tf.TransformBroadcaster()
    rate = rospy.Rate(50)
    T = np.eye(4)
    while not rospy.is_shutdown():
        dT = tf.transformations.compose_matrix(
            translate = np.random.normal(scale=0.001, size=3),
            angles = np.random.normal(scale=0.001, size=3)
            )
        T = dT.dot(T)
        txn = tf.transformations.translation_from_matrix(T)
        rxn = tf.transformations.quaternion_from_matrix(T)

        tfb.sendTransform(txn, rxn, rospy.Time.now(), 'odom', 'noise')
        rate.sleep()

if __name__ == "__main__":
    main()
