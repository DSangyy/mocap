import time
from multiprocessing import Process, Queue
from pykinect2 import PyKinectV2, PyKinectRuntime


def get_joint_cord(joints, joint_id):
    joint_state = joints[joint_id].TrackingState

    # both joints are not tracked
    if joint_state == PyKinectV2.TrackingState_NotTracked:
        return None
    else:
        return joints[joint_id].Position.x, joints[joint_id].Position.y, joints[joint_id].Position.z


def get_root_pos(q: Queue):
    kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Body)
    bodies = None
    last_pelvis_pos = (0, 0, 0)
    while True:
        if kinect.has_new_body_frame():
            bodies = kinect.get_last_body_frame()

        body = None
        if bodies is not None:
            for i in range(kinect.max_body_count):
                if bodies.bodies[i].is_tracked:
                    body = bodies.bodies[i]
            if body is None:
                q.put((last_pelvis_pos, False))
            else:
                pelvis_pos = get_joint_cord(body.joints, PyKinectV2.JointType_SpineBase)
                if pelvis_pos is not None:
                    q.put((pelvis_pos, True))
                    last_pelvis_pos = pelvis_pos
                else:
                    q.put((last_pelvis_pos, False))

        time.sleep(1/60)


if __name__ == '__main__':
    q = Queue()
    p = Process(target=get_root_pos, args=(q,))
    p.daemon = True
    p.start()
    while True:
        print(q.get())


