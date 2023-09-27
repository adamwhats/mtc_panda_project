"""
Defines instances of tasks to be called by the task executor
"""
from mtc_panda_task_execution.task import *
from mtc_panda_task_execution.utils import (list_to_pose, list_to_pose_stamped,
                                            pose_to_homogenous,
                                            transform_pose_stamped)

# Poses
base_frame = 'panda_link0'
camera_offset = list_to_pose([-0.05, 0.0, -0.04])

pick_1 = list_to_pose_stamped(
    [0.411125, 0.182264, 0.041517, 0.922417, -0.385711, 0.018938, 0.003979], base_frame
)
pick_1_camera = transform_pose_stamped(pick_1, pose_to_homogenous(camera_offset), use_pose_frame=True)

place_1 = list_to_pose_stamped(
    [0.330657, 0.424537, 0.049597, -0.401584, 0.9131, 0.069703, -0.011004], base_frame
)
place_2 = list_to_pose_stamped(
    [0.348017, 0.446285, 0.051058, -0.390882, 0.918758, 0.054858, -0.009253], base_frame
)
place_3 = list_to_pose_stamped( 
    [0.363967, 0.469183, 0.052478, -0.39975, 0.916312, 0.02324, 0.005696], base_frame
)
place_4 = list_to_pose_stamped(
    [0.39264, 0.504171, 0.053736, -0.389332, 0.921025, 0.011347, -0.002116], base_frame
)
place_5 = list_to_pose_stamped(
    [0.410397, 0.525826, 0.053225, -0.411566, 0.911341, 0.003617, -0.007689], base_frame
)

place_1_camera = transform_pose_stamped(place_1, pose_to_homogenous(camera_offset), use_pose_frame=True)
place_2_camera = transform_pose_stamped(place_2, pose_to_homogenous(camera_offset), use_pose_frame=True)
place_3_camera = transform_pose_stamped(place_3, pose_to_homogenous(camera_offset), use_pose_frame=True)
place_4_camera = transform_pose_stamped(place_4, pose_to_homogenous(camera_offset), use_pose_frame=True)
place_5_camera = transform_pose_stamped(place_5, pose_to_homogenous(camera_offset), use_pose_frame=True)


# Joint targets
prepick_1_joint = joint_target(0.872882, 0.129, -0.433331, -2.59721, 0.067343, 2.734703, -0.397258)
preplace_3_joint = joint_target(0.910078, 0.334544, -0.044302, -2.165173, 0.012762, 2.424269, 1.619139)

# Steps
move_ready = move_configuration('ready', 'ompl_rrtc')
move_pick_camera = move_pose(pick_1_camera, 'ompl_rrtc')
move_prepick_1_joint = move_joint(prepick_1_joint, 'ompl_rrtc')
move_pick_1 = move_pose(pick_1, 'pilz_lin', 0.01, 0.05)
move_place_3_camera = move_pose(place_3_camera, 'ompl_rrtc')
move_preplace_3_joint = move_joint(preplace_3_joint, 'ompl_rrtc')
move_place_3 = move_pose(place_3, 'pilz_lin', 0.01, 0.05)

move_place_1_camera = move_pose(place_1_camera, 'ompl_rrtc')
move_place_2_camera = move_pose(place_2_camera, 'ompl_rrtc')
move_place_4_camera = move_pose(place_4_camera, 'ompl_rrtc')
move_place_5_camera = move_pose(place_5_camera, 'ompl_rrtc')

pick = modbus_task(c5=True)
place = modbus_task()


# Tasks
demo_task = [
    move_ready, move_pick_camera, move_prepick_1_joint, move_pick_1, pick,
    move_prepick_1_joint, move_ready, move_prepick_1_joint, move_pick_1, place, move_prepick_1_joint,
    move_ready, move_preplace_3_joint, pick, move_place_3, move_preplace_3_joint,
    move_place_3_camera, move_ready, move_preplace_3_joint, move_place_3, place, move_preplace_3_joint
]

if __name__ == '__main__':
    pass
