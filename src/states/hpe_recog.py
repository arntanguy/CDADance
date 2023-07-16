import sys 
# print(f'{sys.path}')

import numpy as np
from scipy.spatial import distance
# import cv2
import pdb
import mediapipe as mp 
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from mediapipe import solutions
from mediapipe.framework.formats import landmark_pb2

# ============================================== #
# ====== correspond landmark in mediapipe ====== #
# 0 - nose
# 1 - left eye (inner)
# 2 - left eye
# 3 - left eye (outer)
# 4 - right eye (inner)
# 5 - right eye
# 6 - right eye (outer)
# 7 - left ear
# 8 - right ear
# 9 - mouth (left)
# 10 - mouth (right)
# 11 - left shoulder
# 12 - right shoulder
# 13 - left elbow
# 14 - right elbow
# 15 - left wrist
# 16 - right wrist
# 17 - left pinky
# 18 - right pinky
# 19 - left index
# 20 - right index
# 21 - left thumb
# 22 - right thumb
# 23 - left hip
# 24 - right hip
# 25 - left knee
# 26 - right knee
# 27 - left ankle
# 28 - right ankle
# 29 - left heel
# 30 - right heel
# 31 - left foot index
# 32 - right foot index
# ============================================== #


model_path = '/home/dell/workspace/HPE/pose_landmarker_lite.task'
# image_path = '/home/dell/workspace/HPE/img/girl-4051811_960_720.jpg'
image_path = '/home/dell/workspace/HPE/img/raisehand.jpg'
# image_path = '/home/dell/workspace/HPE/img/sayhi.jpg'
# image_path = '/home/dell/workspace/HPE/img/squat.jpg'

BUFFER_LEN = 0.4
r_mouth_idx = 10
l_shoulder_idx = 11
r_shoulder_idx = 12
l_elbow_idx = 13
r_elbow_idx = 14
l_wrist_idx = 15
r_wrist_idx = 16
l_thumb_idx = 21
r_thumb_idx = 22

debug_ = False


def draw_landmarks_on_image(rgb_image, detection_result):
  pose_landmarks_list = detection_result.pose_landmarks
  annotated_image = np.copy(rgb_image)

  # Loop through the detected poses to visualize.
  for idx in range(len(pose_landmarks_list)):
    pose_landmarks = pose_landmarks_list[idx]

    # Draw the pose landmarks.
    pose_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
    pose_landmarks_proto.landmark.extend([
      landmark_pb2.NormalizedLandmark(x=landmark.x, y=landmark.y, z=landmark.z) for landmark in pose_landmarks
    ])
    solutions.drawing_utils.draw_landmarks(
      annotated_image,
      pose_landmarks_proto,
      solutions.pose.POSE_CONNECTIONS,
      solutions.drawing_styles.get_default_pose_landmarks_style())

  return annotated_image

def pose_score(detection_result):
	pose_landmarks_list = detection_result.pose_landmarks

	# Loop through the detected poses to visualize.
	for idx in range(len(pose_landmarks_list)):
		pose_landmarks = pose_landmarks_list[idx]

		# extract necessary landmarks
		l_wrist_lm = pose_landmarks[l_wrist_idx] #15
		r_wrist_lm = pose_landmarks[r_wrist_idx] #16
		r_mouth_lm = pose_landmarks[r_mouth_idx] #10
		l_shoulder_lm = pose_landmarks[l_shoulder_idx] #11
		r_shoulder_lm = pose_landmarks[r_shoulder_idx] #12
		l_elbow_lm = pose_landmarks[l_elbow_idx] #13
		r_elbow_lm = pose_landmarks[r_elbow_idx] #14
		l_thumb_lm = pose_landmarks[l_thumb_idx] #21
		r_thumb_lm = pose_landmarks[r_thumb_idx] #22

		# condition: squat 
		score_squat = BUFFER_LEN - abs(l_wrist_lm.y - r_wrist_lm.y) 

		# condition: say hi 
		if(r_mouth_lm.y < r_wrist_lm.y < r_shoulder_lm.y):
			#say hi with left hand
			sayhiLR = 'R'
			score_sayhi = abs(r_wrist_lm.y - abs((r_mouth_lm.y - r_shoulder_lm.y)/2)) #more far away from the middle point of mouth-shoulder, more high possibility of saying hi
		else:
			sayhiLR = None
			score_sayhi = 0
		##[TODO]add another 2 conds for R-hand and both-hand


		# condition: raise hand 
		r_shoulder_lm_set = (r_shoulder_lm.x, r_shoulder_lm.y, 0)
		r_elbow_lm_set = (r_elbow_lm.x, r_elbow_lm.y, 0)
		dst = distance.euclidean(r_shoulder_lm_set, r_elbow_lm_set)
		score_raisehand_R = r_mouth_lm.y - (r_thumb_lm.y - dst/2)
		raisehandLR = 'R'
		score_raisehand = score_raisehand_R
		#[TODO]add another 1 conds for L-hand


	return score_squat, score_sayhi, score_raisehand, sayhiLR, raisehandLR

def human_pose_landmarks_estimation():

	print(f'[HPE-algo] test image path = {image_path} \n')
	print(f'[HPE-algo] model(lite) path =model_path \n')

	# Create an PoseLandmarker object.
	base_options = python.BaseOptions(model_asset_path=model_path)
	options = vision.PoseLandmarkerOptions(
		base_options=base_options,
		output_segmentation_masks=True)
	detector = vision.PoseLandmarker.create_from_options(options)

	image = mp.Image.create_from_file(image_path)

	detection_result = detector.detect(image)

	# detect posture and serve as valueIdx to decide the response movements of HPR4
	score_squat, score_sayhi, score_raisehand, sayhiLR, raisehandLR = pose_score(detection_result)
	print(f'[HPE-algo]---------- PostureCalculationScore ---------- \n score_squat = {score_squat}\n score_sayhi = {score_sayhi}\n score_raisehand = {score_raisehand}\n sayhiLR = {sayhiLR}; raisehandLR = {raisehandLR}\n')

	if(score_squat > score_sayhi and score_squat > score_raisehand):
		print(f'the HRP4 response in the sequence: SeqSF \n')
		return 0
	elif(score_sayhi > score_squat and score_sayhi > score_raisehand):
		print(f'the HRP4 response in the sequence: SeqAFL \n')
		return 1
	elif(score_raisehand > score_squat and score_raisehand > score_sayhi):
		print(f'the HRP4 response in the sequence: Seq3LookForApples \n')
		return 2
	else:
		return 3

	if(debug_):
		annotated_image = draw_landmarks_on_image(image.numpy_view(), detection_result)
		cv2.imshow('annotated_image', cv2.cvtColor(annotated_image, cv2.COLOR_RGB2BGR))

		key = cv2.waitKey(5000)
		if key == 27:#if ESC is pressed, exit loop
			cv2.destroyAllWindows()
	


# segmentation_mask = detection_result.segmentation_masks[0].numpy_view()
# visualized_mask = np.repeat(segmentation_mask[:, :, np.newaxis], 3, axis=2) * 255
# cv2.imshow('visualized_mask', visualized_mask)

# key = cv2.waitKey(5000)
# if key == 27:#if ESC is pressed, exit loop
# 	cv2.destroyAllWindows()



