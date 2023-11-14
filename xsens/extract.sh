mkdir -p extracted
for i in *.bin
do
  echo $i
  mc_bin_utils extract --in "$i" --out "extracted/$i" --keys \
    body6d_hrp4_L_ELBOW_P_LINK_target body6d_hrp4_L_HIP_P_LINK_target body6d_hrp4_L_KNEE_P_LINK_target \
    body6d_hrp4_L_SHOULDER_R_LINK_target body6d_hrp4_L_SHOULDER_Y_LINK_target body6d_hrp4_NECK_P_LINK_target \
    body6d_hrp4_R_ELBOW_P_LINK_target body6d_hrp4_R_HIP_P_LINK_target body6d_hrp4_R_KNEE_P_LINK_target \
    body6d_hrp4_R_SHOULDER_R_LINK_target body6d_hrp4_R_SHOULDER_Y_LINK_target body6d_hrp4_body_target \
    body6d_hrp4_l_ankle_target body6d_hrp4_l_wrist_target body6d_hrp4_r_ankle_target body6d_hrp4_r_wrist_target body6d_hrp4_torso_target
done
