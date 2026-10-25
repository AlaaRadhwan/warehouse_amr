# B -> A navigation analysis: 20261022_130824_after_calibration_repeat3

Result: **STATUS_SUCCEEDED** after 60.6 sim-s (105 wall-s); watchdog abort: None

```json
{
 "result": {
  "status": "STATUS_SUCCEEDED",
  "abort_reason": null,
  "sim_duration_s": 60.57150000000001,
  "wall_duration_s": 105.08980178833008,
  "min_dist_to_goal_xy": 0.052401400346046005,
  "error_code": 0,
  "error_msg": ""
 },
 "frame_check": {
  "nav2_estimate_minus_ground_truth_xy": {
   "mean_m": 0.03361918382333446,
   "max_m": 0.08689750828098645,
   "final_m": 0.012182627467347738
  },
  "feedback_xy": [
   0.1333294020838114,
   -0.21900544913395859
  ],
  "gt_xy": [
   0.11348103332786472,
   -0.2300418251763038
  ],
  "feedback_yaw": 0.02901212948802054,
  "gt_yaw": 0.010286453724349887
 },
 "first_within_1m_s_after_goal": 52.49949999999999,
 "near_goal": {
  "time_in_zone_s": 11.995500000000007,
  "max_abs_gt_wz": 0.37721260905135506,
  "time_abs_wz_gt_0p5_s": 0.0,
  "total_rotation_deg": 76.5190604558223,
  "signed_net_rotation_deg": -36.06450881524528
 },
 "near_goal_cmd_vx_sign_changes": 0,
 "near_goal_cmd_vx_min_max": [
  -0.33308905363082886,
  0.004696176387369633
 ],
 "near_goal_cmd_wz_min_max": [
  -0.37961888313293457,
  0.2722020149230957
 ],
 "near_goal_cmd_wz_mean_abs": 0.1483374573715582,
 "final_gt": {
  "x": 0.11348103345447677,
  "y": -0.23004182504823575,
  "dist_xy": 0.05691487357932256,
  "yaw_err_deg": 0.4542084819644329,
  "v": 0.0,
  "wz": 0.0
 },
 "checker_conditions_first_met_s_after_goal": 60.398749999999986,
 "checker_conditions_met_fraction_after_first": 1.0,
 "at_result": {
  "gt_v": 0.001734832269164521,
  "gt_wz": -0.05512631619749416,
  "dist_xy": 0.05691487367096584,
  "yaw_err_deg": 0.5826233381451875
 },
 "yaw_rate_ratio_to_gt_when_|wz|>0.2": {
  "raw_wheel_odom": 1.0214792651719937,
  "fused_odom": 1.0144228891268605,
  "imu": 1.0091286935658805,
  "cmd_vel": 1.0214173272848561,
  "cmd_vel_nav": 1.0206415521384853
 },
 "cmd_wz_abs_max": {
  "cmd_vel_nav": 0.42996203899383545,
  "cmd_vel_smoothed": 0.42996203899383545,
  "cmd_vel": 0.42996203899383545,
  "controller_out": 0.42996203899383545
 },
 "plans": {
  "/unsmoothed_plan": {
   "count": 102,
   "first": {
    "t": 0.009000000000000341,
    "n": 187,
    "length": 9.595800138305052,
    "cusps": 2,
    "rot_only": 16,
    "reverse_frac": 0.23529411764705882,
    "end": [
     0.09899944151936779,
     -0.17500077104568845
    ],
    "end_yaw": 0.0
   },
   "last": {
    "t": 60.154500000000006,
    "n": 7,
    "length": 0.3607934750132411,
    "cusps": 2,
    "rot_only": 0,
    "reverse_frac": 0.6666666666666666,
    "end": [
     0.09899944151936779,
     -0.17500077104568845
    ],
    "end_yaw": 0.0
   },
   "max_length": 10.007023033688274,
   "max_cusps": 3,
   "msgs_with_cusp": 100,
   "msgs_with_reverse": 102,
   "max_rot_only_poses": 20,
   "single_pose_msgs": 0
  },
  "/plan": {
   "count": 102,
   "first": {
    "t": 0.009499999999995623,
    "n": 187,
    "length": 9.648005575921875,
    "cusps": 2,
    "rot_only": 14,
    "reverse_frac": 0.23837209302325582,
    "end": [
     0.09899944151936779,
     -0.17500077104568845
    ],
    "end_yaw": 0.0
   },
   "last": {
    "t": 60.154500000000006,
    "n": 7,
    "length": 0.3607934750132411,
    "cusps": 2,
    "rot_only": 0,
    "reverse_frac": 0.6666666666666666,
    "end": [
     0.09899944151936779,
     -0.17500077104568845
    ],
    "end_yaw": 0.0
   },
   "max_length": 9.995743930885192,
   "max_cusps": 3,
   "msgs_with_cusp": 100,
   "msgs_with_reverse": 102,
   "max_rot_only_poses": 19,
   "single_pose_msgs": 0
  },
  "/transformed_global_plan": {
   "count": 2101,
   "first": {
    "t": 0.021999999999998465,
    "n": 58,
    "length": 2.4559469434698458,
    "cusps": 1,
    "rot_only": 7,
    "reverse_frac": 0.26,
    "end": [
     14.090078889077773,
     1.311177670150434
    ],
    "end_yaw": -2.862234594888928
   },
   "last": {
    "t": 60.5405,
    "n": 1,
    "length": 0.0,
    "cusps": 0,
    "rot_only": 0,
    "reverse_frac": 0.0,
    "end": [
     7.687435723810259,
     -1.4448040797478696
    ],
    "end_yaw": -0.016134796571897397
   },
   "max_length": 2.4999736062973716,
   "max_cusps": 1,
   "msgs_with_cusp": 293,
   "msgs_with_reverse": 592,
   "max_rot_only_poses": 9,
   "single_pose_msgs": 60
  }
 },
 "transformed_plan": {
  "frame": "odom (goal mapped via end-of-run /odom minus feedback offset)",
  "end_within_015m_frac_all": 0.18372203712517848,
  "end_within_015m_frac_in_1m_zone": 0.8168498168498168,
  "end_dist_to_goal_max_in_zone": 0.9637375206767934,
  "single_pose_frac_in_zone": 0.21978021978021978,
  "msgs_with_cusp_in_zone": 0,
  "msgs_in_zone": 273
 },
 "optimal_trajectory_pred_yaw_change_deg": {
  "n": 2101,
  "mean_abs": 26.657896216116274,
  "last10_mean_abs": 4.23859005069207
 },
 "feedback_trace_samples": 61
}
```