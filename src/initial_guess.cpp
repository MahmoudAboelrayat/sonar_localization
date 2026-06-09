/**
 * computeInitialGuess()
 *
 * Priority order:
 *   1. DVL + IMU  — best: gyro rotation + DVL accumulated displacement (single integration)
 *   2. IMU only   — full preintegration with bias correction (double integration)
 *   3. DVL only   — accumulated body-frame displacement, rotation frozen
 *   4. Constant velocity — project last scan-to-scan delta forward
 *   5. Identity   — first scan, no prior information
 *
 * Coordinate conventions:
 *   - current_global  : T_{k-1} in world frame (4x4 SE3, row-major Eigen float)
 *   - prev_scan_pose_ : T_{k-2} in world frame, updated AFTER this function returns
 *   - scan_dvl_dp_    : accumulated body-frame displacement integrated over the scan interval
 *   - prev_velocity_  : velocity in WORLD frame (required by gtsam::NavState)
 */
Eigen::Matrix4f computeInitialGuess()
{
    Eigen::Matrix4f initial_guess = current_global;  // safe default

    // -------------------------------------------------------------------------
    // Case 1: DVL + IMU
    //   R_guess = R_last * Exp(ω * dt)           ← gyro from IMU preintegration
    //   p_guess = p_last + R_last * Δp_dvl        ← accumulated DVL body-frame displacement
    //
    //   Using scan_dvl_dp_ (accumulated over scan interval) is more accurate
    //   than the snapshot v_dvl * dt, which assumes constant velocity.
    // -------------------------------------------------------------------------
    if (use_dvl_ && use_imu_ && scan_preint_)
    {
        gtsam::Rot3 delta_R;
        {
            std::lock_guard<std::mutex> ilk(imu_mutex_);
            delta_R = scan_preint_->deltaRij();
            // dt_scan not needed here — displacement already integrated in scan_dvl_dp_
        }

        Eigen::Vector3f dvl_dp = Eigen::Vector3f::Zero();
        bool dvl_ok = false;
        {
            std::lock_guard<std::mutex> dlk(dvl_mutex_);
            dvl_ok = scan_dvl_valid_;
            dvl_dp = scan_dvl_dp_;  // accumulated body-frame displacement over scan interval
        }

        // Build body-frame delta transform:
        //   scan_delta = [ΔR | Δp_body]
        //                [ 0 |    1   ]
        // Then: T_guess = T_{k-1} * scan_delta
        //   → rotation composed in world frame
        //   → body displacement rotated to world via T_{k-1}'s rotation block
        Eigen::Matrix4f scan_delta = Eigen::Matrix4f::Identity();
        scan_delta.block<3, 3>(0, 0) = delta_R.matrix().cast<float>();
        if (dvl_ok) {
            scan_delta.block<3, 1>(0, 3) = dvl_dp;  // body-frame; world rotation applied by ⊕
        }

        initial_guess = current_global * scan_delta;
    }

    // -------------------------------------------------------------------------
    // Case 2: IMU only (no DVL)
    //   Full preintegration: gravity + double-integrated accel + gyro rotation
    //   p_guess = p_last + v_last * dt + ΔP_imu
    //   R_guess = R_last * ΔR_imu
    //
    //   NOTE: prev_velocity_ must be in WORLD frame, not body frame.
    //         gtsam::NavState expects world-frame velocity.
    // -------------------------------------------------------------------------
    else if (use_imu_ && scan_preint_)
    {
        gtsam::NavState prop;
        {
            std::lock_guard<std::mutex> ilk(imu_mutex_);
            prop = scan_preint_->predict(
                gtsam::NavState(matrix2Pose3(current_global), prev_velocity_),  // prev_velocity_ in world frame
                prev_bias_);
        }
        initial_guess = pose32Matrix(prop.pose());
    }

    // -------------------------------------------------------------------------
    // Case 3: DVL only, no IMU
    //   Translation: accumulated body-frame DVL displacement rotated to world
    //   Rotation:    frozen at current_global's rotation (DVL cannot measure rotation)
    //
    //   Yaw will not update in this mode — acceptable for slow yaw-rate platforms
    //   but will accumulate heading error over time without a separate compass/gyro.
    // -------------------------------------------------------------------------
    else if (use_dvl_)
    {
        Eigen::Vector3f dvl_dp = Eigen::Vector3f::Zero();
        bool dvl_ok = false;
        {
            std::lock_guard<std::mutex> dlk(dvl_mutex_);
            dvl_ok    = scan_dvl_valid_;
            dvl_dp    = scan_dvl_dp_;
        }

        initial_guess = current_global;  // rotation unchanged — no gyro available
        if (dvl_ok) {
            // Rotate body-frame displacement into world frame using current rotation
            initial_guess.block<3, 1>(0, 3) +=
                current_global.block<3, 3>(0, 0) * dvl_dp;
        }
        // Note: no rotation update — yaw/pitch/roll frozen at current_global values
    }

    // -------------------------------------------------------------------------
    // Case 4: No IMU, no DVL — constant velocity model
    //   ΔT_{k-2 → k-1} = T_{k-2}^{-1} * T_{k-1}
    //   T_guess         = T_{k-1}  *  ΔT_{k-2 → k-1}   (project same motion forward)
    //
    //   IMPORTANT: prev_scan_pose_ must be T_{k-2} here.
    //   Update prev_scan_pose_ = current_global AFTER this block, not before.
    // -------------------------------------------------------------------------
    else if (has_prev_scan_)
    {
        // delta: relative motion from k-2 to k-1 expressed in frame k-2
        Eigen::Matrix4f delta = prev_scan_pose_.inverse() * current_global;

        // project forward: assume robot continues with the same relative motion
        initial_guess = current_global * delta;
    }

    // -------------------------------------------------------------------------
    // Case 5: First scan — no prior information
    //   Use current_global as-is (set to Identity or initial prior elsewhere)
    // -------------------------------------------------------------------------
    else
    {
        initial_guess = current_global;
    }

    // -------------------------------------------------------------------------
    // Update previous scan pose AFTER computing the guess.
    // If updated before, prev_scan_pose_ == current_global and delta == Identity.
    // -------------------------------------------------------------------------
    prev_scan_pose_ = current_global;
    has_prev_scan_  = true;

    return initial_guess;
}
