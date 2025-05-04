// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;

/** Add your docs here. */
public class Camera {
    private String name;
    private Pose3d robotToCamera;
    private PhotonCamera camera;

    public Camera(String name, Pose3d robotToCamera) {
        this.name = name;
        this.robotToCamera = robotToCamera;
        this.camera = new PhotonCamera(name);
    }

    /**
     * Returns the best result seen by this camera
     * @return PhotonTrackedTarget representing the best target
     */
    public Optional<PhotonTrackedTarget> getBestResult() {
        var results = camera.getLatestResult();
        if (results.hasTargets() == false) {
            return Optional.empty();
        }

        PhotonTrackedTarget target = results.getBestTarget();
        return Optional.of(target);
    }

    /**
     * Returns the estimated global pose that this camera sees (if any)
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        var results = camera.getLatestResult();
        if (results.hasTargets() == false) {
            return Optional.empty();
        }

        PhotonTrackedTarget target = results.getBestTarget();
        var cameraToTarget = target.getBestCameraToTarget();
        Pose3d robotToTarget = robotToCamera.transformBy(cameraToTarget.inverse());
        return Optional.empty();
    }

}