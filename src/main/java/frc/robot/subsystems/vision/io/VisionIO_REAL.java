// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision.io;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionConstants.GeneralConstants;

/** Add your docs here. */
public class VisionIO_REAL implements VisionIO {
    PhotonCamera[] cameras;
    PhotonPoseEstimator[] poseEstimators;
    public VisionIO_REAL() {
        cameras = new PhotonCamera[GeneralConstants.CameraIDs.length];
        poseEstimators = new PhotonPoseEstimator[cameras.length];
        for (int i=0; i < cameras.length - 1; i++) {
            cameras[i] = new PhotonCamera(GeneralConstants.CameraIDs[i]);
            poseEstimators[i] = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(GeneralConstants.field), GeneralConstants.strategy, GeneralConstants.CameraTransforms[i]);
        }
    }

    @Override
    public Pose2d[] getMeasurements() {
        Pose2d[] visionMeasurement = new Pose2d[cameras.length];
        for (int i=0; i < cameras.length; i++) {
            List<PhotonPipelineResult> pipelineResults = cameras[i].getAllUnreadResults();
            if (pipelineResults.size() > 0) {
                // may be last elem
                Optional<EstimatedRobotPose> estimatedPose = poseEstimators[i].update(pipelineResults.get(0));
                if (estimatedPose.isPresent()) {
                    visionMeasurement[i] = estimatedPose.get().estimatedPose.toPose2d();
                }
            }
        }
        return visionMeasurement;
    }

    @Override
    public void update(Pose2d pose) {
        
    }

}
