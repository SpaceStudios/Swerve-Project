// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision.io;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionConstants.GeneralConstants;

/** Add your docs here. */
public class VisionIO_SIM implements VisionIO {
    PhotonCamera[] cameras;
    PhotonCameraSim[] simCameras;
    VisionSystemSim visionSim;
    PhotonPoseEstimator[] poseEstimators;

    public VisionIO_SIM() {
        SimCameraProperties camProperties = new SimCameraProperties();
        camProperties.setCalibration(320, 320, Rotation2d.fromDegrees(70));
        // camProperties.setCalibError(0.25, 0.08);
        // camProperties.setFPS(90);
        // camProperties.setAvgLatencyMs(35);
        // camProperties.setLatencyStdDevMs(5);
        cameras = new PhotonCamera[GeneralConstants.CameraIDs.length];
        simCameras = new PhotonCameraSim[GeneralConstants.CameraIDs.length];
        poseEstimators = new PhotonPoseEstimator[cameras.length];
        visionSim = new VisionSystemSim("visionSim");
        visionSim.addAprilTags(AprilTagFieldLayout.loadField(GeneralConstants.field));
        for (int i=0; i<cameras.length; i++) {
            cameras[i] = new PhotonCamera(GeneralConstants.CameraIDs[i]);
            simCameras[i] = new PhotonCameraSim(cameras[i], camProperties);
            poseEstimators[i] = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(GeneralConstants.field), GeneralConstants.strategy, GeneralConstants.CameraTransforms[i]);
            visionSim.addCamera(simCameras[i], poseEstimators[i].getRobotToCameraTransform());
        }
    }

    @Override
    public Pose2d[] getMeasurements() {
        Pose2d[] visionMeasurement = new Pose2d[simCameras.length];
        for (int i=0; i<simCameras.length; i++) {
            List<PhotonPipelineResult> pipelineResult = simCameras[i].getCamera().getAllUnreadResults();
            if (pipelineResult.size() > 0) {
                PhotonPipelineResult result = pipelineResult.get(0);
                Optional<EstimatedRobotPose> pose = poseEstimators[i].update(result);
                if (pose.isPresent()) {
                    visionMeasurement[i] = pose.get().estimatedPose.toPose2d();
                }
            }
        }
        return visionMeasurement;
    }

	@Override
	public void update(Pose2d pose) {
		visionSim.update(pose);
        for (int i=0; i<poseEstimators.length; i++) {
            poseEstimators[i].setReferencePose(pose);
        }
	}
}
