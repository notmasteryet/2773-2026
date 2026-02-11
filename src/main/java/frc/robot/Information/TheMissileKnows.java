// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Information;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveSubsystems.DriveSubsystem;

public class TheMissileKnows extends SubsystemBase {
  private final DriveSubsystem drive;
  private final PhotonCamera camera;
  private final PhotonPoseEstimator photonEstimator;
  private final SwerveDrivePoseEstimator poseEstimator;
  private final SwerveDriveKinematics kinematics;
  private final AHRS gyro;

  private double lastVisionTimestamp = 0D;

  public TheMissileKnows(DriveSubsystem drive) {
    this.drive = drive;
    this.camera = new PhotonCamera(Constants.CameraName);
    this.photonEstimator = new PhotonPoseEstimator(
        Constants.TagLayout, 
        Constants.RobotToCam);
    this.kinematics = new SwerveDriveKinematics(
        Constants.kfrontLeftLocation, 
        Constants.kfrontRightLocation, 
        Constants.kbackLeftLocation, 
        Constants.kbackRightLocation);
    this.gyro = new AHRS(NavXComType.kMXP_SPI);

    this.poseEstimator = new SwerveDrivePoseEstimator(
        kinematics, 
        gyro.getRotation2d(), 
        drive.getPositions(), 
        new Pose2d());
  }

  private Matrix<N3, N1> calculateStdDevs(EstimatedRobotPose estimate) {
    // start with confident estimate
    var stdDevs = Constants.SingleTagStdDevs;

    // get first target
    var target = estimate.targetsUsed.get(0);

    // calculate 3d distance to tag
    double distance = target.getBestCameraToTarget().getTranslation().getNorm();
  }

  private void masterUpdate() {
    // update chassis odometry first
    poseEstimator.update(gyro.getRotation2d(), drive.getPositions());

    // process vision
    var results = camera.getAllUnreadResults();
    for (var result : results) {
      var optEstimate = photonEstimator.estimateLowestAmbiguityPose(result);
      if (optEstimate.isPresent()) {
        var est = optEstimate.get();
        double ambiguity = est.targetsUsed.get(0).getPoseAmbiguity();

        // filter results
        if (ambiguity < 0.2 && est.timestampSeconds > lastVisionTimestamp) {
          lastVisionTimestamp = est.timestampSeconds;
          poseEstimator.addVisionMeasurement(
            est.estimatedPose.toPose2d(),
            est.timestampSeconds,
            calculateStdDevs(est));
        }
      }
    }
  }

  @Override
  public void periodic() {
    masterUpdate();
  }
}
