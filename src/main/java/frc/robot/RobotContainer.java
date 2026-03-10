// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.SwerveSubsystems.*;
import frc.robot.Commands.*;
import frc.robot.Information.*;

public class RobotContainer {
  public RobotContainer() {

  }

  // Base inits
  // Controllers
  XboxController xbox = new XboxController(0);
  Joystick joystick = new Joystick(1);

  // Subsystems
  DriveSubsystem driveSub = new DriveSubsystem();
  PhotonSubsystem photonSub;// = new PhotonSubsystem();
  VisionSubsystem visionSub = new VisionSubsystem(this::acceptEstimatedRobotPose);
  OdometrySubsystem odomSub = new OdometrySubsystem(driveSub, photonSub);

  // Commands from files
  XBOXDriveCommand driveCommand = new XBOXDriveCommand(driveSub, xbox, odomSub);
  // JoystickDriveCommand jdriveCommand = new JoystickDriveCommand(driveSub, joystick, odomSub);

  // Command scheduler
  {
    driveSub.setDefaultCommand(driveCommand);
    
  }

  void acceptEstimatedRobotPose(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs) {
    System.out.println(pose);
  }
  // Autonomous chooser

  // return new DeltaPoseCommand(0, 1.5, 0, driveSub, odomSub);
}
