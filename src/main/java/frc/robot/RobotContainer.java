// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SwerveSubsystems.*;
import frc.robot.Commands.*;
import frc.robot.Information.*;

public class RobotContainer {
  // Autonomous chooser
  private final SendableChooser<Command> autoChooser;

  // Controllers
  XboxController xbox = new XboxController(0);
  Joystick joystick = new Joystick(1);

  // Subsystems
  DriveSubsystem driveSub = new DriveSubsystem();
  OdometrySubsystem odomSub = new OdometrySubsystem(driveSub);
  VisionSubsystem visionSub = new VisionSubsystem(this::acceptEstimatedRobotPose);

  // Commands from files
  XBOXDriveCommand driveCommand = new XBOXDriveCommand(driveSub, xbox, odomSub);
  // JoystickDriveCommand jdriveCommand = new JoystickDriveCommand(driveSub, joystick, odomSub);

  public RobotContainer() {
    // Initialize AutoBuilder after subsystems are created
    driveSub.initAutoBuilder(odomSub);
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    
    // Command scheduler
    driveSub.setDefaultCommand(driveCommand);
  }

  void acceptEstimatedRobotPose(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs) {
    odomSub.addVisionMeasurement(pose, timestamp, estimationStdDevs);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
