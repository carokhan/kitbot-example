// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Drive extends SubsystemBase {
  private final DriveIO io;
  private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();

  private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(new Rotation2d(), 0, 0);
  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(DriveConstants.trackWidth);
  
  private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(DriveConstants.kS, DriveConstants.kV);
  /** Creates a new DriveSubsystem. */
  public Drive(DriveIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive", inputs);
    odometry.update(inputs.gyroYaw, inputs.leftPositionMeters, inputs.rightPositionMeters);
    Logger.recordOutput("Drivebase Pose", odometry.getPoseMeters());
  }

  public void drive(double speed, double angle, boolean isClosedLoop) {
    var wheelSpeeds = DifferentialDrive.arcadeDriveIK(speed, angle, false);
    if (isClosedLoop) {
      io.setMetersPerSecond(wheelSpeeds.left * 12.0, wheelSpeeds.right * 12.0);
    } else {
      io.setVolts(wheelSpeeds.left * 12.0, wheelSpeeds.right * 12.0);
    } 
  }

  public Command driveCommand(DoubleSupplier speed, DoubleSupplier angle, BooleanSupplier isClosedLoop) {
    return new RunCommand(() -> drive(speed.getAsDouble(), angle.getAsDouble(), isClosedLoop.getAsBoolean()), this);
  }
}
