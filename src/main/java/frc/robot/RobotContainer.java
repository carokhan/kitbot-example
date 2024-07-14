// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.climber.Climber;
import frc.robot.climber.ClimberIO;
import frc.robot.climber.ClimberIOSim;
import frc.robot.climber.ClimberIOSparkMax;
import frc.robot.drive.Drive;
import frc.robot.drive.DriveIO;
import frc.robot.drive.DriveIOSim;
import frc.robot.drive.DriveIOSparkMax;

public class RobotContainer {
  private final Drive drive;
  private final Climber climber;
  private final CommandXboxController controller = new CommandXboxController(0);

  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        drive = new Drive(new DriveIOSparkMax());
        climber = new Climber(new ClimberIOSparkMax());
        break;
      case SIM:
        drive = new Drive(new DriveIOSim());
        climber = new Climber(new ClimberIOSim());
        break;
      default:
        drive = new Drive(new DriveIO() {});
        climber = new Climber(new ClimberIO() {});
        break;
    }
    configureBindings();
  }

  private void configureBindings() {
    drive.setDefaultCommand(
      drive.driveCommand(
        () -> modifyJoystick(controller.getLeftY()),
        () -> modifyJoystick(controller.getRightX()),
        () -> DriveConstants.isClosedLoop));

    controller.start().whileTrue(climber.runCurrentHoming());
    controller
      .leftBumper()
      .onTrue(
        climber.setExtensionCmd(() -> ClimbConstants.maxHeight));
    controller
      .leftTrigger(0.1)
      .onTrue(
        climber.setExtensionCmd(() -> ClimbConstants.minHeight));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public double modifyJoystick(double in) {
    if (Math.abs(in) < 0.05) {
      return 0;
    }
    return in * in * Math.signum(in);
  }
}
