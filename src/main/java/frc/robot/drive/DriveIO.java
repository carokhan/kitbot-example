// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.DriveConstants;

/** Add your docs here. */
public interface DriveIO {
    @AutoLog
    public static class DriveIOInputs {
        public boolean isClosedLoop = DriveConstants.isClosedLoop;

        public double leftRotationsRad = 0.0;
        public double leftPositionMeters = 0.0;
        public double leftVelocityRadPerSec = 0.0;
        public double leftVelocityMetersPerSecond = 0.0;
        public double leftVelocityGoalMetersPerSecond = 0.0;
        public double leftAppliedVolts = 0.0;
        public double[] leftCurrentAmps = new double[] {};
        public double[] leftTempCelsius = new double[] {};

        public double rightRotationsRad = 0.0;
        public double rightPositionMeters = 0.0;
        public double rightVelocityRadPerSec = 0.0;
        public double rightVelocityMetersPerSecond = 0.0;
        public double rightVelocityGoalMetersPerSecond = 0.0;
        public double rightAppliedVolts = 0.0;
        public double[] rightCurrentAmps = new double[] {};
        public double[] rightTempCelsius = new double[] {};

        public Rotation2d gyroYaw = new Rotation2d();
    }

    public default void updateInputs(DriveIOInputsAutoLogged inputs) {}

    public default void setVolts(double left, double right) {}

    public default void setMetersPerSecond(double left, double right) {}
    
} 
