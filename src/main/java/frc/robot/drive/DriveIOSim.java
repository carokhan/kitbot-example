// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;

import static frc.robot.Constants.DriveConstants;

/** Add your docs here. */
public class DriveIOSim implements DriveIO {
    private DifferentialDrivetrainSim sim = DifferentialDrivetrainSim.createKitbotSim(
        KitbotMotor.kDoubleNEOPerSide, 
        KitbotGearing.k8p45, 
        KitbotWheelSize.kSixInch, 
        null);

    private double leftAppliedVolts = 0.0;
    private double rightAppliedVolts = 0.0;

    private boolean isClosedLoop = false;
    private PIDController leftPID = new PIDController(DriveConstants.kPSim, DriveConstants.kISim, DriveConstants.kDSim);
    private PIDController rightPID = new PIDController(DriveConstants.kPSim, DriveConstants.kISim, DriveConstants.kDSim);

    @Override
    public void updateInputs(final DriveIOInputsAutoLogged inputs) {
        if (isClosedLoop) {
            leftAppliedVolts = MathUtil.clamp(leftPID.calculate(sim.getLeftVelocityMetersPerSecond() / DriveConstants.wheelRadius) + DriveConstants.kLeftFFVoltsSim, -12.0, 12.0);
            rightAppliedVolts = MathUtil.clamp(rightPID.calculate(sim.getRightVelocityMetersPerSecond() / DriveConstants.wheelRadius) + DriveConstants.kRightFFVoltsSim, -12.0, 12.0);
        }
        sim.update(.02);

        sim.setInputs(leftAppliedVolts, rightAppliedVolts);

        inputs.leftRotationsRad = sim.getLeftPositionMeters() / DriveConstants.wheelRadius;
        inputs.leftPositionMeters = sim.getLeftPositionMeters();
        inputs.leftVelocityRadPerSec = sim.getLeftVelocityMetersPerSecond() / DriveConstants.wheelRadius;
        inputs.leftVelocityMetersPerSecond = sim.getLeftVelocityMetersPerSecond();
        inputs.leftVelocityGoalMetersPerSecond = (leftPID.getSetpoint());
        inputs.leftAppliedVolts = leftAppliedVolts;
        inputs.leftCurrentAmps = new double[] {sim.getLeftCurrentDrawAmps()};
        // inputs.leftTempCelsius = new double[] {};

        inputs.rightRotationsRad = sim.getRightPositionMeters() / DriveConstants.wheelRadius;
        inputs.rightPositionMeters = sim.getRightPositionMeters();
        inputs.rightVelocityRadPerSec = sim.getRightVelocityMetersPerSecond();
        inputs.rightVelocityMetersPerSecond = sim.getRightVelocityMetersPerSecond();
        inputs.rightVelocityGoalMetersPerSecond = (leftPID.getSetpoint());
        inputs.rightAppliedVolts = rightAppliedVolts;
        inputs.rightCurrentAmps = new double[] {sim.getRightCurrentDrawAmps()};
        // inputs.rightTempCelsius = new double[] {};

        inputs.isClosedLoop = isClosedLoop;
        inputs.gyroYaw = sim.getHeading();
    }

    @Override
    public void setVolts(final double left, final double right) {
        leftAppliedVolts = -left;
        rightAppliedVolts = -right;
        isClosedLoop = false;
    }

    @Override
    public void setMetersPerSecond(final double left, final double right) {
        isClosedLoop = true;
        leftPID.setSetpoint(left);
        rightPID.setSetpoint(right);
    }
    
}
