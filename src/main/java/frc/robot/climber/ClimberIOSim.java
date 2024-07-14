package frc.robot.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

import static frc.robot.Constants.RobotConstants;
import static frc.robot.Constants.ClimbConstants;

public class ClimberIOSim implements ClimberIO {
   private ElevatorSim sim = new ElevatorSim(
    DCMotor.getNEO(1),
    ClimbConstants.gearRatio,
    1,
    ClimbConstants.spoolRadius,
    ClimbConstants.minHeight,
    ClimbConstants.maxHeight,
    true, 
    0
   );

   private double appliedVolts = 0.0;
   private ProfiledPIDController pid = new ProfiledPIDController(
    ClimbConstants.kPSim, 
    ClimbConstants.kISim, 
    ClimbConstants.kDSim, 
    new Constraints(
        ClimbConstants.maxVelocity, 
        ClimbConstants.maxAccel
        )
        );
    private ElevatorFeedforward ff = new ElevatorFeedforward(
        ClimbConstants.kSSim, 
        ClimbConstants.kGSim, 
        ClimbConstants.kVSim, 
        ClimbConstants.kASim
        );

   @Override
   public void updateInputs(final ClimberIOInputsAutoLogged inputs) {
       if (DriverStation.isDisabled()) {
        stop();
       }

       sim.update(0.02);
       inputs.climberPositionMeters = sim.getPositionMeters();
       inputs.climberVelocityMetersPerSecond = sim.getVelocityMetersPerSecond();
       inputs.climberAppliedVolts = appliedVolts;
       inputs.climberCurrentAmps = sim.getCurrentDrawAmps();
   }

   @Override
   public void setTarget(final double meters) {
       setVoltage(
        pid.calculate(sim.getPositionMeters(), meters)
        + ff.calculate(pid.getSetpoint().velocity)
       );
   }

   @Override
   public void setVoltage(final double voltage) {
       appliedVolts = voltage;
       sim.setInputVoltage(MathUtil.clamp(voltage, -12.0, 12.0));
   }

   @Override
   public void resetEncoder(final double position) {
       sim.setState(position, 0.0);
   }

}
