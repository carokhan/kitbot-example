package frc.robot.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import static frc.robot.Constants.ClimbConstants;
import static frc.robot.Constants.MotorConstants;

public class ClimberIOSparkMax implements ClimberIO {
    private final CANSparkMax motor = new CANSparkMax(MotorConstants.climber, MotorType.kBrushless);
    private final RelativeEncoder encoder = motor.getEncoder();
    
    private final SparkPIDController pid = motor.getPIDController();

    public ClimberIOSparkMax() {
        motor.restoreFactoryDefaults();
        motor.setCANTimeout(250);
        motor.setInverted(false);
        motor.enableVoltageCompensation(12.0);
        motor.setSmartCurrentLimit(30);

        encoder.setPositionConversionFactor(ClimbConstants.encoderConversion);
        encoder.setVelocityConversionFactor(ClimbConstants.encoderConversion / 60);

        pid.setP(ClimbConstants.kPReal);
        pid.setD(ClimbConstants.kDReal);
        pid.setFF(ClimbConstants.kFFReal);
        pid.setOutputRange(-1, 1);

        motor.burnFlash();
    }

    @Override
    public void updateInputs(final ClimberIOInputsAutoLogged inputs) {
        inputs.climberPositionMeters = encoder.getPosition();
        inputs.climberVelocityMetersPerSecond = encoder.getVelocity();
        inputs.climberAppliedVolts = motor.getBusVoltage() * motor.getAppliedOutput(); 
        inputs.climberCurrentAmps = motor.getOutputCurrent();
        inputs.climberTempCelsius = motor.getMotorTemperature();
    }

    @Override
    public void setTarget(final double meters) {
        pid.setReference(meters, ControlType.kPosition);
    }

    @Override
    public void setVoltage(final double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void resetEncoder(final double position) {
        encoder.setPosition(position);
    }
}
