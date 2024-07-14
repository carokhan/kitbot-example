package frc.robot;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;;

public final class Constants {
    public static final class MotorConstants {
        public static final int driveLeftMain = 1;
        public static final int driveLeftFollow = 2;

        public static final int driveRightMain = 3;
        public static final int driveRightFollow = 4;

        public static final int climber = 31;
    }
    
    public static class RobotConstants {
        public static double mass = Units.lbsToKilograms(154);
    }

    public static class DriveConstants {
        public static double wheelRadius = Units.inchesToMeters(3.0);
        public static double trackWidth = Units.inchesToMeters(26.0);
        public static double gearRatio = 8.46;

        public static double kPSim = 0.2;
        public static double kISim = 0.0;
        public static double kDSim = 0.0;

        public static double kPReal = 0.2;
        public static double kIReal = 0.0;
        public static double kDReal = 0.0;

        public static double kS = 0.0;
        public static double kV = 0.0;

        public static double kLeftFFVoltsSim = 0.0;
        public static double kRightFFVoltsSim = 0.0;

        public static boolean isClosedLoop = false;
    }

    public static class ClimbConstants {
        public static double gearRatio = 45;
        public static double spoolRadius = Units.inchesToMeters(.75);
        public static double encoderConversion = 2 * spoolRadius * Math.PI / gearRatio;
        public static double width = Units.inchesToMeters(2.0);

        public static double minHeight = 0.0;
        public static double maxHeight = Units.inchesToMeters(18);

        public static double kPSim = 20;
        public static double kISim = 0.0;
        public static double kDSim = 0.0;

        public static double kPReal = 0.0;
        public static double kIReal = 0.0;
        public static double kDReal = 0.0;

        public static double kSSim = 0.0;
        public static double kGSim = 0.0;
        public static double kVSim = 0.0;
        public static double kASim = 0.09;

        public static double kFFReal = 0.0;

        public static double maxVelocity = Units.inchesToMeters(17);
        public static double maxAccel = Units.inchesToMeters(180);
    }

    public static final Mode currentMode = Mode.SIM;

    public static enum Mode {
        REAL,
        SIM,
        REPLAY
    }
    
}
