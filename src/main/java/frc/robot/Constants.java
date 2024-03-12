package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final class IntakeConstants {
        public static final int intakeMotorID = 23;
        public static final double INTAKE_FORWARD = -1;
        public static final double INTAKE_REVERSE = 1;
    }

    public static final class IndexerConstants {
        public static final int ID = 22;
        public static final double FORWARD_DUTY_CYCLE = -0.7;
        public static final double FLUSH_DUTY_CYCLE = 0.75;
        
        public static final int CURRENT_CYCLES = 20;
        public static final int CUTOFF_VOLTAGE = 10;
    }

    public static final class ShooterConstants {
        public static final double launchVoltage = 12.0;
        public static final double flushVoltage = -6.0;

        public static final double SHOOTER_PID_TOLERANCE = 5;
        public static final double MAX_SPEED = 45;
        public static final double INTAKE_SPEED = -15;
        public static final double SPEAKER_SPEED = 45; // 0-100 rev per second
        public static final double AMP_SPEED = 5; // 0-100

        public static final InterpolatingDoubleTreeMap shooterMap = new InterpolatingDoubleTreeMap();

        public static final TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
        public static final int rightShooterID = 20;
        public static final int leftShooterID = 21;

        static {
            shooterMap.put(0.0, 45.0);
            shooterMap.put(1.00, 45.0);
            shooterMap.put(1.53, 45.0);
            shooterMap.put(1.85, 45.0);
            shooterMap.put(2.16, 45.0);
            shooterMap.put(2.68, 45.0);
            shooterMap.put(3.08, 45.0);
            shooterMap.put(3.2, 45.0);
            shooterMap.put(3.36, 45.0);
            shooterMap.put(3.53, 44.0);
            shooterMap.put(3.96, 41.0);

            talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

            talonFXConfigs.Voltage.PeakForwardVoltage = 12.0;
            talonFXConfigs.Voltage.PeakReverseVoltage = -12.0;

            // set slot 0 gains
            talonFXConfigs.Slot0.kS = 0.11591;
            talonFXConfigs.Slot0.kV = 0.12626;
            talonFXConfigs.Slot0.kA = 0.009693;
            talonFXConfigs.Slot0.kP = 0.5;
            talonFXConfigs.Slot0.kI = 0.0;
            talonFXConfigs.Slot0.kD = 0.0;

            // set Motion Magic settings
            talonFXConfigs.MotionMagic.MotionMagicAcceleration = 1000;
        }
    }

    public static final class TilterConstants {
        public static final int tilterMotorID = 24;
        public static final double PID_TOLERANCE = 0.05;
        public static final double START_POSITION = 160 / 4.0; // 0-50
        public static final double IDLE_POSITION = 80 / 4.0; // 0-50
        public static final double SPEAKER_CENTER_POSITION = 130.0 / 4.0; // 0-50
        public static final double SPEAKER_CORNER_POSITION = 145.0 / 4.0; // 0-50
        public static final double AMP_POSITION = 185 / 4.0; // 0-50
        public static final double TRAP_POSITION = 135 / 4.0; // 0-50
        public static final double HUMAN_INTAKE_POSITION = 95.0 / 4.0; // 0-50
        public static final double GROUND_INTAKE_POSITION = 140.0 / 4.0; // 0-50
        public static final double MAX_POSITION = 50; // 0-50
      
        public static final int kLIFTER_LIMIT_BOTTOM = 0;

        // 1 falcon rotation = 12 mm of travel (0.47 inches)
        // 25.53 rotation for full extension
        // 8 inches long
        public static final double ROTATION_TO_METERS = 0.012;
        public static final double LENGTH_METERS = Units.inchesToMeters(8);
        public static final double GEAR_RATIO = 4.0;
        public static final double ROTATIONS_TO_FULL_EXTENSION = LENGTH_METERS / ROTATION_TO_METERS * GEAR_RATIO; // 17.02 rotation for full extension
        // 219 rotations
        // 12:1

        public static final InterpolatingDoubleTreeMap tilterMap = new InterpolatingDoubleTreeMap();

        public static final TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
        static {
            tilterMap.put(1.23, 135.0 / 4.0);
            tilterMap.put(1.32, 130.0 / 4.0);
            tilterMap.put(1.4, 125.0 / 4.0);
            tilterMap.put(1.53, 110.0 / 4.0);
            tilterMap.put(1.7, 100.0 / 4.0);
            tilterMap.put(1.86, 85.0 / 4.0);
            tilterMap.put(2.16, 60.0 / 4.0);
            tilterMap.put(2.39, 40.0 / 4.0);
            tilterMap.put(2.55, 30.0 / 4.0);
            tilterMap.put(2.68, 17.0 / 4.0);
            tilterMap.put(2.8, 10.0 / 4.0);
            tilterMap.put(3.07, 1.5);
            tilterMap.put(3.2, 1.0);
            tilterMap.put(3.53, 0.75);
            tilterMap.put(3.96, 0.25);

            talonFXConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

            talonFXConfigs.Voltage.PeakForwardVoltage = 12.0;
            talonFXConfigs.Voltage.PeakReverseVoltage = -12.0;

            talonFXConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ROTATIONS_TO_FULL_EXTENSION;
            talonFXConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
            talonFXConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
            talonFXConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

            // set slot 0 gains
//            talonFXConfigs.Slot0.kS = 1; // Add 0.25 V output to overcome static friction
//            talonFXConfigs.Slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
//            talonFXConfigs.Slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
            talonFXConfigs.Slot0.kP = 10; // old: 5.0
            talonFXConfigs.Slot0.kI = 0; // no output for integrated error
            talonFXConfigs.Slot0.kD = 0; // A velocity error of 1 rps results in 0.1 V output

            // set Motion Magic settings
            talonFXConfigs.MotionMagic.MotionMagicCruiseVelocity = 80; // old: 110
            talonFXConfigs.MotionMagic.MotionMagicAcceleration = 400; // old: 100
//            talonFXConfigs.MotionMagic.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)
        }
    }

    public static final class ClimberConstants {
        public static final int LIMIT_SWITCH_BOTTOM_ID = 2;
        public static final int LIMIT_SWITCH_TOP_ID = 1;

        public static final double climbHeight = 0;

        // 1 falcon rotation = 12 mm of travel (0.47 inches)
//        public static final double ROTATION_TO_INCHES = Units.metersToInches(0.012);
//        public static final double LENGTH_INCHES = Units.inchesToMeters(12);
//        public static final double ROTATIONS_TO_FULL_EXTENSION = LENGTH_INCHES / ROTATION_TO_INCHES; // 25.53 rotation for full extension

        public static final TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
        public static final int rightClimberMotorID = 25;
        public static final int leftClimberMotorID = 26;

        static {
            var softLimit = talonFXConfigs.SoftwareLimitSwitch;
//            softLimit.ForwardSoftLimitThreshold = ROTATIONS_TO_FULL_EXTENSION;
//            softLimit.ForwardSoftLimitEnable = true;
//            softLimit.ReverseSoftLimitThreshold = 0;
//            softLimit.ReverseSoftLimitEnable = false;

            // set slot 0 gains
//            var slot0Configs = talonFXConfigs.Slot0;
//            slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
//            slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
//            slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
//            slot0Configs.kP = 0.2; // A position error of 2.5 rotations results in 12 V output
//            slot0Configs.kI = 0; // no output for integrated error
//            slot0Configs.kD = 0; // A velocity error of 1 rps results in 0.1 V output

            // set Motion Magic settings
//            var motionMagicConfigs = talonFXConfigs.MotionMagic;
//            motionMagicConfigs.MotionMagicCruiseVelocity = 10; // Target cruise velocity of 80 rps
//            motionMagicConfigs.MotionMagicAcceleration = 5; // Target acceleration of 160 rps/s (0.5 seconds)
//            motionMagicConfigs.MotionMagicJerk = 0; // Target jerk of 1600 rps/s/s (0.1 seconds)
        }
    }

    public static final class SwerveConstants {
        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(31 - 2.625 * 2); // 31
        public static final double wheelBase = Units.inchesToMeters(28 - 2.625 * 2); // 28

        public static final double AUTO_LINEAR_P = 10;
        public static final double AUTO_ROT_P = 10;
        public static final double DRIVEBASE_RADIUS = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
        public static final double AUTO_MAX_SPEED = 1;

        public static final double ROTATE_P = 5;
        public static final double ROTATE_I = 0;
        public static final double ROTATE_D = 0;
        public static final double ROTATE_FF = 1;
        public static final double ROTATE_TOLERANCE = 0.1;

        public static final int pigeonID = 28;

        public static final COTSTalonFXSwerveConstants chosenModule = COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* SwerveConstants Kinematics
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* SwerveConstants Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 50;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.52307;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.0906094;
        public static final double driveKV = 2.875;
        public static final double driveKA = 0.22967;

        /* SwerveConstants Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = Units.feetToMeters(13.7);
        /** Radians per Second */
        public static final double maxAngularVelocity = 10;

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 6;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-75.41);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 8;
            public static final int angleMotorID = 9;
            public static final int canCoderID = 7;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-158.11);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 11;
            public static final int angleMotorID = 10;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(169.8);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 15;
            public static final int angleMotorID = 14;
            public static final int canCoderID = 16;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-39.55);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants {
        public static final Pose2d TRAP_POSITION = new Pose2d();

        public static final double kMaxSpeedMetersPerSecond = 3.7;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class VisionConstants {
        public static final Translation3d LEFT_CAMERA_TRANSLATION = new Translation3d(Units.inchesToMeters(-8), Units.inchesToMeters(-11), Units.inchesToMeters(16));
        public static final Rotation3d LEFT_CAMERA_ROTATION = new Rotation3d(0, Units.degreesToRadians(-30), Units.degreesToRadians(180));

        public static final Translation3d RIGHT_CAMERA_TRANSLATION = new Translation3d(Units.inchesToMeters(-8), Units.inchesToMeters(11), Units.inchesToMeters(16));
        public static final Rotation3d RIGHT_CAMERA_ROTATION = new Rotation3d(0, Units.degreesToRadians(-30), Units.degreesToRadians(180));
    }
}