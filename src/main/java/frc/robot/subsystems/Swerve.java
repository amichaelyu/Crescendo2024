package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.mechanisms.swerve.*;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.GeomUtils;
import frc.lib.PointWheelsAtCustom;
import frc.robot.Constants.SwerveConstants;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.Speaker;
import frc.robot.TunerConstants;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Volts;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used in command-based projects easily.
 */
public class Swerve extends SwerveDrivetrain implements Subsystem {
    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean hasAppliedOperatorPerspective = false;

    private final SwerveRequest.ApplyChassisSpeeds AutoRequest = new SwerveRequest.ApplyChassisSpeeds();

    private final SwerveRequest.SysIdSwerveTranslation TranslationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveRotation RotationCharacterization = new SwerveRequest.SysIdSwerveRotation();
    private final SwerveRequest.SysIdSwerveSteerGains SteerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();

    public static Swerve getInstance() {
        return TunerConstants.DriveTrain;
    }

    /* Use one of these sysidroutines for your particular test */
    private final SysIdRoutine SysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    Volts.of(4),
                    null,
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    (volts) -> setControl(TranslationCharacterization.withVolts(volts)),
                    null,
                    this));

    private final SysIdRoutine SysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    Volts.of(4),
                    null,
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    (volts) -> setControl(RotationCharacterization.withVolts(volts)),
                    null,
                    this));
    private final SysIdRoutine SysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    Volts.of(7),
                    null,
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    (volts) -> setControl(SteerCharacterization.withVolts(volts)),
                    null,
                    this));

    /* Change this to the sysid routine you want to test */
    private final SysIdRoutine RoutineToApply = SysIdRoutineTranslation;

    public Swerve(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, 250, VecBuilder.fill(0.1, 0.1, 0.000001), VecBuilder.fill(0.3, 0.3, 9999999), modules);
        configurePathPlanner();
    }

    public void setChassisSpeed(ChassisSpeeds speeds) {
        setControl(new SwerveRequest.ApplyChassisSpeeds());
    }

    public void setPose(Pose2d pose) {
        m_odometry.resetPosition(Rotation2d.fromDegrees(m_pigeon2.getYaw().getValue()), m_modulePositions, pose);
    }

    public void driveX() {
        this.setControl(new PointWheelsAtCustom().withModuleDirection(
                new Rotation2d[]{
                        Rotation2d.fromDegrees(45),
                        Rotation2d.fromDegrees(-45),
                        Rotation2d.fromDegrees(315),
                        Rotation2d.fromDegrees(45)
                }));
    }

    private void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(
                ()->this.getState().Pose, // Supplier of current robot pose
                this::seedFieldRelative,  // Consumer for seeding pose against auto
                this::getCurrentRobotChassisSpeeds,
                (speeds) -> this.setControl(AutoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
                new HolonomicPathFollowerConfig(new PIDConstants(SwerveConstants.AUTO_LINEAR_P, 0, 0),
                        new PIDConstants(SwerveConstants.AUTO_ROT_P, 0, 0),
                        TunerConstants.kSpeedAt12VoltsMps,
                        driveBaseRadius,
                        new ReplanningConfig(true, true)
                ),
                () -> DriverStation.getAlliance().orElse(Alliance.Blue)==Alliance.Red, // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                this
        ); // Subsystem for requirements
    }

    public void zeroHeading() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == DriverStation.Alliance.Red) {
                this.m_odometry.resetPosition(Rotation2d.fromDegrees(this.m_pigeon2.getYaw().getValue()), m_modulePositions, new Pose2d(this.getState().Pose.getTranslation(), Rotation2d.fromDegrees(180)));
            }
            else if (alliance.get() == DriverStation.Alliance.Blue) {
                this.m_odometry.resetPosition(Rotation2d.fromDegrees(this.m_pigeon2.getYaw().getValue()), m_modulePositions, new Pose2d(this.getState().Pose.getTranslation(), Rotation2d.fromDegrees(0)));
            }
        }
    }

    public void drive(Translation2d translation2d, double rotation, boolean fieldRelative) {
        if (fieldRelative) {
            this.setControl(new SwerveRequest.FieldCentric()
                    .withVelocityX(translation2d.getX())
                    .withVelocityY(translation2d.getY())
                    .withRotationalRate(rotation));
        }
        else {
            this.setControl(new SwerveRequest.RobotCentric()
                    .withVelocityX(translation2d.getX())
                    .withVelocityY(translation2d.getY())
                    .withRotationalRate(rotation));
        }
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    /*
     * Both the sysid commands are specific to one particular sysid routine, change
     * which one you're trying to characterize
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return RoutineToApply.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return RoutineToApply.dynamic(direction);
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    public void addVision(Pose2d pose, double time, double std) {
        this.m_odometry.addVisionMeasurement(pose, time, VecBuilder.fill(std, std, 999999));
    }

    public Rotation2d getRotationToTargetSwervePose() {
        Pose2d botPose = this.getState().Pose;
        if (DriverStation.getAlliance().isPresent()) {
            Pose2d adjustedSpeaker = FieldConstants.allianceFlipper(new Pose2d(Speaker.centerSpeakerOpening.getX(), Speaker.centerSpeakerOpening.getY(), new Rotation2d()), DriverStation.getAlliance().get());
            double xDiff = adjustedSpeaker.getX() - botPose.getX();
            double yDiff = adjustedSpeaker.getY() - botPose.getY();
            return new Rotation2d(xDiff, yDiff);
        }
        System.out.println("No Alliance present! Failing to generate desired rotation!");
        return new Rotation2d();
    }

    public double distanceToTargetSwervePose() {
        Pose2d botPose = this.getState().Pose;
        if (DriverStation.getAlliance().isPresent()) {
            Pose2d adjustedSpeaker = FieldConstants.allianceFlipper(new Pose2d(Speaker.centerSpeakerOpening.getX(), Speaker.centerSpeakerOpening.getY(), new Rotation2d()), DriverStation.getAlliance().get());
            return GeomUtils.distance(botPose, adjustedSpeaker);
        }
        System.out.println("No Alliance present! Failing to generate desired distance!");
        return 0.0;
    }

    public Pose2d getPose() {
        return this.getState().Pose;
    }

    @Override
    public void periodic() {
        /* Periodically try to apply the operator perspective */
        /* If we haven't applied the operator perspective before, then we should apply it regardless of DS state */
        /* This allows us to correct the perspective in case the robot code restarts mid-match */
        /* Otherwise, only check and apply the operator perspective if the DS is disabled */
        /* This ensures driving behavior doesn't change until an explicit disable event occurs during testing*/
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent((allianceColor) -> {
                this.setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red ? RedAlliancePerspectiveRotation
                                : BlueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective = true;
            });
        }
    }
}