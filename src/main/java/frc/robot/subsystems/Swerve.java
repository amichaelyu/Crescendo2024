package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.Speaker;
import frc.robot.SwerveModule;

public class Swerve extends SubsystemBase {
    private final SwerveDrivePoseEstimator poseEstimator;
    private final SwerveModule[] mSwerveMods;
    private final Pigeon2 gyro;

    private static final Swerve INSTANCE = new Swerve();

    public static Swerve getInstance() {
        return INSTANCE;
    }

    private Swerve() {
        gyro = new Pigeon2(SwerveConstants.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, SwerveConstants.Mod0.constants),
            new SwerveModule(1, SwerveConstants.Mod1.constants),
            new SwerveModule(2, SwerveConstants.Mod2.constants),
            new SwerveModule(3, SwerveConstants.Mod3.constants)
        };

        poseEstimator = new SwerveDrivePoseEstimator(SwerveConstants.swerveKinematics, getGyroYaw(), getModulePositions(), new Pose2d());

        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(SwerveConstants.AUTO_LINEAR_P, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(SwerveConstants.AUTO_ROT_P, 0.0, 0.0), // Rotation PID constants
                        SwerveConstants.AUTO_MAX_SPEED, // Max module speed, in m/s
                        SwerveConstants.DRIVEBASE_RADIUS, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig()
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }

    @Override
    public void periodic(){
        poseEstimator.update(getGyroYaw(), getModulePositions());

        SmartDashboard.putNumberArray("pose", new double[]{getPose().getX(), getPose().getY(), getPose().getRotation().getRadians()});

        SmartDashboard.putNumber("swerve degrees", getPose().getRotation().getDegrees());
        SmartDashboard.putNumber("swerve distance to target", distanceToTargetSwervePose());

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }
    }

    public void addVision(Pose2d pose, double time) {
        Pose2d poseWithoutHeading = new Pose2d(pose.getX(), pose.getY(), getPose().getRotation());
        poseEstimator.addVisionMeasurement(poseWithoutHeading, time);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            SwerveConstants.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public void driveX() {
        setModuleStates(new SwerveModuleState[]{
                new SwerveModuleState(0.1, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(0.1, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(0.1, Rotation2d.fromDegrees(315)),
                new SwerveModuleState(0.1, Rotation2d.fromDegrees(45)),
        });
    }

    public void driveForward() {
        setModuleStates(new SwerveModuleState[]{
                new SwerveModuleState(0.1, Rotation2d.fromDegrees(0)),
                new SwerveModuleState(0.1, Rotation2d.fromDegrees(0)),
                new SwerveModuleState(0.1, Rotation2d.fromDegrees(0)),
                new SwerveModuleState(0.1, Rotation2d.fromDegrees(0)),
        });
    }

    public void setChassisSpeeds(ChassisSpeeds speeds) {
        setModuleStates(SwerveConstants.swerveKinematics.toSwerveModuleStates(speeds));
    }

    public void setModuleVoltage(double voltage) {
        for(SwerveModule mod : mSwerveMods){
            mod.setVoltage(voltage);
        }
    }

    public double getMotorVoltage() {
        return mSwerveMods[0].getVoltage();
    }

    public double getMotorPosition() {
        return getModulePositions()[0].distanceMeters;
    }

    public double getMotorVelocity() {
        return mSwerveMods[0].getVelocity();
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return SwerveConstants.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(pose.getTranslation(), pose.getRotation()));
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading){
        poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading(){
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == DriverStation.Alliance.Red) {
                poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
            }
            else if (alliance.get() == DriverStation.Alliance.Blue) {
                poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
            }
        }
    }

    public void zeroHeadingWithVision(){
        if (Limelight.getInstance().hasTarget()) {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                if (alliance.get() == DriverStation.Alliance.Red) {
                    poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), Limelight.getInstance().getBotPose().getRotation().rotateBy(Rotation2d.fromDegrees(180))));
                } else if (alliance.get() == DriverStation.Alliance.Blue) {
                    poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), Limelight.getInstance().getBotPose().getRotation()));
                }
            }
        }
    }

    public Rotation2d getRotationToTargetSwervePose() {
        if (DriverStation.getAlliance().isPresent()) {
            Pose2d adjustedSpeaker = FieldConstants.allianceFlipper(new Pose2d(Speaker.centerSpeakerOpening.getX(), Speaker.centerSpeakerOpening.getY(), new Rotation2d()), DriverStation.getAlliance().get());
            double xDiff = adjustedSpeaker.getX() - Swerve.getInstance().getPose().getX();
            double yDiff = adjustedSpeaker.getY() - Swerve.getInstance().getPose().getY();
            return new Rotation2d(xDiff, yDiff);
        }
        return new Rotation2d();
    }

    public double distanceToTargetSwervePose() {
        Pose2d botPose = Swerve.getInstance().getPose();
        if (DriverStation.getAlliance().isPresent()) {
            Pose2d adjustedSpeaker = FieldConstants.allianceFlipper(new Pose2d(Speaker.centerSpeakerOpening.getX(), Speaker.centerSpeakerOpening.getY(), new Rotation2d()), DriverStation.getAlliance().get());
            return distance(botPose, adjustedSpeaker);
        }
        return 0.0;
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    private double distance(Pose2d pose1, Pose2d pose2) {
        Pose2d relPose = pose1.relativeTo(pose2);
        return Math.sqrt(Math.pow(relPose.getX(), 2) + Math.pow(relPose.getY(), 2));
    }
}