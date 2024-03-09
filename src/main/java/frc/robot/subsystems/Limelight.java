package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.Speaker;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class Limelight extends SubsystemBase {
    private final PhotonPoseEstimator poseEstimatorLeft;
    private final PhotonPoseEstimator poseEstimatorRight;
    private final PhotonCamera camLeft;
    private final PhotonCamera camRight;

    private static final Limelight INSTANCE = new Limelight();

    public static Limelight getInstance() {
        return INSTANCE;
    }

    private Limelight() {
        //    private final NetworkTable limelightRight = LimelightHelpers.getLimelightNTTable(limelightRightName);
        //    private final NetworkTable limelightLeft = LimelightHelpers.getLimelightNTTable(limelightLeftName);
        AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
        camLeft = new PhotonCamera("left"); // 10.63.00.11
        camRight = new PhotonCamera("right"); // 10.63.00.22

        poseEstimatorLeft = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camLeft, new Transform3d(new Translation3d(Units.inchesToMeters(-8), Units.inchesToMeters(-11), Units.inchesToMeters(16)), new Rotation3d(0, Units.degreesToRadians(-30), Units.degreesToRadians(180))));
        poseEstimatorLeft.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        poseEstimatorRight = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camRight, new Transform3d(new Translation3d(Units.inchesToMeters(-8), Units.inchesToMeters(11), Units.inchesToMeters(16)), new Rotation3d(0, Units.degreesToRadians(-30), Units.degreesToRadians(180))));
        poseEstimatorRight.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
//        setLEDs(LED.OFF);
//        setPipeline(Pipelines.APRIL);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("has target", hasTarget());
        SmartDashboard.putNumber("wanted rotation", getRotationToTarget().rotateBy(Rotation2d.fromRadians(Math.PI)).getDegrees());




        if (camRight.getLatestResult().hasTargets() && poseEstimatorRight.update().isPresent()) {
            Pose3d pose3d = poseEstimatorRight.update().get().estimatedPose;
            SmartDashboard.putNumberArray("limelight left pose", new double[]{pose3d.getX(), pose3d.getY(), pose3d.getRotation().toRotation2d().getRadians()});
//            Swerve.getInstance().addVision(new Pose2d(pose3d.getX(), pose3d.getY(), pose3d.getRotation().toRotation2d()), camRight.getLatestResult().getTimestampSeconds());
        }
        if (camLeft.getLatestResult().hasTargets() && poseEstimatorLeft.update().isPresent()) {
            Pose3d pose3d = poseEstimatorLeft.update().get().estimatedPose;
            SmartDashboard.putNumberArray("limelight right pose", new double[]{pose3d.getX(), pose3d.getY(), pose3d.getRotation().toRotation2d().getRadians()});
//            Swerve.getInstance().addVision(new Pose2d(pose3d.getX(), pose3d.getY(), pose3d.getRotation().toRotation2d()), camLeft.getLatestResult().getTimestampSeconds());
        }

//        if (DriverStation.getAlliance().isPresent() && hasTarget()) {
//            double limeRot = getRotationToTarget().getRadians();
//            double cramRot = Swerve.getInstance().getPose().getRotation().getRadians();
//            Rotation2d wantedRotation = Rotation2d.fromRadians(Swerve.getInstance().getPose().getRotation().getRadians() + limeRot - cramRot);
//            if (wantedRotation.getRadians() > Math.PI) {
//                wantedRotation = Rotation2d.fromRadians(wantedRotation.getRadians() - 2 * Math.PI);
//            }
//            else if (wantedRotation.getRadians() < -Math.PI) {
//                wantedRotation = Rotation2d.fromRadians(wantedRotation.getRadians() + 2 * Math.PI);
//            }
//            SmartDashboard.putNumber("wanted lime rot", getRotationToTarget().getRadians());
//        }
    }

    public Rotation2d getRotationToTarget() {
        if (DriverStation.getAlliance().isPresent() && hasTarget() && getBotPose() != null) {
            Pose2d adjustedSpeaker = FieldConstants.allianceFlipper(new Pose2d(Speaker.centerSpeakerOpening.getX(), Speaker.centerSpeakerOpening.getY(), new Rotation2d()), DriverStation.getAlliance().get());
            double xDiff = adjustedSpeaker.getX() - getBotPose().getX();
            double yDiff = adjustedSpeaker.getY() - getBotPose().getY();
            return new Rotation2d(xDiff, yDiff);
        }
        return new Rotation2d();
    }

    public boolean hasTarget() {
        return camLeft.getLatestResult().hasTargets() || camRight.getLatestResult().hasTargets();
    }

    public Pose2d getBotPose() {
        if (camLeft.getLatestResult().hasTargets() && camRight.getLatestResult().hasTargets() && poseEstimatorLeft.update().isPresent() && poseEstimatorRight.update().isPresent()) {
            Pose3d pose3d = poseEstimatorLeft.update().get().estimatedPose;
            Pose3d pose3dSecond = poseEstimatorRight.update().get().estimatedPose;
            return new Pose2d(mean(pose3d.getX(), pose3dSecond.getX()), mean(pose3d.getY(), pose3dSecond.getY()), Rotation2d.fromDegrees(mean(pose3d.getRotation().toRotation2d().getDegrees(), pose3dSecond.getRotation().toRotation2d().getDegrees())));
        }
        else if (camRight.getLatestResult().hasTargets() && poseEstimatorRight.update().isPresent()) {
            Pose3d pose3d = poseEstimatorRight.update().get().estimatedPose;
            return new Pose2d(pose3d.getX(), pose3d.getY(), pose3d.getRotation().toRotation2d());
        }
        else if (camLeft.getLatestResult().hasTargets() && poseEstimatorLeft.update().isPresent()) {
            Pose3d pose3d = poseEstimatorLeft.update().get().estimatedPose;
            return new Pose2d(pose3d.getX(), pose3d.getY(), pose3d.getRotation().toRotation2d());
        }
        return new Pose2d();
    }

    // in meters
    public double distanceToTarget() {
        Pose2d botPose = getBotPose();
        if (botPose != null) {
                if (DriverStation.getAlliance().isPresent()) {
                    Pose2d adjustedSpeaker = FieldConstants.allianceFlipper(new Pose2d(Speaker.centerSpeakerOpening.getX(), Speaker.centerSpeakerOpening.getY(), new Rotation2d()), DriverStation.getAlliance().get());
                    return distance(botPose, adjustedSpeaker);
                }
        }
        return 0.0;
    }

//    public void setPipeline(Pipelines pipeline) {
//        limelightRight.getEntry("pipeline").setDouble(pipeline.getNum());
//        limelightLeft.getEntry("pipeline").setDouble(pipeline.getNum());
//    }
//
//    public void setCameraModes(CameraMode camera) {
//        limelightRight.getEntry("camMode").setDouble(camera.getNum());
//        limelightLeft.getEntry("camMode").setDouble(camera.getNum());
//    }
//
//    public void setLEDs(LED led) {
//        limelightRight.getEntry("ledMode").setDouble(led.getNum());
//        limelightLeft.getEntry("ledMode").setDouble(led.getNum());
//    }

    private double distance(Pose2d pose1, Pose2d pose2) {
        Pose2d relPose = pose1.relativeTo(pose2);
        return Math.sqrt(Math.pow(relPose.getX(), 2) + Math.pow(relPose.getY(), 2));
    }

    private double mean(double a, double b) {
        return (a + b)/2;
    }
}