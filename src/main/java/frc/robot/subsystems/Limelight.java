package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.Speaker;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.Optional;

public class Limelight extends SubsystemBase {
    private final PhotonPoseEstimator poseEstimatorLeft;
    private final PhotonPoseEstimator poseEstimatorRight;
    private final PhotonCamera camLeft;
    private final PhotonCamera camRight;
    private Optional<EstimatedRobotPose> leftPose;
    private Optional<EstimatedRobotPose> rightPose;
    private PhotonPipelineResult leftResult = new PhotonPipelineResult();
    private PhotonPipelineResult rightResult = new PhotonPipelineResult();
    private final AprilTagFieldLayout aprilTagFieldLayout;

    private Pose2d lastPose = new Pose2d();

    private static final Limelight INSTANCE = new Limelight();

    public static Limelight getInstance() {
        return INSTANCE;
    }

    private Limelight() {
        //    private final NetworkTable limelightRight = LimelightHelpers.getLimelightNTTable(limelightRightName);
        //    private final NetworkTable limelightLeft = LimelightHelpers.getLimelightNTTable(limelightLeftName);
        aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
        camLeft = new PhotonCamera("left"); // 10.63.00.11
        camRight = new PhotonCamera("right"); // 10.63.00.22
        camLeft.setLED(VisionLEDMode.kOff);
        camRight.setLED(VisionLEDMode.kOff);

        poseEstimatorLeft = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camLeft, new Transform3d(VisionConstants.LEFT_CAMERA_TRANSLATION, VisionConstants.LEFT_CAMERA_ROTATION));
        poseEstimatorLeft.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        poseEstimatorRight = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camRight, new Transform3d(VisionConstants.RIGHT_CAMERA_TRANSLATION, VisionConstants.RIGHT_CAMERA_ROTATION));
        poseEstimatorRight.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
//        setLEDs(LED.OFF);
//        setPipeline(Pipelines.APRIL);
    }

    @Override
    public void periodic() {
        leftPose = poseEstimatorLeft.update();
        rightPose = poseEstimatorRight.update();

        leftResult = camLeft.getLatestResult();
        rightResult = camRight.getLatestResult();

        if (DriverStation.getAlliance().isPresent()) {
            Pose2d adjustedSpeaker = FieldConstants.allianceFlipper(new Pose2d(Speaker.centerSpeakerOpening.getX(), Speaker.centerSpeakerOpening.getY(), new Rotation2d()), DriverStation.getAlliance().get());

//        if (!DriverStation.isAutonomous()) {
//            SmartDashboard.putBoolean("right has targets", rightResult.hasTargets());
//            SmartDashboard.putBoolean("right pose present", rightPose.isPresent());
//            SmartDashboard.putNumber("right targets size", rightResult.targets.size());
//            SmartDashboard.putBoolean("left has targets", leftResult.hasTargets());
//            SmartDashboard.putBoolean("left pose present", leftPose.isPresent());
//            SmartDashboard.putNumber("left targets size", leftResult.targets.size());

            if (rightResult.hasTargets() && rightPose.isPresent()) {
                Pose3d pose3d = rightPose.get().estimatedPose;
                SmartDashboard.putNumberArray("limelight right pose", new double[]{pose3d.getX(), pose3d.getY(), pose3d.getRotation().toRotation2d().getRadians()});
                if (rightResult.targets.size() >= 2) {
                    Swerve.getInstance().addVision(new Pose2d(pose3d.getX(), pose3d.getY(), pose3d.getRotation().toRotation2d()), rightPose.get().timestampSeconds);
                } else if (!DriverStation.isAutonomous()) {
                    if (distance(adjustedSpeaker, new Pose2d(pose3d.getX(), pose3d.getY(), pose3d.getRotation().toRotation2d())) < 3.0) {
                        Swerve.getInstance().addVision(new Pose2d(pose3d.getX(), pose3d.getY(), pose3d.getRotation().toRotation2d()), rightPose.get().timestampSeconds);
                    }
//                    if (distance(Swerve.getInstance().getPose(), new Pose2d(pose3d.getX(), pose3d.getY(), pose3d.getRotation().toRotation2d())) < 0.5) {
//                        Swerve.getInstance().addVision(new Pose2d(pose3d.getX(), pose3d.getY(), pose3d.getRotation().toRotation2d()), rightPose.get().timestampSeconds);
//                    }
                }
            }
            if (leftResult.hasTargets() && leftPose.isPresent()) {
                Pose3d pose3d = leftPose.get().estimatedPose;
                SmartDashboard.putNumberArray("limelight left pose", new double[]{pose3d.getX(), pose3d.getY(), pose3d.getRotation().toRotation2d().getRadians()});
                if (leftResult.targets.size() >= 2) {
                    Swerve.getInstance().addVision(new Pose2d(pose3d.getX(), pose3d.getY(), pose3d.getRotation().toRotation2d()), leftPose.get().timestampSeconds);
                } else if (!DriverStation.isAutonomous()) {
                    if (distance(adjustedSpeaker, new Pose2d(pose3d.getX(), pose3d.getY(), pose3d.getRotation().toRotation2d())) < 3.0) {
                        Swerve.getInstance().addVision(new Pose2d(pose3d.getX(), pose3d.getY(), pose3d.getRotation().toRotation2d()), leftPose.get().timestampSeconds);
                    }
//                    if (distance(Swerve.getInstance().getPose(), new Pose2d(pose3d.getX(), pose3d.getY(), pose3d.getRotation().toRotation2d())) < 0.5) {
//                        Swerve.getInstance().addVision(new Pose2d(pose3d.getX(), pose3d.getY(), pose3d.getRotation().toRotation2d()), rightPose.get().timestampSeconds);
//                    }
                }
            }
        }


//        SmartDashboard.putNumberArray("limelight bot pose", new double[]{getBotPose().getX(), getBotPose().getY(), getBotPose().getRotation().getRadians()});
        SmartDashboard.putBoolean("has target", hasTarget());
//        SmartDashboard.putNumber("wanted rotation", getRotationToTarget().rotateBy(Rotation2d.fromRadians(Math.PI)).getDegrees());
        SmartDashboard.putNumber("distance to target", distanceToTarget());
    }

    public Rotation2d getRotationToTarget() {
        if (DriverStation.getAlliance().isPresent() && hasTarget()) {
            Pose2d adjustedSpeaker = FieldConstants.allianceFlipper(new Pose2d(Speaker.centerSpeakerOpening.getX(), Speaker.centerSpeakerOpening.getY(), new Rotation2d()), DriverStation.getAlliance().get());
            double xDiff = adjustedSpeaker.getX() - getBotPose().getX();
            double yDiff = adjustedSpeaker.getY() - getBotPose().getY();
            return new Rotation2d(xDiff, yDiff);
        }
        return new Rotation2d();
    }

    public boolean hasTarget() {
        return leftResult.hasTargets() || rightResult.hasTargets();
    }

    public Pose2d getBotPose() {
        if (leftResult.hasTargets() && rightResult.hasTargets() && leftPose.isPresent() && rightPose.isPresent()) {
            Pose3d pose3dLeft = leftPose.get().estimatedPose;
            Pose3d pose3dRight = rightPose.get().estimatedPose;
            if (leftResult.targets.size() > 2) {
                lastPose = new Pose2d(pose3dLeft.getX(), pose3dLeft.getY(), pose3dLeft.getRotation().toRotation2d());
                return lastPose;
            }
            else if (rightResult.targets.size() > 2) {
                lastPose = new Pose2d(pose3dRight.getX(), pose3dRight.getY(), pose3dRight.getRotation().toRotation2d());
                return lastPose;
            }
            lastPose = new Pose2d(mean(pose3dLeft.getX(), pose3dRight.getX()), mean(pose3dLeft.getY(), pose3dRight.getY()), Rotation2d.fromDegrees(mean(pose3dLeft.getRotation().toRotation2d().getDegrees(), pose3dRight.getRotation().toRotation2d().getDegrees())));
            return lastPose;
        }
        else if (rightResult.hasTargets() && rightPose.isPresent()) {
            Pose3d pose3d = rightPose.get().estimatedPose;
            lastPose = new Pose2d(pose3d.getX(), pose3d.getY(), pose3d.getRotation().toRotation2d());
            return lastPose;
        }
        else if (leftResult.hasTargets() && leftPose.isPresent()) {
            Pose3d pose3d = leftPose.get().estimatedPose;
            lastPose = new Pose2d(pose3d.getX(), pose3d.getY(), pose3d.getRotation().toRotation2d());
            return lastPose;
        }
        return lastPose;
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

    private double distance(Pose2d pose1, Pose2d pose2) {
        Pose2d relPose = pose1.relativeTo(pose2);
        return Math.sqrt(Math.pow(relPose.getX(), 2) + Math.pow(relPose.getY(), 2));
    }

    private double mean(double a, double b) {
        return (a + b)/2;
    }
}