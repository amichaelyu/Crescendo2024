package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LimelightHelpers;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.Speaker;

public class Limelight extends SubsystemBase {

    public enum Pipelines {
        APRIL(0);

        private final int num;
        Pipelines(int num) {
            this.num = num;
        }

        public int getNum() {
            return num;
        }
    }

    public enum LED {
        PIPELINE(0), OFF(1), BLINK(2), ON(3);

        private final int num;

        LED(int num) {
            this.num = num;
        }

        public int getNum() {
            return num;
        }
    }

    public enum CameraMode {
        VISION_PROCESSING(0), DRIVER_CAMERA(1);

        private final int num;

        CameraMode(int num) {
            this.num = num;
        }

        public int getNum() {
            return num;
        }
    }

    private final String limelightRightName = "limelight-right";
    private final String limelightLeftName = "limelight-left";
    private final NetworkTable limelightRight = LimelightHelpers.getLimelightNTTable(limelightRightName);
    private final NetworkTable limelightLeft = LimelightHelpers.getLimelightNTTable(limelightLeftName);

    private static final Limelight INSTANCE = new Limelight();

    public static Limelight getInstance() {
        return INSTANCE;
    }

    private Limelight() {
        setLEDs(LED.OFF);
        setPipeline(Pipelines.APRIL);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("distance to target", distanceToTarget());
        SmartDashboard.putBoolean("has target", hasTarget());

        if (DriverStation.getAlliance().isPresent() && hasTarget()) {
            double limeRot = getRotationToTarget().getRadians();
            double cramRot = Swerve.getInstance().getPose().getRotation().getRadians() % (2 * Math.PI); //crammed rotation between 0 and 2pi
            if (cramRot < 0) {
                cramRot += Math.PI * 2;
            }
            Rotation2d wantedRotation = Rotation2d.fromRadians(Swerve.getInstance().getPose().getRotation().getRadians() + limeRot - cramRot);
            if (wantedRotation.getRadians() > Math.PI) {
                wantedRotation = Rotation2d.fromRadians(wantedRotation.getRadians() - 2 * Math.PI);
            }
            else if (wantedRotation.getRadians() < -Math.PI) {
                wantedRotation = Rotation2d.fromRadians(wantedRotation.getRadians() + 2 * Math.PI);
            }
            SmartDashboard.putNumber("wanted lime rot", wantedRotation.getRadians());
        }
    }

    public Rotation2d getRotationToTarget() {
        if (DriverStation.getAlliance().isPresent() && hasTarget()) {
            Pose2d adjustedSpeaker = FieldConstants.allianceFlipper(new Pose2d(Speaker.centerSpeakerOpening.getX(), Speaker.centerSpeakerOpening.getY(), new Rotation2d()), DriverStation.getAlliance().get());
            double xDiff = adjustedSpeaker.getX() - getBotPose().getX();
            double yDiff = adjustedSpeaker.getY() - getBotPose().getY();
            double angle = Math.atan(xDiff / yDiff);
            double offset = angle > 0 ? Math.PI / 2 : 3 * Math.PI / 2;
            return Rotation2d.fromRadians(offset - angle);
        }
        return new Rotation2d();
    }

    public boolean hasTarget() {
        return LimelightHelpers.getTV(limelightRightName) || LimelightHelpers.getTV(limelightLeftName);
    }

    public Pose2d getBotPose() {
        NetworkTableEntry botposeRightEntry;
        NetworkTableEntry botposeLeftEntry;
        if (LimelightHelpers.getTV(limelightRightName) && LimelightHelpers.getTV(limelightLeftName)) {
//            if (DriverStation.getAlliance().isPresent()) {
//                if (DriverStation.getAlliance().get() == Alliance.Blue) {
//                    botposeEntry = limelight.getEntry("botpose_wpiblue");
//                } else if (DriverStation.getAlliance().get() == Alliance.Red) {
//                    botposeEntry = limelight.getEntry("botpose_wpired");
//                }
                botposeRightEntry = limelightRight.getEntry("botpose_wpiblue");
                botposeLeftEntry = limelightLeft.getEntry("botpose_wpiblue");
                return new Pose2d(mean(botposeRightEntry.getDoubleArray(new double[7])[0], botposeLeftEntry.getDoubleArray(new double[7])[0]), mean(botposeRightEntry.getDoubleArray(new double[7])[1], botposeLeftEntry.getDoubleArray(new double[7])[1]), Rotation2d.fromDegrees(mean(botposeRightEntry.getDoubleArray(new double[7])[5], botposeLeftEntry.getDoubleArray(new double[7])[5]) + 180));
//            }
        }
        else if (LimelightHelpers.getTV(limelightRightName)) {
//            if (DriverStation.getAlliance().isPresent()) {
//                if (DriverStation.getAlliance().get() == Alliance.Blue) {
//                    botposeEntry = limelight.getEntry("botpose_wpiblue");
//                } else if (DriverStation.getAlliance().get() == Alliance.Red) {
//                    botposeEntry = limelight.getEntry("botpose_wpired");
//                }
            botposeRightEntry = limelightRight.getEntry("botpose_wpiblue");
            return new Pose2d(botposeRightEntry.getDoubleArray(new double[7])[0], botposeRightEntry.getDoubleArray(new double[7])[1], Rotation2d.fromDegrees(botposeRightEntry.getDoubleArray(new double[7])[5] + 180));
//            }
        }
        else if (LimelightHelpers.getTV(limelightLeftName)) {
//            if (DriverStation.getAlliance().isPresent()) {
//                if (DriverStation.getAlliance().get() == Alliance.Blue) {
//                    botposeEntry = limelight.getEntry("botpose_wpiblue");
//                } else if (DriverStation.getAlliance().get() == Alliance.Red) {
//                    botposeEntry = limelight.getEntry("botpose_wpired");
//                }
            botposeLeftEntry = limelightLeft.getEntry("botpose_wpiblue");
            return new Pose2d(botposeLeftEntry.getDoubleArray(new double[7])[0], botposeLeftEntry.getDoubleArray(new double[7])[1], Rotation2d.fromDegrees(botposeLeftEntry.getDoubleArray(new double[7])[5] + 180));
//            }
        }
        return null;
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

    public void setPipeline(Pipelines pipeline) {
        limelightRight.getEntry("pipeline").setDouble(pipeline.getNum());
        limelightLeft.getEntry("pipeline").setDouble(pipeline.getNum());
    }

    public void setCameraModes(CameraMode camera) {
        limelightRight.getEntry("camMode").setDouble(camera.getNum());
        limelightLeft.getEntry("camMode").setDouble(camera.getNum());
    }

    public void setLEDs(LED led) {
        limelightRight.getEntry("ledMode").setDouble(led.getNum());
        limelightLeft.getEntry("ledMode").setDouble(led.getNum());
    }

    private double distance(Pose2d pose1, Pose2d pose2) {
        Pose2d relPose = pose1.relativeTo(pose2);
        return Math.sqrt(Math.pow(relPose.getX(), 2) + Math.pow(relPose.getY(), 2));
    }

    private double mean(double a, double b) {
        return (a + b)/2;
    }
}