package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LimelightHelpers;
import frc.robot.FieldConstants;

import java.util.Arrays;

public class Limelight extends SubsystemBase {
    private final double kEpsilon = 1e-12;
    private final String leftName = "limelight-left"; // 10.63.0.22 0.2 to right
    private final String rightName = "limelight-right"; // 10.63.0.11 -0.2 to right
    private final double fieldBorderMargin = 0.5;
    private boolean neverBeenEnabled = true;
    private final Field2d leftPoseEstimatorField = new Field2d();
    private final Field2d rightPoseEstimatorField = new Field2d();

    private static final Limelight INSTANCE = new Limelight();

    public static Limelight getInstance() {
        return INSTANCE;
    }

    private Limelight() {
        SmartDashboard.putData("Left LL Pose", leftPoseEstimatorField);
        SmartDashboard.putData("Right LL Pose", rightPoseEstimatorField);
    }

    @Override
    public void periodic() {
        if (neverBeenEnabled) {
            neverBeenEnabled = !DriverStation.isEnabled();
        }

        String[] limelights = {
                leftName,
                rightName
        };

        for (int i = 0; i < 2; ++i) {
            if (DriverStation.getAlliance().isPresent()) {
                Pose2d robotPose2d = LimelightHelpers.getBotPose2d_wpiBlue(limelights[i]);
                Pose3d robotPose3d = LimelightHelpers.getBotPose3d_wpiBlue(limelights[i]);
                if (!(LimelightHelpers.getLimelightNTTable(limelights[i]).getEntry("hb").getDouble(0.0) > 0.0)
                        || robotPose2d.equals(new Pose2d())
                        || robotPose3d.equals(new Pose3d())
                ) {
                    continue;
                }

                SmartDashboard.putNumberArray("LLPose3Draw" + i, new double[]{robotPose3d.getX(), robotPose3d.getY(), robotPose3d.getZ()});
                SmartDashboard.putNumberArray("LLPose2Draw" + i, new double[]{robotPose2d.getX(), robotPose2d.getY(), robotPose2d.getRotation().getRadians()});

                if (robotPose3d.getX() < -fieldBorderMargin
                        || robotPose3d.getX() > FieldConstants.fieldLength + fieldBorderMargin
                        || robotPose3d.getY() < -fieldBorderMargin
                        || robotPose3d.getY() > FieldConstants.fieldWidth + fieldBorderMargin
                        || robotPose3d.getZ() < -0.4
                        || robotPose3d.getZ() > 0.1) {
                    continue;
                }

                double[] poseDump = LimelightHelpers.getLimelightNTTable(limelights[i]).getEntry("botpose_wpiblue").getDoubleArray(new double[3]);
                if (Arrays.equals(poseDump, new double[3])) {
                    continue;
                }
                double avgDist = poseDump[9];
                double tagCount = poseDump[7];

                if (MathUtil.isNear(avgDist, 0.0, kEpsilon) || MathUtil.isNear(tagCount, 0.0, kEpsilon)) {
                    continue;
                }

                double ambg = poseDump[17];

                if (tagCount == 1 && ambg >= 0.9) {
                    continue;
                }

                SmartDashboard.putNumber("LLavgDist" + i, avgDist);
                SmartDashboard.putNumber("LLtagCount" + i, tagCount);

                boolean headingCorrecting = (tagCount >= 2) || neverBeenEnabled;
                SmartDashboard.putBoolean("LLheadingCorrecting" + i, headingCorrecting);

                double autoWeight = DriverStation.isAutonomous() ? 0.5 : 1;
                double xyStdDev = (0.01
                                * Math.pow(avgDist, 2.0)
                                / tagCount) * autoWeight;
                double thetaStdDev = (headingCorrecting ? 0.01
                        * Math.pow(avgDist, 2.0)
                        / tagCount : Double.POSITIVE_INFINITY) * autoWeight;
                SmartDashboard.putNumber("LLxyStd" + i, xyStdDev);
                SmartDashboard.putNumber("LLthetaStd" + i, thetaStdDev);

                if (MathUtil.isNear(xyStdDev, 0.0, kEpsilon) || MathUtil.isNear(thetaStdDev, 0.0, kEpsilon)) {
                    continue;
                }

                if (!headingCorrecting
                    && Math.abs(Swerve.getInstance().getPose().getRotation().minus(robotPose2d.getRotation()).getRadians()) > Units.degreesToRadians(5)) {
                    continue;
                }

                if (DriverStation.isAutonomous())
                {
                    if (avgDist > 4.0) {
                        continue;
                    }
                }

                if (i == 0) {
                    leftPoseEstimatorField.setRobotPose(robotPose2d);
                }
                else {
                    rightPoseEstimatorField.setRobotPose(robotPose2d);
                }
                SmartDashboard.putNumberArray("LLPose3Daccepted" + i, new double[]{robotPose3d.getX(), robotPose3d.getY(), robotPose3d.getZ()});
                SmartDashboard.putNumberArray("LLPose2Daccepted" + i, new double[]{robotPose2d.getX(), robotPose2d.getY(), robotPose2d.getRotation().getRadians()});
                Swerve.getInstance().addVision(robotPose2d, Timer.getFPGATimestamp() - Units.millisecondsToSeconds(LimelightHelpers.getLatency_Pipeline(limelights[i]) + LimelightHelpers.getLatency_Capture(limelights[i])), xyStdDev, thetaStdDev);
            }
        }

        SmartDashboard.putBoolean("has right target", hasTargetRight());
        SmartDashboard.putBoolean("has left target", hasTargetLeft());
    }

    public boolean hasTargetLeft() {
        return LimelightHelpers.getTV(leftName);
    }

    public boolean hasTargetRight() {
        return LimelightHelpers.getTV(rightName);
    }

    public Pose2d getBotPose() {
        if (hasTargetLeft() & hasTargetRight()) {
            Double[] leftPoseDump = LimelightHelpers.getLimelightNTTable(leftName).getEntry("botpose_wpiblue").getDoubleArray(new Double[]{});
            Double[] rightPoseDump = LimelightHelpers.getLimelightNTTable(rightName).getEntry("botpose_wpiblue").getDoubleArray(new Double[]{});
            if (!Arrays.equals(leftPoseDump, new Double[]{}) && !Arrays.equals(rightPoseDump, new Double[]{})) {
                double tagCountRight = rightPoseDump[7];
                double tagCountLeft = leftPoseDump[7];
                if (tagCountRight > tagCountLeft) {
                    return LimelightHelpers.getBotPose2d_wpiBlue(rightName);
                }
                else {
                    return LimelightHelpers.getBotPose2d_wpiBlue(leftName);
                }
            }
        }
        else if (hasTargetRight()) {
            return LimelightHelpers.getBotPose2d_wpiBlue(rightName);
        }
        else if (hasTargetLeft()) {
            return LimelightHelpers.getBotPose2d_wpiBlue(leftName);
        }
        return new Pose2d();
    }
}