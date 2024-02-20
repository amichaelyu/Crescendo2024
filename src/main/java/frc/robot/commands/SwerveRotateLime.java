package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.Speaker;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

import java.util.Objects;


public class SwerveRotateLime extends Command {
    private final Swerve swerve = Swerve.getInstance();
    private final Limelight limelight = Limelight.getInstance();
    private final PIDController pidController;
    private Pose2d adjustedSpeaker = new Pose2d();
    private Rotation2d wantedRotation;

    public SwerveRotateLime() {
        pidController = new PIDController(SwerveConstants.ROTATE_P, SwerveConstants.ROTATE_I, SwerveConstants.ROTATE_D);
        addRequirements(this.swerve, this.limelight);
    }

    @Override
    public void initialize() {
        if (DriverStation.getAlliance().isPresent()) {
            adjustedSpeaker = FieldConstants.allianceFlipper(new Pose2d(Speaker.centerSpeakerOpening.getX(), Speaker.centerSpeakerOpening.getY(), new Rotation2d()), DriverStation.getAlliance().get());
            double xDiff = adjustedSpeaker.getX() - limelight.getBotPose().getX();
            double yDiff = adjustedSpeaker.getY() - limelight.getBotPose().getY();
            double angle = Math.atan(xDiff / yDiff);
            double offset = 0;
            if ((- Math.PI / 2) - angle > (2 * Math.PI)) {
                offset = - Math.PI * 2;
            }
            else if ((- Math.PI / 2) - angle < 0) {
                offset = Math.PI * 2;
            }
            wantedRotation = Rotation2d.fromRadians((-Math.PI / 2) - angle + offset);
            SmartDashboard.putNumber("wanted lime rot", wantedRotation.getRadians());
            pidController.setSetpoint(wantedRotation.getRadians());
            pidController.setTolerance(SwerveConstants.ROTATE_TOLERANCE);
            pidController.enableContinuousInput(0, 2 * Math.PI);
        }
    }

    @Override
    public void execute() {
        double feedforward = swerve.getPose().getRotation().getRadians() < wantedRotation.getRadians() ? SwerveConstants.ROTATE_FF : -SwerveConstants.ROTATE_FF;
        swerve.drive(new Translation2d(), pidController.calculate(swerve.getPose().getRotation().getRadians()) + feedforward, true, false);
    }

    @Override
    public boolean isFinished() {
        return Objects.equals(adjustedSpeaker, new Pose2d()) || pidController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.driveX();
    }
}