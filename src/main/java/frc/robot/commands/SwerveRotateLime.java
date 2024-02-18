package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.Speaker;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

import java.util.Objects;


public class SwerveRotateLime extends Command {
    private final Swerve swerve;
    private final Limelight limelight;
    private Pose2d adjustedSpeaker = new Pose2d();
    private final PIDController pidController;
    private Rotation2d wantedRotation;

    public SwerveRotateLime(Swerve swerve, Limelight limelight) {
        this.swerve = swerve;
        this.limelight = limelight;
        pidController = new PIDController(SwerveConstants.AUTO_ROTATE_P, SwerveConstants.AUTO_ROTATE_I, SwerveConstants.AUTO_ROTATE_D);
        addRequirements(this.swerve, this.limelight);
    }

    @Override
    public void initialize() {
        if (DriverStation.getAlliance().isPresent()) {
            Pose2d adjustedSpeaker = FieldConstants.allianceFlipper(new Pose2d(Speaker.centerSpeakerOpening.getX(), Speaker.centerSpeakerOpening.getY(), new Rotation2d()), DriverStation.getAlliance().get());
            double xDiff = adjustedSpeaker.getX() - limelight.getBotPose().getX();
            double yDiff = adjustedSpeaker.getY() - limelight.getBotPose().getY();
            double angle = Math.atan(xDiff / yDiff);
            wantedRotation = Rotation2d.fromRadians(Math.PI / 2 - angle);
            pidController.setSetpoint(wantedRotation.getRadians());
        }
    }

    @Override
    public void execute() {
        swerve.drive(new Translation2d(), pidController.calculate(swerve.getPose().getRotation().getRadians()), true, false);
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