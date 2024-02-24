package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

import java.util.Objects;


public class SwerveRotateLime extends Command {
    private final Swerve swerve = Swerve.getInstance();
    private final Limelight limelight = Limelight.getInstance();
    private final PIDController pidController;
    private Rotation2d wantedRotation = new Rotation2d();

    public SwerveRotateLime() {
        addRequirements(this.swerve);
        pidController = new PIDController(SwerveConstants.ROTATE_P, SwerveConstants.ROTATE_I, SwerveConstants.ROTATE_D);
        pidController.setTolerance(SwerveConstants.ROTATE_TOLERANCE);
        pidController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        if (DriverStation.getAlliance().isPresent() && limelight.hasTarget() && !Objects.equals(limelight.getRotationToTarget(), new Rotation2d())) {
            double limeRot = limelight.getRotationToTarget().getRadians();
            double cramRot = swerve.getPose().getRotation().getRadians() % (2 * Math.PI); //crammed rotation between 0 and 2pi
            if (cramRot < 0) {
                cramRot += Math.PI * 2;
            }
            wantedRotation = Rotation2d.fromRadians(swerve.getPose().getRotation().getRadians() + limeRot - cramRot);
            if (DriverStation.getAlliance().get() == Alliance.Red) {
                wantedRotation = Rotation2d.fromRadians(swerve.getPose().getRotation().getRadians() + cramRot - limeRot);
            }
            if (wantedRotation.getRadians() > Math.PI) {
                wantedRotation = Rotation2d.fromRadians(wantedRotation.getRadians() - 2 * Math.PI);
            }
            else if (wantedRotation.getRadians() < -Math.PI) {
                wantedRotation = Rotation2d.fromRadians(wantedRotation.getRadians() + 2 * Math.PI);
            }
            if (DriverStation.getAlliance().get() == Alliance.Red) {
                wantedRotation = new Rotation2d(-wantedRotation.getCos(), wantedRotation.getSin());
            }
            pidController.setSetpoint(wantedRotation.getRadians());
        }
        else {
            pidController.setSetpoint(swerve.getPose().getRotation().getRadians());
        }
    }

    @Override
    public void execute() {
        double feedforward = swerve.getPose().getRotation().getRadians() < wantedRotation.getRadians() ? SwerveConstants.ROTATE_FF : -SwerveConstants.ROTATE_FF;
        swerve.drive(new Translation2d(), pidController.calculate(swerve.getPose().getRotation().getRadians()) + feedforward, true, false);
    }

    @Override
    public boolean isFinished() {
        return pidController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.driveX();
    }
}