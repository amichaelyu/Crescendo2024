package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.controller.BetterXboxController;
import frc.lib.controller.BetterXboxController.Humans;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;


public class SwerveRotateLimeOnTheFly extends Command {
    private final Swerve swerve = Swerve.getInstance();
    private final PIDController pidController;
    private Rotation2d wantedRotation = new Rotation2d();

    public SwerveRotateLimeOnTheFly() {
        addRequirements(this.swerve);
        pidController = new PIDController(SwerveConstants.ROTATE_P, SwerveConstants.ROTATE_I, SwerveConstants.ROTATE_D);
        pidController.setTolerance(SwerveConstants.ROTATE_TOLERANCE);
        pidController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (DriverStation.getAlliance().isPresent()) {
            wantedRotation = swerve.getRotationToTargetSwervePose().rotateBy(Rotation2d.fromRadians(Math.PI));
            pidController.setSetpoint(wantedRotation.getRadians());
        }
        else {
            pidController.setSetpoint(swerve.getPose().getRotation().getRadians());
        }
        int shouldNegate = 1;
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == DriverStation.Alliance.Red) {
                shouldNegate = -1;
            }
            else if (alliance.get() == DriverStation.Alliance.Blue) {
                shouldNegate = 1;
            }
        }
        double feedforward = swerve.getPose().getRotation().getRadians() < wantedRotation.getRadians() ? SwerveConstants.ROTATE_FF : -SwerveConstants.ROTATE_FF;
        swerve.drive(BetterXboxController.getController(Humans.DRIVER).getSwerveTranslation().times(SwerveConstants.maxSpeed * shouldNegate), pidController.calculate(swerve.getPose().getRotation().getRadians()) + feedforward, true, false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
    }
}