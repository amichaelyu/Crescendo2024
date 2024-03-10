package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;


public class SwerveRotateLime extends Command {
    private final Swerve swerve = Swerve.getInstance();
//    private final Limelight limelight = Limelight.getInstance();
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
//        if (DriverStation.getAlliance().isPresent() && limelight.hasTarget() && !Objects.equals(limelight.getRotationToTarget(), new Rotation2d())) {
//            Rotation2d limeRot = limelight.getRotationToTarget();
//            limeRot = limeRot.rotateBy(Rotation2d.fromRadians(Math.PI));
//            wantedRotation = limeRot;
//            pidController.setSetpoint(wantedRotation.getRadians());
//        }
//        else {
//            pidController.setSetpoint(swerve.getPose().getRotation().getRadians());
//        }
        if (DriverStation.getAlliance().isPresent()) {
            Rotation2d limeRot = swerve.getRotationToTargetSwervePose();
            limeRot = limeRot.rotateBy(Rotation2d.fromRadians(Math.PI));
            wantedRotation = limeRot;
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