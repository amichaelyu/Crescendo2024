package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;


public class SwerveAutoRotate extends Command {
    private final Swerve swerve = Swerve.getInstance();
    private final Rotation2d rotation2d;
    private final PIDController pidController;

    /**
     * Creates a new SwerveAutoRotate.
     * @param rot wanted {@link Rotation2d} relative to field
     */
    public SwerveAutoRotate(Rotation2d rot) {
        addRequirements(swerve);
        rotation2d = rot;
        pidController = new PIDController(SwerveConstants.ROTATE_P, SwerveConstants.ROTATE_I, SwerveConstants.ROTATE_D);
        pidController.setTolerance(SwerveConstants.ROTATE_TOLERANCE);
//        SmartDashboard.putNumber("swerve rotate P", 0);
    }

    @Override
    public void initialize() {
//        pidController.setP(SmartDashboard.getNumber("swerve rotate P", 0));
        pidController.setSetpoint(rotation2d.getRadians());
    }

    @Override
    public void execute() {
        swerve.drive(new Translation2d(), pidController.calculate(swerve.getPose().getRotation().getRadians()), true, false);
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