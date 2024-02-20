package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;


public class SwerveAutoRotate extends Command {
    private final Swerve swerve = Swerve.getInstance();
    private Rotation2d rotation2d;
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
        SmartDashboard.putNumber("swerve rotate degrees", 0);
        SmartDashboard.putNumber("swerve rotate P", 0);
        SmartDashboard.putNumber("swerve rotate ff", 0);
    }

    @Override
    public void initialize() {
        pidController.setP(SmartDashboard.getNumber("swerve rotate P", 0));
        rotation2d = Rotation2d.fromDegrees(SmartDashboard.getNumber("get rotate degrees",0));
        pidController.setSetpoint(rotation2d.getRadians());
    }

    @Override
    public void execute() {
        double feedforward = swerve.getPose().getRotation().getRadians() < rotation2d.getRadians() ? SmartDashboard.getNumber("swerve rotate ff", 0) : -SmartDashboard.getNumber("swerve rotate ff", 0);
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