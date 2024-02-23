package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;


public class SwerveLimePose extends Command {
    private final Limelight limelight = Limelight.getInstance();
    private final Swerve swerve = Swerve.getInstance();

    public SwerveLimePose() {
        addRequirements(this.limelight, this.swerve);
    }

    @Override
    public void initialize() {
        swerve.setPose(limelight.getBotPose());
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {

    }
}