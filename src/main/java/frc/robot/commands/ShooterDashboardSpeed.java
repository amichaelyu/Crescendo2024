package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;


public class ShooterDashboardSpeed extends Command {
    private final Shooter shooter;

    public ShooterDashboardSpeed(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(this.shooter);
        SmartDashboard.putNumber("shooter speed", 0);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        shooter.setSpeed(SmartDashboard.getNumber("shooter speed", 0));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setVoltage(0);
    }
}