package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Tilter;


public class TilterDashboardPosition extends Command {
    private final Tilter tilter;

    public TilterDashboardPosition(Tilter tilter) {
        this.tilter = tilter;
        addRequirements(this.tilter);
        SmartDashboard.putNumber("tilter setpoint", 0);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        tilter.setPosition(SmartDashboard.getNumber("tilter setpoint", 0));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        tilter.stop();
    }
}