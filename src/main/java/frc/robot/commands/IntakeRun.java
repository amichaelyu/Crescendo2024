package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;


public class IntakeRun extends Command {
    private final Intake intake = Intake.getInstance();

    public IntakeRun() {
        addRequirements(this.intake);
    }

    @Override
    public void execute() {
        intake.forward();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}