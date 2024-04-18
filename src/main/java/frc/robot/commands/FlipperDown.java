package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flipper;


public class FlipperDown extends Command {
    private final Flipper flipper = Flipper.getInstance();

    public FlipperDown() {
        addRequirements(this.flipper);
    }

    @Override
    public void execute() {
       flipper.flipDown();
    }

    @Override
    public boolean isFinished() {
        return flipper.atBottom();
    }

    @Override
    public void end(boolean interrupted) {
       flipper.stop();
    }
}