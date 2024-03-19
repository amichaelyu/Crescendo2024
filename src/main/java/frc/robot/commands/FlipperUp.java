package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flipper;


public class FlipperUp extends Command {
    private final Flipper flipper = Flipper.getInstance();

    public FlipperUp() {
        addRequirements(this.flipper);
    }

    @Override
    public void execute() {
       flipper.flipUp();
    }

    @Override
    public boolean isFinished() {
        return flipper.atTop();
    }

    @Override
    public void end(boolean interrupted) {
       flipper.stop();
    }
}