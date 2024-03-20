package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flipper;


public class FlipperCheckUp extends Command {
    private final Flipper flipper = Flipper.getInstance();

    public FlipperCheckUp() {
        addRequirements(this.flipper);
    }

    @Override
    public boolean isFinished() {
        return flipper.atTop();
    }
}