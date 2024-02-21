package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;


public class IndexerSlightBack extends Command {
    private final Indexer indexer = Indexer.getInstance();
    private final Timer timer;

    public IndexerSlightBack() {
        addRequirements(this.indexer);
        timer = new Timer();
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        indexer.move(0.25);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(0.1);
    }

    @Override
    public void end(boolean interrupted) {
        indexer.stop();
    }
}