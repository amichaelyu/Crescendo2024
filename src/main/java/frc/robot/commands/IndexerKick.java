package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;

public class IndexerKick extends Command {
    private final Indexer indexer;
    private final Timer timer;

    public IndexerKick(Indexer indexer) {
        this.indexer = indexer;
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
        indexer.move(1.0);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(1);
    }

    @Override
    public void end(boolean interrupted) {
        indexer.move(0);
    }
}