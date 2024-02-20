package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.Indexer;

public class IndexerKick extends Command {
    private final Indexer indexer = Indexer.getInstance();
    private final Timer timer;

    public IndexerKick() {
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
        indexer.move(IndexerConstants.INDEXER_DUTY_CYCLE);
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