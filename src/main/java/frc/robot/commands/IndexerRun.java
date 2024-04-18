package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.Indexer;


public class IndexerRun extends Command {
    private final Indexer indexer = Indexer.getInstance();
//    private final Vector<Double> indexerCurrents = new Vector<>();
//    private final int currentCycles = IndexerConstants.CURRENT_CYCLES;

    public IndexerRun() {
        addRequirements(this.indexer);
    }

    @Override
    public void initialize() {
//        for (int i = 0; i < currentCycles; i++) indexerCurrents.add(0.0);
    }

    @Override
    public void execute() {
        indexer.move(IndexerConstants.FORWARD_DUTY_CYCLE);
//        indexerCurrents.remove(0);
//        indexerCurrents.add(indexer.getTorqueCurrent());
    }

    @Override
    public boolean isFinished() {
//        double sum = 0;
//        for (int i = 0; i < currentCycles; i++) {
//            sum += indexerCurrents.get(i);
//        }
//        double avg = sum / currentCycles;
//        return Math.abs(avg) > IndexerConstants.CUTOFF_VOLTAGE;
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}