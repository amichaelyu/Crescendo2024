package frc.robot.commands.old;



import frc.robot.subsystems.Intake;


import edu.wpi.first.wpilibj2.command.Command;


public class AutoIntake extends Command {

      private final Intake m_intake;
      private double pwr;
      private double durationMillis;
      private long startTime;
 


  public AutoIntake(Intake intake, double pwr, double seconds) {

      m_intake = intake;
      this.pwr=pwr;
      durationMillis=seconds*1000;
      addRequirements(m_intake);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
    m_intake.move(pwr);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Always return false so the command never ends on it's own. In this project we use the
    // scheduler to end the command when the button is released.
    return System.currentTimeMillis() >startTime+durationMillis;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the wheels when the command ends.`
    if (durationMillis !=  0)
    { 
      m_intake.stop();
    }
  }
}