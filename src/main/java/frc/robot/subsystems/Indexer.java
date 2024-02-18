package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.IndexerConstants;


public class Indexer extends SubsystemBase {
  private final TalonFX m_indexMotor = new TalonFX(IndexerConstants.indexerMotorID);

  /** Creates a new Indexer. */
  public Indexer() {
    m_indexMotor.getConfigurator().apply(new TalonFXConfiguration());

    m_indexMotor.setNeutralMode(NeutralModeValue.Coast);

    m_indexMotor.setInverted(true);
  }


  @Override
  public void periodic() {

  }

  public void move(double pwr) {
    m_indexMotor.set(pwr);
  }


 
  public void stop() {
    m_indexMotor.stopMotor();
  }

 
}