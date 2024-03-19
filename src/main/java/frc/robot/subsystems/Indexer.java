package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.IndexerConstants;


public class Indexer extends SubsystemBase {
  private final TalonFX m_indexMotor = new TalonFX(IndexerConstants.ID, Constants.CAN_BUS_NAME);

  private static final Indexer INSTANCE = new Indexer();

  public static Indexer getInstance() {
    return INSTANCE;
  }

  /** Creates a new Indexer. */
  private Indexer() {
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

  public double getTorqueCurrent() {
    return m_indexMotor.getTorqueCurrent().getValue();
  }
 
  public void stop() {
    m_indexMotor.setControl(new VoltageOut(0));
  }
}