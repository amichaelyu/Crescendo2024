import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.TilterConstants;

public class Tilter extends SubsystemBase {
  private final TalonFX m_tilterMotor = new TalonFX(TilterConstants.tilterMotorID);
  private boolean isHomed;
  private DigitalInput limitSwitchTop;
	private DigitalInput limitSwitchBottom;
  
  /** Creates a new Tilter. */
  public Tilter() {
    limitSwitchBottom = new DigitalInput(kLIFTER_LIMIT_BOTTOM);
    
    m_tilterMotor.getConfigurator().apply(TilterConstants.talonFXConfigs);
    m_tilterMotor.setNeutralMode(NeutralModeValue.Brake);
    m_tilterMotor.setInverted(true);
    isHomed = false;
  }

  public void home() {
    m_tilterMotor.setControl(new VoltageOut(-1));
  }

  public void setPosition(double position) {
    if (isHomed) {
      m_tilterMotor.setControl(new MotionMagicVoltage(position));
    }
  }

  public void move(double pwr) {
    if (isAtBottom() && pwr > 0) {
			m_tilterMotor.set(0);
			System.out.println("Lifter at Bottom; not going down.");
		}
    else{
    m_tilterMotor.set(pwr);
    }
  }


 
  public void stop() {
    m_tilterMotor.stopMotor();
  }
	public boolean isAtBottom() {
		return !limitSwitchBottom.get();
    }
  }

 
