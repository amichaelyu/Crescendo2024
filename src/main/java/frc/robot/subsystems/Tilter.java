package frc.robot.subsystems;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;

public class Tilter extends SubsystemBase {
  private final TalonFX m_tilterMotor = new TalonFX(tilterMotorID);
  private DigitalInput limitSwitchTop;
	private DigitalInput limitSwitchBottom;

  /** Creates a new Tilter. */
  public Tilter() {
    
    limitSwitchBottom = new DigitalInput(kLIFTER_LIMIT_BOTTOM);
    m_tilterMotor.getConfigurator().apply(new TalonFXConfiguration());

    m_tilterMotor.setInverted(true);
    
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

 
