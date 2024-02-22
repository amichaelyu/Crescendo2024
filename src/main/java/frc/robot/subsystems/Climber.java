package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  private final TalonFX rightClimberMotor = new TalonFX(ClimberConstants.rightClimberMotorID);
  private final TalonFX leftClimberMotor = new TalonFX(ClimberConstants.leftClimberMotorID);
  private final DigitalInput limitSwitchRightTop;
  private final DigitalInput limitSwitchRightBottom;

  private static final Climber INSTANCE = new Climber();

  public static Climber getInstance() {
    return INSTANCE;
  }

  /** Creates a new Climber. */
  private Climber() {
    limitSwitchRightTop = new DigitalInput(ClimberConstants.LIMIT_SWITCH_TOP_ID);
    limitSwitchRightBottom = new DigitalInput(ClimberConstants.LIMIT_SWITCH_BOTTOM_ID);

    rightClimberMotor.getConfigurator().apply(ClimberConstants.talonFXConfigs);
    leftClimberMotor.getConfigurator().apply(ClimberConstants.talonFXConfigs);

    rightClimberMotor.setNeutralMode(NeutralModeValue.Brake);
    leftClimberMotor.setNeutralMode(NeutralModeValue.Brake);
//    m_rightClimberMotor.setPosition(0);
//    m_leftClimberMotor.setPosition(0);

    rightClimberMotor.setInverted(true);
    leftClimberMotor.setInverted(true);
  }

  public void periodic() {
    SmartDashboard.putBoolean("right Top", limitSwitchRightTop.get());
    SmartDashboard.putBoolean("right bottom", limitSwitchRightBottom.get());
  }

  public void move(double pwr) {
    if (limitSwitchRightBottom.get() && pwr < 0) {
      pwr = Math.min(pwr, 0);
    }
    else if (limitSwitchRightTop.get() && pwr > 0) {
      pwr = Math.max(pwr, 0);
    }
    rightClimberMotor.set(pwr);
    leftClimberMotor.set(pwr);
  }
 
  public void stop() {
    rightClimberMotor.stopMotor();
  }
}