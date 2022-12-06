package frc.robot.subsystems.TestBoard;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.TestMotors.LoggedMotor;
import frc.robot.subsystems.TestMotors.LoggedMotorIOInputsAutoLogged;
import frc.robot.subsystems.TestMotors.LoggedMotor.LoggedMotorIOInputs;
import org.littletonrobotics.junction.AutoLog;

public class DualMotor extends SubsystemBase{
    private final LoggedMotor motor1;
    private final LoggedMotor motor2; 
    private final LoggedMotorIOInputsAutoLogged input1 = new LoggedMotorIOInputsAutoLogged();
    private final LoggedMotorIOInputsAutoLogged input2 = new LoggedMotorIOInputsAutoLogged();
    private final SimpleMotorFeedforward ffModel;

    public DualMotor(LoggedMotor motor1, LoggedMotor motor2){
        this.motor1 = motor1;
        this.motor2 = motor2;
        switch (Constants.currentMode) {
            case REAL:
            case REPLAY:
              ffModel = new SimpleMotorFeedforward(0.1, 0.05);
              motor1.configurePID(1.0, 0.0, 0.0);
              motor2.configurePID(1.0, 0, 0);
              break;
            case SIM:
              ffModel = new SimpleMotorFeedforward(0.1, 0.05);
              motor1.configurePID(0.5, 0.0, 0.0);
              motor2.configurePID(0.5, 0.0, 0.0);
              break;
            default:
              ffModel = new SimpleMotorFeedforward(0.0, 0.0);
              break;
          }
    }
    @Override
  public void periodic() {
    motor1.updateInputs(input1);
    Logger.getInstance().processInputs("Motor1", input1);
    motor1.updateInputs(input2);
    Logger.getInstance().processInputs("Motor1", input2);
  }
  public void runVelocity(double velocityRPM) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    motor1.setVelocity(velocityRadPerSec, ffModel.calculate(velocityRadPerSec));
    motor2.setVelocity(velocityRadPerSec, ffModel.calculate(velocityRadPerSec));

    // Log flywheel setpoint
    Logger.getInstance().recordOutput("FlywheelSetpointRPM", velocityRPM);
  }
  public void stop() {
    motor1.stop();
    motor2.stop();
  }
  public double getVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(input1.velocityRadPerSec);
  } 

}
