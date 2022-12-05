package frc.robot.subsystems.TestMotors;

import org.littletonrobotics.junction.AutoLog;

public interface LoggedMotor {
    @AutoLog
    public class LoggedMotorIOInputs{
        public double positionRad = 0.0;
        public double velicityRadPerSec = 0.0;
        public double appliedVolts =0.0; 
        public double[] statorAmps = new double[]{}; 
        public double[] statorTempCelcius = new double[]{};

    } 
    public default void updateInputs(LoggedMotorIOInputs inputs) {
    }
  
    /** Run closed loop at the specified velocity. */
    public default void setVelocity(double velocityRadPerSec, double ffVolts) {
    }
  
    /** Stop in open loop. */
    public default void stop() {
    }
  
    /** Set velocity PID constants. */
    public default void configurePID(double kP, double kI, double kD) {
    }
  }


    

