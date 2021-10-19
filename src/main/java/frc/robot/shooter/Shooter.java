/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.shooter;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.vision.Limelight;

import static frc.robot.Gains.shooterPID;

import java.util.function.BooleanSupplier;

import static frc.robot.Constants.*;

public class Shooter extends SubsystemBase {
  private ChangePosition goalMover;

  private Limelight vision;

  private CANSparkMax launcher = new CANSparkMax(kShooterPort, MotorType.kBrushless);

  private CANEncoder launcherEncoder = launcher.getEncoder();
  
  private CANPIDController launcherController = launcher.getPIDController();

  // Create toggle for shooting
  private boolean engaged = false;

  public Shooter(ChangePosition changePosition, Limelight limelight) {
    // Makes changePosition instance the same as in RobotContainer
    goalMover = changePosition;
    vision = limelight;
    // Spark PID Stuff
    launcherController.setP(shooterPID.kP);
    launcherController.setI(shooterPID.kI);
    launcherController.setD(shooterPID.kD); 
    launcherController.setFF(shooterPID.kF);

    //launcherEncoder.setVelocityConversionFactor(factor)
  }


  public void collect() {
    launcher.setVoltage(intakeVolts);
    engaged = true;
  }

  public void shoot() {
    launcher.setVoltage(shooterVolts);
    engaged = true;
  }

  public void stop() {
    launcher.setVoltage(0);
    launcherController.setReference(0, ControlType.kVoltage);

    engaged = false;
  }

  /**
   * Sets a voltage based on whether the robot is in shooting position or
   * intake position.
   */
  public void setSpeedVolts() {
    if (goalMover.isCollectingPose()) {
      // if in intake position, intake
      collect();

    } else {
      shoot();

    }
  }

  /**
   * Toggles shooter on and off with the specified voltage
   */
  public void toggleSpeedVolts() {
    if (engaged) {
      stop();

    } else {
      setSpeedVolts();
    }
  }

  /**
   * Sets RPM based on whether the robot is in shooting position or 
   * intake position.
   */
  public void setSpeedSpark() {
    if (goalMover.isCollectingPose()) {
      launcherController.setReference(intakeRPM, ControlType.kVelocity);

    } else {
      launcherController.setReference(shooterRPM, ControlType.kVelocity);

    }

    engaged = true;
  }

  public void toggleSpeedSpark() {
    if (engaged) {
      stop();

    } else {
      setSpeedSpark();

    }
  }

  private double calculateRPM2(double distanceMeters) {
    // Get the velocity needed to shoot the ball
    double distance = Units.metersToFeet(distanceMeters);
    double top = Math.sqrt(4.9 * Math.pow(distance, 2)); 
    double bottom = Math.sqrt(Math.pow(Math.cos(Math.toRadians(shooterAngle)), 2) *
     (highGoalToShooterHeight - Math.tan(shooterAngle) * distance));

    double metersPerSecond = Units.feetToMeters(top/bottom);

    double radiansPerSecond = metersPerSecond / kShooterWheelRadiusMeters;

    return Units.radiansPerSecondToRotationsPerMinute(radiansPerSecond);
  }

  /**
   * The Quadratic Formula, using a plus after B
   * @param A squared variable value 
   * @param B multiplied by variable
   * @param C constant
   * @return "x =" value (velocity in m/s for the calculateRPM2() method)
   */
  private double QuadraticFormulaPlus(double A, double B, double C) {
    double top = -B + Math.sqrt(Math.pow(B, 2)-(4*A*C));
    double bottom = 2*A;

    return top/bottom;
  }

  /**
   * The Quadratic Formula, using a minus after B
   * @param A squared variable value 
   * @param B multiplied by variable
   * @param C constant
   * @return "x =" value (velocity in m/s for the calculateRPM2() method)
   */
  private double QuadraticFormulaMinus(double A, double B, double C) {
    double top = -B - Math.sqrt(Math.pow(B, 2)-(4*A*C));
    double bottom = 2*A;
    
    return top/bottom;
  }

  /**
   * Using a physics model and input distance, converts the output of meters per second
   * to RPM, which can then be output by the shooter
   * @param distance distance from the target
   * @return Rotations Per Minute (RPM) required to shoot a ball into the goal
   */
  public double calculateRPM(double distance) {
    double A = -Math.sin(Math.toRadians(shooterAngle))/distance;
    double B = ((centerOfBallTargetHeight- centerofBallShooterHeight)*Math.cos(Math.toRadians(shooterAngle)))
              /(Math.pow(distance, 2));
    double C = gravity;

    // puts found values into quadratic formula, finds exit velocity in m/s
    double velocityPlus = QuadraticFormulaPlus(A, B, C);
    double velocityMinus = QuadraticFormulaMinus(A, B, C);

    // converts m/s velocity to rads/sec to RPM
    double RPMPlus = Units.radiansPerSecondToRotationsPerMinute(velocityPlus / kShooterWheelRadiusMeters);
    double RPMMinus = Units.radiansPerSecondToRotationsPerMinute(velocityMinus / kShooterWheelRadiusMeters);
    
    if (RPMPlus < RPMMinus && RPMPlus > 0) { // If the RPM + is less than - and positive, use it for a lower arc
      return RPMPlus;

    } else if (RPMMinus < RPMPlus && RPMMinus > 0) { // If the RPM - is less than + and positive, use it for a lower arc
      return RPMMinus;

    } else if (RPMPlus == RPMMinus && RPMMinus > 0) { // If they are the same value (and positive), return that value
      return RPMMinus;

    } else if (RPMMinus > 0) { // Return RPMMinus if positive, this is here in case RPMPlus is negative
      return RPMMinus;

    } else if (RPMPlus > 0) { // Return RPMPlus if positive, this is here in case RPMMinus is negative
      return RPMPlus;

    } else {
      return 0; // something went wrong, let us not break things. It could be that RPMMinus and RPMPlus are both negative
    }

  }

  /**
   * Sets RPM based on whether the robot is in shooting position or intake position. 
   * If it is in shooting position, calculates the RPM needed for the shooter to 
   * hit the target.
   * @param inRPM the velocity in RPM the shooter goes to while in the intake position
   * @param distance the distance in meters from the target
   */
  public void setRelativeSpeedSpark(double distance){
    if (goalMover.isCollectingPose()) {
      launcherController.setReference(intakeRPM, ControlType.kVelocity);
      engaged = true;

    } else {
      launcherController.setReference(calculateRPM(distance), ControlType.kVelocity);
      engaged = true;

    }
  }

  public void toggleRelativeSpeedSpark(double distance) {
    if (engaged) {
      stop();
      
      vision.driverMode();
      vision.lightOff();

    } else {
      setRelativeSpeedSpark(distance);

      vision.visionMode();
      vision.lightOn();
    }
  }

  public boolean isEngaged() {
    return engaged;
  }

  public boolean isDisabled() {
    return !engaged;
  }

  /**
   * Gets the shooter velocity in RPM
   */
  public double getVelocity() {
    return launcherEncoder.getVelocity();
  }

  public boolean atSpeed() {
    return shooterRPM <= getVelocity();
  }
  
  public boolean atAutonomousSpeed() {
    return shooterAutonomousRPM <= getVelocity();
  }

  @Override
  public void periodic() {
    //System.out.println("" + calculateRPM(3.8));
  }
}
