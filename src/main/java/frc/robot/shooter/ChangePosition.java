/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.shooter;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class ChangePosition extends SubsystemBase {

  private Compressor airow = new Compressor(20);

  private Solenoid leftPiston = new Solenoid(compressorModule, leftPoseMoverPort);
  private Solenoid rightPiston = new Solenoid(compressorModule, rightPoseMoverPort);

  private boolean posOut = false;

  public ChangePosition() {
    airow.start();
  }

  /**
   * Switch from a shooting position to a collecting position and vice versa.
   */ 
  public void posSwitch() {
    if (posOut) {
      leftPiston.set(false);
      rightPiston.set(false);
      posOut = false;

      System.out.println("Collecting Pose Set");

    } else {
      leftPiston.set(true);
      rightPiston.set(true);
      posOut = true;

      System.out.println("Shooting Pose Set");
    }
  }

  public void collectPose() {
    leftPiston.set(true);
    rightPiston.set(true);

    posOut = true;
    System.out.println("Collecting Pose Set");
  }

  public void shootPose(){
    leftPiston.set(false);
    rightPiston.set(false);

    posOut = false;
    System.out.println("Shooting Pose Set");
  }

  public boolean isPosOut() {
    return posOut;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
