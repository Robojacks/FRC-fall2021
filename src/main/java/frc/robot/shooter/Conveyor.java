/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.shooter;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class Conveyor extends SubsystemBase {
  
  private WPI_TalonSRX conveyor = new WPI_TalonSRX(kConveyorPort);

  private ChangePosition goalMover;
  private Shooter m_shooter;

  public Conveyor(ChangePosition changePosition, Shooter shooter) {
    goalMover = changePosition;
    m_shooter = shooter;
  }

  public void shoot(double conveyorVolts) {
    conveyor.setVoltage(conveyorVolts);
  }

  public void collect(double conveyorVolts) {
    conveyor.setVoltage(-conveyorVolts);
  }

  public void stop() {
    setSpeed(0);
  }
  public void setSpeed(double conveyorVolts) {
    if (goalMover.isPosOut()) {
      collect(conveyorVolts);

    } else {
      shoot(conveyorVolts);
    }
  }

  public void toggleSpeed(double conveyorVolts) {
    if (m_shooter.isEngaged()) {
      setSpeed(conveyorVolts);

    } else {
      stop();
    }
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}