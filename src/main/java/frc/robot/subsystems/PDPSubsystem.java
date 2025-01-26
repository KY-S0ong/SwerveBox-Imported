// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PDPSubsystem extends SubsystemBase {
  /** Creates a new PDPSubsystem. */
  private final PowerDistribution pdp = new PowerDistribution(0, ModuleType.kCTRE);
  public PDPSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("PDP voltage", pdp.getVoltage());
    SmartDashboard.putNumber("Rio Volts", RoboRioDataJNI.getVInVoltage());
    SmartDashboard.putNumber("Brown Out Volt", RoboRioDataJNI.getBrownoutVoltage());
    SmartDashboard.putNumber("PDP current", pdp.getTotalCurrent());
    SmartDashboard.putNumber("PDP power", pdp.getTotalPower());
    
    SmartDashboard.putNumber("a motor amps", pdp.getCurrent(15));
    //pdp.getCurrent(2);
  }
}
