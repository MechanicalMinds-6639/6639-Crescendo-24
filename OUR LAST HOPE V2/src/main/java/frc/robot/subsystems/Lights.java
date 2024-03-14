// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Lights extends SubsystemBase {
  /** Creates a new Lights. */
  private final AddressableLED mLed;
  private final AddressableLEDBuffer mLedBuffer;
  private final DigitalInput mSensor;
  //private boolean dashboardOutput;

  public Lights() {
    mLed = new AddressableLED(Constants.Utility.lightPort);
    mLedBuffer = new AddressableLEDBuffer(Constants.Utility.lightNumber);
    mLed.setLength(mLedBuffer.getLength());
    mSensor = new DigitalInput(Constants.Utility.proxSensor);

    mLed.setData(mLedBuffer);
    mLed.start();

    //dashboardOutput = !mSensor.get();
  }

  public void indicateSensor() {

    //System.out.println(!mSensor.get());
    //dashboardOutput = !mSensor.get();

    for (int i = 0; i < mLedBuffer.getLength(); i++) {
      if (!mSensor.get()) {
        mLedBuffer.setRGB(i, 0, 255, 0);
      } else {
        mLedBuffer.setRGB(i, 255, 0, 0);
      }
    }
    mLed.setData(mLedBuffer);
  }

  public boolean getSensorValue() {
    return !mSensor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
