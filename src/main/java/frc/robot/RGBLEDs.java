// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/** Add your docs here. */
public class RGBLEDs {
    AddressableLED m_led;
    AddressableLEDBuffer m_ledBuffer;
    int currentLEDNumber = 0;
    int desiredState = 0;
    void RGBLEDs(){
        m_led = new AddressableLED(0);
        m_ledBuffer = new AddressableLEDBuffer(59);
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
    }
    void setRed(){
        if(desiredState != 1){
            setOff();
            currentLEDNumber = 0;
            desiredState = 1;
        }
    }
    void setPurple(){
        if(desiredState != 2){
            setOff();
            currentLEDNumber = 0;
            desiredState = 2;
        }
    }
    void setYellow(){
        if(desiredState != 3){
            setOff();
            currentLEDNumber = 0;
            desiredState = 3;
        }
    }
    void updateState(){
        if(desiredState == 0){
            setOff();
        }else if(desiredState == 1){
            m_ledBuffer.setRGB(currentLEDNumber, 255, 0, 0);
            if(currentLEDNumber < m_ledBuffer.getLength()){
                currentLEDNumber++;
            }
        }else if(desiredState == 2){
            m_ledBuffer.setRGB(currentLEDNumber, 75, 0, 130);
            if(currentLEDNumber < m_ledBuffer.getLength()){
                currentLEDNumber++;
            }  
        }else if(desiredState == 3){
            m_ledBuffer.setRGB(currentLEDNumber, 255, 234, 0);
            if(currentLEDNumber < m_ledBuffer.getLength()){
                currentLEDNumber++;
            }
        }
        m_led.setData(m_ledBuffer);
    }
    void setOff(){
        for (var i = 0; i < m_ledBuffer.getLength(); i++){
            m_ledBuffer.setRGB(i, 0, 0, 0);
        }
    }
}
