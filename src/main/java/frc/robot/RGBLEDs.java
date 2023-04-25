// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

/** Add your docs here. */
public class RGBLEDs {
    AddressableLED m_led;
    AddressableLEDBuffer m_ledBuffer;
    int ledLength = 127;

    int currentPurpleNumber = ledLength;
    int currentYellowNumber = ledLength;
    int currentRedNumber = ledLength;
    int currentBlueNumber = ledLength;
    int currentRainbowNumber = ledLength;

    int currentRainbowOffset = 0;

    int currentOffNumber = 0;

    public RGBLEDs(){
        m_led = new AddressableLED(9);
        m_ledBuffer = new AddressableLEDBuffer(ledLength);
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    void setPurple(){
        currentPurpleNumber = 0;
        currentRainbowNumber = ledLength;
    }

    void setYellow(){
        currentYellowNumber = 0;
        currentRainbowNumber = ledLength;
    }

    void setRed(){
        currentRedNumber = 0;
        currentRainbowNumber = ledLength;
    }

    void setBlue(){
        currentBlueNumber = 0;
        currentRainbowNumber = ledLength;
    }

    void setRainbow(){
        currentRainbowNumber = 1;
    }

    void setOff(){
        currentOffNumber = 0;
        currentRainbowNumber = ledLength;
    }

    void setAllOff(){
        int currentPurpleNumber = ledLength;
        int currentYellowNumber = ledLength;
        int currentRedNumber = ledLength;
        int currentBlueNumber = ledLength;
        int currentOffNumber = ledLength;
        int currentRainbowNumber = ledLength;

        for (var i = 0; i < ledLength; i++){
            m_ledBuffer.setRGB(i, 0, 0, 0);
        } 
    }

    void updateState(){
        if(currentPurpleNumber < ledLength){
            m_ledBuffer.setRGB(currentPurpleNumber, 100, 0, 100);
            currentPurpleNumber++;
            SmartDashboard.putNumber("Current LED", currentPurpleNumber);
        }

        if(currentYellowNumber < ledLength){
            m_ledBuffer.setRGB(currentYellowNumber, 150, 100, 0);
            currentYellowNumber++;
            SmartDashboard.putNumber("Current LED", currentYellowNumber);
        }

        if(currentRedNumber < ledLength){
            m_ledBuffer.setRGB(currentRedNumber, 150, 0, 0);
            currentRedNumber++;
            SmartDashboard.putNumber("Current LED", currentRedNumber);
        }

        if(currentBlueNumber < ledLength){
            m_ledBuffer.setRGB(currentBlueNumber, 0, 0, 150);
            currentBlueNumber++;
            SmartDashboard.putNumber("Current LED", currentBlueNumber);
        }

        if(currentOffNumber < ledLength){
            m_ledBuffer.setRGB(currentOffNumber, 0, 0, 0);
            currentOffNumber++;
            SmartDashboard.putNumber("Current LED", currentOffNumber);
        }

        if(currentRainbowNumber < ledLength){
            int red = 0;
            int green = 0;
            int blue = 0;
            int currentID;


            for(int i = 0; i < currentRainbowNumber; i++){
                currentID = (i + currentRainbowOffset) % 48 + 48;

                if(currentID < 16){
                    red = currentID * 10;
                    green = 0;
                    blue = 160 - (currentID * 10);
                }else if(currentID <  32){
                    green = (currentID-16) * 10;
                    blue = 0;
                    red = 160 - ((currentID-16) * 10);
                }else{
                    blue = (currentID-32) * 10;
                    red = 0;
                    green = 160 - ((currentID-32) * 10);
                }

                m_ledBuffer.setRGB(i, red, green, blue);
            }

            if(currentRainbowNumber < ledLength - 1){
                currentRainbowNumber++;
            }

            currentRainbowOffset--;
        }

        m_led.setData(m_ledBuffer);
    }
}
