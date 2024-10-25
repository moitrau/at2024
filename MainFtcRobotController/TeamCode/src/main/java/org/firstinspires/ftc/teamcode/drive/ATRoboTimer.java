package org.firstinspires.ftc.teamcode.drive;

/*
Enums used by AT24 - Into The Deep
 */

public class ATRoboTimer {

    public double startTime = 0.0;
    public double stopTime = 0.0;
    public boolean isActive = false ;
    public void startTimer(){
        if(isActive == false){
            startTime = System.currentTimeMillis();
            isActive = true;
        }

    }
    public void stopTimer(){
        stopTime = System.currentTimeMillis();
        isActive = false;
    }
    public double elapsedTime() {
        if(isActive == true){
            long now = System.currentTimeMillis();
            return ((double) now - startTime) / 1000.0;
        }else {
            return ((double) stopTime - startTime) / 1000.0;
        }
    }
}
