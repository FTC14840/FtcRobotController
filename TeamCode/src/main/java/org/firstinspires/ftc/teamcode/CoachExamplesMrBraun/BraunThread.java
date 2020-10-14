package org.firstinspires.ftc.teamcode.CoachExamplesMrBraun;

public class BraunThread extends Thread {

    BraunMethods robotThread = new BraunMethods();

    @Override
    public void run() {
        super.run();
        // Enter code here to be executed concurrently
        // If the code throws an exception, do the try-catch commented out below

//        try {
//            robotThread.gyroForward(12,.20,0,1000);
//        } catch (InterruptedException e) {
//            e.printStackTrace();
//        }

    }
}
