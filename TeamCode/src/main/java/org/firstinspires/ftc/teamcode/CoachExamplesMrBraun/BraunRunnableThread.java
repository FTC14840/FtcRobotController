package org.firstinspires.ftc.teamcode.CoachExamplesMrBraun;

public class BraunRunnableThread implements Runnable {

    BraunMethods robotThread = new BraunMethods();

    @Override
    public void run() {
        // Enter code here to be executed concurrently
        // If the code throws an exception, do the try-catch commented out below

        try {
            robotThread.gyroForward(12,.20,0,1000);
        } catch (InterruptedException e) {
            return;
        }

    }
}
