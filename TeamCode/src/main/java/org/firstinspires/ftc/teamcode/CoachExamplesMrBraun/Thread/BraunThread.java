//package org.firstinspires.ftc.teamcode.CoachExamplesMrBraun.Thread;
//
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//public class BraunThread {
//
//    boolean thread_run=true; /*Set to false to stop the thread i.e. when your opmode is ending */
//    double rpm_gate_time=250; /*How long to wait (mS) between encoder samples - trade off between update rate and precision*/
//    double LRPM,RRPM; /*Left motor RPM, Right motor RPM*/
//    public void init(){
//        /* ..... */
//        tm = new ElapsedTime();
//        new Thread(new Runnable() {
//            @Override
//            public void run() {
//                double sms;
//                while (thread_run) {
//                    /*left and right are dcMotor instances*/
//                    last_left_encoder = left.getCurrentPosition(); /*Get first sample*/
//                    last_right_encoder = right.getCurrentPosition();
//                    sms = tm.milliseconds();
//                    while(tm.milliseconds()-sms < rpm_gate_time){} /*Wait rpm_gate_time mS*/
//                    int delta_l = left.getCurrentPosition() - last_left_encoder; /*Get second sample, subtract first sample to get change*/
//                    int delta_r = right.getCurrentPosition() - last_right_encoder;
//                    double factor = ((1000/rpm_gate_time)*60)/1120; /*Compute factor to convert encoder ticks per gate window to RPM (assumes 1120 ticks/rev)*/
//                    RPM = delta_l * factor; /*Calculate the RPM for the left motor*/
//                    if(Math.abs(RPM) < 400){
//                        /*If we get an absurdly high RPM, the it may be encoder jitter or the encoder was reset mid reading; keep the last reading instead*/
//                        LRPM = -RPM; /*Store the calculated RPM in the LRPM variable*/
//                    }
//                    RPM = delta_r * factor; /*^ Ditto for the right motor*/
//                    if(Math.abs(RPM) < 400){
//                        RRPM = -RPM;
//                    }
//                }
//            }
//        }).start();
//        /* ..... */
//    }
//    public void loop(){
//        /* ..... */
//        telemetry.addData("Motor RPM: ", "%.f, %.f", LRPM, RRPM); //The last measured/computed RPM value will always be available in the LRPM and RRPM global variables
//        telemetry.update();
//        /* ..... */
//    }
//
//}
