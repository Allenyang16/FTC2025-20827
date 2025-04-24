//package org.firstinspires.ftc.teamcode.actions.actioncore;
//
//import org.firstinspires.ftc.teamcode.uppersystems.SuperStructure;
//
//public class ParallelActionGroup extends Action {
////    private int toleranceRange = 100;
////    private SuperStructure upper;
//    //Params not in super class
//    private Action[] actions;
//
//    public ParallelActionGroup(Action...actions){
//        this.actions = actions;
//    }
//
//    public int getError() {
//        int sum = 0;
//        for(Action a:actions){
//            sum += a.getError();
//        }
//        return sum/actions.length;
//    }
//
//    public boolean canStartNext(){
//        boolean canStart = true;
//        for(Action a:actions){
//            canStart = a.canStartNext() && canStart;
//        }
//        return canStart;
//    }
//
//    public boolean isFinished(){
//        boolean canFinish = true;
//        for(Action a:actions){
//            canFinish = a.isFinished() && canFinish;
//        }
//        return canFinish;
//    }
//
//    public void actuate() {
//        for(Action a:actions){
//            a.actuate();
//        }
//    }
//
//    public void stop(){
//        for(Action a:actions){
//            a.stop();
//        }
//    }
//
//    public void forceStop(){
//        for(Action a:actions){
//            a.forceStop();
//        }
//    }
//
//    public String returnType(){
//        return "ParallelAction";
//    }
//
//}