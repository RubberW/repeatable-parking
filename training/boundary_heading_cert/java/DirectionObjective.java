/** Simulation-only reward/evaluation; never appended to policy observations.
 * Score the requested heading, independently of the executor's turning arc.
 */
public final class DirectionObjective {
 public static final double SUPPORT_WEIGHT=.002, OUTWARD_WEIGHT=.05;
 public static boolean boundary(CompactSearch m){return m.headingGate.enabled?m.headingGate.active:m.edgeChoice;}
 public static double progress(PanelWorker.Plant p,CompactSearch m,int action){
  double x=p.robot.odometry.preciseX,y=p.robot.odometry.preciseY;
  double a=p.robot.odometry.preciseTheta+m.turn(action);
  return (p.arena.clearance(x+20*Math.cos(a),y+20*Math.sin(a))-p.arena.clearance(x,y))/20;
 }
 public static boolean wrong(PanelWorker.Plant p,CompactSearch m,int action){
  double a=p.robot.odometry.preciseTheta+m.turn(action),x=p.robot.odometry.preciseX,y=p.robot.odometry.preciseY;
  boolean exterior=p.arena.clearance(x,y)<0;
  return p.arena.clearance(x+m.length(action)*Math.cos(a),y+m.length(action)*Math.sin(a))<0 || exterior&&progress(p,m,action)<-1e-6;
 }
 public static double cost(PanelWorker.Plant p,CompactSearch m,int action){
  if(!boundary(m))return 0;
  double inward=progress(p,m,action),outward=Math.max(0,-inward);
  // Tangential choices are allowed. When already outside, weakly prefer re-entry.
  double recovery=p.arena.clearance(p.robot.odometry.preciseX,p.robot.odometry.preciseY)<0?
   .01*Math.pow(Math.max(0,.1-inward)/1.1,2):0;
  return OUTWARD_WEIGHT*outward*outward+recovery;
 }
 public static double outwardProbability(PanelWorker.Plant p,CompactSearch m,double[] scores){
  double top=-Double.MAX_VALUE,total=0,bad=0;for(double s:scores)top=Math.max(top,s);
  for(int a=0;a<scores.length;a++){double w=Math.exp(scores[a]-top);total+=w;if(wrong(p,m,a))bad+=w;}
  return bad/total;
 }
}
