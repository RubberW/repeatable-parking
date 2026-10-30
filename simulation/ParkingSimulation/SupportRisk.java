/**
 * 文件职责：用模拟真实几何独立统计轮子支撑和严重越界
 * 输入：模拟轮心、车中心与边界；输出：越界时长、次数和代价。
 * 调用关系：仅 SimulationRunner.Plant 评估使用，不进入模型输入。
 */
/** Training/evaluation geometry only. Never supplied to policy observations.
 * Wheel support is a configurable 5mm-radius disc proxy, not a measured tyre.
 * Centre of mass is assumed at axle midpoint: crossing puts half a symmetric body outside a straight edge.
 */
public final class SupportRisk {
 public static final double RADIUS=Double.parseDouble(System.getProperty("parking.supportRadiusMm","5"));
 public double leftFraction,rightFraction,centreClearance,rate,totalCost=0,partialSeconds=0,fullWheelSeconds=0,criticalSeconds=0;
 public int criticalEvents=0;public boolean critical=false;boolean charged=false;double clearSeconds=0;
 static double fraction(double clearance){return ParkingCore.clip((RADIUS-clearance)/(2*RADIUS),0,1);}
 public void update(double left,double right,double centre,double dt){
  leftFraction=fraction(left);rightFraction=fraction(right);centreClearance=centre;
  int full=(left<=-RADIUS?1:0)+(right<=-RADIUS?1:0);
  critical=centre<0||(left<0&&right<0);
  // Count once per severe excursion; re-arm only after 0.3s fully supported with 2mm reserve.
  if(left>=RADIUS+2&&right>=RADIUS+2&&centre>=2){clearSeconds+=dt;if(clearSeconds>=.3)charged=false;}else clearSeconds=0;
  if(critical&&!charged){criticalEvents++;totalCost+=1;charged=true;}
  rate=.25*(leftFraction*leftFraction+rightFraction*rightFraction)+.75*full;
  if(critical)rate+=2*(1+Math.min(2,Math.max(0,-centre)/50));
  totalCost+=rate*dt;
  if(leftFraction>0||rightFraction>0)partialSeconds+=dt;
  if(full>0)fullWheelSeconds+=dt;
  if(critical)criticalSeconds+=dt;
 }
 public void observe(SimulationRunner.Plant p,double dt){
  double x=p.robot.odometry.preciseX,y=p.robot.odometry.preciseY,a=p.robot.odometry.preciseTheta;
  double sx=45*Math.sin(a),sy=45*Math.cos(a);
  update(p.arena.clearance(x-sx,y+sy),p.arena.clearance(x+sx,y-sy),p.arena.clearance(x,y),dt);
 }
}
