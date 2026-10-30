/**
 * 文件职责：电机惯性、加速、制动与反转的仿真响应
 * 输入：请求的左右轮 PWM；输出：实际模拟轮速。
 * 调用关系：SimulationRunner.Plant 每个10毫秒步长调用。
 */
/** Assumed actuator response, applied before the differential-drive kinematics.
 * Not a measured motor fit. Speeds in mm/s, time in seconds. */
public final class WheelDynamics {
 public final double tau=Double.parseDouble(System.getProperty("smooth.tau",".12"));
 public final double acceleration=Double.parseDouble(System.getProperty("smooth.accel","500"));
 public final double braking=Double.parseDouble(System.getProperty("smooth.brake","800"));
 public double left=0,right=0;
 public void reset(){left=right=0;}
 double step(double current,double desired){
  boolean reversing=current*desired<0;
  double limit=reversing||Math.abs(desired)<Math.abs(current)?braking:acceleration;
  double delta=(desired-current)*(1-Math.exp(-.01/tau));
  double next=current+ParkingCore.clip(delta,-limit*.01,limit*.01);
  if(reversing && current*next<0)next=0;
  if(Math.abs(next)<.5 && desired==0)next=0;
  return next;
 }
 public ParkingCore.Command apply(ParkingCore.Command requested){
  left=step(left,requested.left*1.25);right=step(right,requested.right*1.25);
  return new ParkingCore.Command(left/1.25,right/1.25);
 }
}
