/**
 * 文件职责：重放完整场景并评测停车；可记录每步触边估计、目标方向和轮子动作。
 * 输入：场景CSV与固定权重；输出：逐场景结果CSV，可选第四参数为诊断轨迹CSV。
 * 调用关系：scripts/verify.py 编译后运行；不参与正常仿真。
 */
import java.io.*;import java.nio.file.*;import java.util.*;import java.lang.reflect.*;
/** Paired complete search and unchanged early_fit parking, no truth controller inputs. */
public final class ParkingEvaluation {
 public static void main(String[] args)throws Exception{
  EvaluationScenes.settings();System.setProperty("parking.policy",args[1]);System.setProperty("parking.localMethod","early_fit");System.setProperty("parking.offline","true");System.setProperty("parking.heuristicScore","false");
  Field field=ParkingCore.Controller.class.getDeclaredField("compact");field.setAccessible(true);
  List<String> lines=Files.readAllLines(Paths.get(args[0]));int n=0;
  try(PrintWriter out=new PrintWriter(Files.newBufferedWriter(Paths.get(args[2]),StandardOpenOption.CREATE_NEW));
      PrintWriter trace=args.length>3?new PrintWriter(Files.newBufferedWriter(Paths.get(args[3]),StandardOpenOption.CREATE_NEW)):null){
   out.println("scene,size,placement,status,first_black_s,total_s,local_s,parked,false_completion,search_full_wheel_s,search_critical_s,search_min_wheel_mm,total_full_wheel_s,total_critical_s,centre_error_mm,heading_error_deg,motion_starts,motion_completed,max_hold_drift_mm,decisions");
   if(trace!=null)trace.println("scene,time_ms,edge,decisions,action,guard,active,phase,alpha_deg,spread_deg,reliable,normal_x,normal_y,goal_x_world,goal_y_world,goal_clearance_mm,x_world,y_world,yaw_world,wheel_clearance_mm,centre_clearance_mm,left_pwm,right_pwm,left_speed,right_speed,estimated_yaw,target_yaw,remembered_lines,edge_preemptions,gate_active,gate_blocked,gate_branch");
   for(String row:lines.subList(1,lines.size())){String[] c=row.split(",");System.setProperty("parking.policySeed",c[10]);SimulationRunner.Plant p=EvaluationScenes.scene(c);
    ParkingCore.Trial t=new ParkingCore.Trial(p,new ParkingCore.MapDescription(),c[1],c[2],new ParkingCore.Calibration(1000));
    double full=0,critical=0,min=0,hold=0;boolean searched=false;CompactSearch m=null;String status="censored_resource_budget";
    for(int k=0;k<2000000&&!t.controller.done();k++){
     t.tick();m=(CompactSearch)field.get(t.controller);if(m!=null){hold=Math.max(hold,m.boundaryMotion.holdMaximum);if(m.decisions>=4096&&t.controller.firstBlackMs<0){status="censored_decision_budget";break;}}
     if(trace!=null&&m!=null&&t.controller.firstBlackMs<0){
      BoundaryMotion b=m.boundaryMotion;double a=Double.parseDouble(c[5]),cs=Math.cos(a),sn=Math.sin(a);
      double gx=Double.parseDouble(c[3])+cs*m.goalX-sn*m.goalY,gy=Double.parseDouble(c[4])+sn*m.goalX+cs*m.goalY;
      trace.printf(Locale.US,"%s,%d,%d,%d,%d,%s,%b,%d,%.5f,%.5f,%b,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.3f,%.3f,%.5f,%.5f,%.5f,%.5f,%d,%d,%b,%b,%d%n",c[1],p.robot.getMillis(),m.edges,m.decisions,m.action,m.guard,b.active,b.phase,b.edge.alpha,b.edge.spread,b.edge.reliable,b.edge.nx,b.edge.ny,gx,gy,p.arena.clearance(gx,gy),p.robot.odometry.preciseX,p.robot.odometry.preciseY,p.robot.odometry.preciseTheta,p.currentWheel,p.risk.centreClearance,t.command.left,t.command.right,p.dynamics.left,p.dynamics.right,m.pose.a,Math.atan2(m.goalY-m.pose.y,m.goalX-m.pose.x),m.memory.lines,m.memory.preemptions,m.headingGate.active,m.headingGate.blocked,m.headingGate.branch);
     }
     if(!searched&&t.controller.firstBlackMs>=0){searched=true;full=p.risk.fullWheelSeconds;critical=p.risk.criticalSeconds;min=p.minWheel;}
     if(t.controller.firstBlackMs>=0&&p.robot.getMillis()-t.controller.firstBlackMs>120000){status="censored_local_evaluation_budget";break;}
     // 只约束离线实验预算；不改变Processing运行的搜索期限。
     if(p.robot.getMillis()>=1000*Double.parseDouble(System.getProperty("parking.evaluationSeconds","20000"))){status="censored_evaluation_time_budget";break;}
    }
    if(t.controller.done())status=t.success?"parked":t.falsePositive?"false_completion":t.controller.failure;
    if(!searched){full=p.risk.fullWheelSeconds;critical=p.risk.criticalSeconds;min=p.minWheel;}
    double first=t.controller.firstBlackMs<0?-1:t.controller.firstBlackMs*.001,total=p.robot.getMillis()*.001;
    out.printf(Locale.US,"%s,%s,%s,%s,%.3f,%.3f,%.3f,%b,%b,%.4f,%.4f,%.4f,%.4f,%.4f,%.5f,%.5f,%d,%d,%.4f,%d%n",c[1],c[0],c[2],status,first,total,first<0?-1:total-first,t.success,t.falsePositive,full,critical,min,p.risk.fullWheelSeconds,p.risk.criticalSeconds,Math.hypot(p.truth().x,p.truth().y),Math.toDegrees(Math.abs(ParkingCore.wrap(p.truth().a-Math.PI/2))),m==null?0:m.boundaryMotion.starts,m==null?0:m.boundaryMotion.completed,hold,m==null?0:m.decisions);out.flush();
    if(++n%8==0)System.out.println(System.getProperty("parking.boundaryMotion")+" "+n+"/"+(lines.size()-1)+" "+status);
   }
  }
 }
}
