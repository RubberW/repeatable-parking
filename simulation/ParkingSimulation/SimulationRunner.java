/**
 * 文件职责：仿真环境、停车标记、10毫秒推进和回放；趣味模式另支持多边形与墙线。
 * 输入：场景配置、左右轮指令和推进步数；输出：模拟传感器、编码器、画面帧和结果摘要。
 * 调用关系：Processing 主窗口调用；独立评测只使用其中的 Plant。
 */
import java.io.*;import java.nio.file.*;import java.util.*;import java.util.zip.*;import processing.core.*;import processing.data.*;

/** One independent simulation state per run. Only sensors/encoders reach the controller. */
public final class SimulationRunner {
 static class Arena {
  // 几何仅供环境、碰撞与显示使用；控制器仍只收到传感器和编码器。
  final String shape;final double w,h;final double[][] polygon;final double[][][] walls;final boolean custom;
  static final double WALL_HALF=5,BODY_RADIUS=50,SENSOR_BAND=15;
  Arena(JSONObject c){
   shape=c.getString("shape");w=c.getDouble("width_mm");h=c.getDouble("height_mm");
   if(!Double.isFinite(w)||!Double.isFinite(h)||w<=0||h<=0)throw new IllegalArgumentException("地图尺寸无效");
   polygon=points(c.getJSONArray("polygon"));
   JSONArray lines=c.hasKey("walls")?c.getJSONArray("walls"):new JSONArray();walls=new double[lines.size()][][];
   if(lines.size()>64)throw new IllegalArgumentException("最多64道墙");
   for(int i=0;i<walls.length;i++){walls[i]=points(lines.getJSONArray(i));if(walls[i].length<2)throw new IllegalArgumentException("墙线至少需要两个点");}
   if(!shape.equals("rectangle")&&!shape.equals("polygon"))throw new IllegalArgumentException("地图形状无效");
   if(shape.equals("polygon"))validatePolygon(polygon);
   for(double[][] line:walls)for(double[] p:line)if(outerClearance(p[0],p[1])<-.001)throw new IllegalArgumentException("墙线必须画在外边界以内");
   // 即便一条墙线的端点均在凹形内部，中间也可能穿出；沿整条线检查。
   for(double[][] line:walls)for(int i=1;i<line.length;i++){
    double[] a=line[i-1],b=line[i];int n=Math.max(1,(int)Math.ceil(Math.hypot(b[0]-a[0],b[1]-a[1])/5));
    for(int j=1;j<n;j++)if(outerClearance(a[0]+(b[0]-a[0])*j/n,a[1]+(b[1]-a[1])*j/n)<-.001)throw new IllegalArgumentException("墙线穿出了外边界，请撤销后重画");
   }
   custom=c.getBoolean("custom_map",false)||shape.equals("polygon")||walls.length>0;
  }
  static double[][] points(JSONArray a){
   if(a==null)return new double[0][2];if(a.size()>512)throw new IllegalArgumentException("一条线最多512个点");
   double[][] p=new double[a.size()][2];for(int i=0;i<p.length;i++){JSONArray q=a.getJSONArray(i);p[i][0]=q.getDouble(0);p[i][1]=q.getDouble(1);if(!Double.isFinite(p[i][0])||!Double.isFinite(p[i][1]))throw new IllegalArgumentException("地图坐标无效");}return p;
  }
  static double segmentDistance(double x,double y,double[] a,double[] b){
   double dx=b[0]-a[0],dy=b[1]-a[1],n=dx*dx+dy*dy,t=n==0?0:ParkingCore.clip(((x-a[0])*dx+(y-a[1])*dy)/n,0,1);
   return Math.hypot(x-a[0]-t*dx,y-a[1]-t*dy);
  }
  static double cross(double[] a,double[] b,double[] c){return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]);}
  static boolean intersects(double[] a,double[] b,double[] c,double[] d){
   double p=cross(a,b,c),q=cross(a,b,d),r=cross(c,d,a),s=cross(c,d,b);
   if(p*q<0&&r*s<0)return true;
   return Math.abs(p)<1e-7&&segmentDistance(c[0],c[1],a,b)<1e-7||Math.abs(q)<1e-7&&segmentDistance(d[0],d[1],a,b)<1e-7||Math.abs(r)<1e-7&&segmentDistance(a[0],a[1],c,d)<1e-7||Math.abs(s)<1e-7&&segmentDistance(b[0],b[1],c,d)<1e-7;
  }
  static void validatePolygon(double[][] p){
   if(p.length<3)throw new IllegalArgumentException("外边界至少需要三个点，或使用圆形/椭圆工具");double area=0;
   for(int i=0;i<p.length;i++){double[] a=p[i],b=p[(i+1)%p.length];if(Math.hypot(a[0]-b[0],a[1]-b[1])<.001)throw new IllegalArgumentException("外边界有重复点，请重画");area+=a[0]*b[1]-b[0]*a[1];
    for(int j=i+1;j<p.length;j++)if(j!=i+1&&!(i==0&&j==p.length-1)&&intersects(a,b,p[j],p[(j+1)%p.length]))throw new IllegalArgumentException("外边界不能自交，请撤销后重画");
   }if(Math.abs(area)<20000)throw new IllegalArgumentException("外边界太小，请画大一点");
  }
  double outerClearance(double x,double y){
   if(shape.equals("rectangle"))return Math.min(w/2-Math.abs(x),h/2-Math.abs(y));
   boolean inside=false;double distance=Double.POSITIVE_INFINITY;
   for(int i=0,j=polygon.length-1;i<polygon.length;j=i++){
    double[] a=polygon[j],b=polygon[i];distance=Math.min(distance,segmentDistance(x,y,a,b));
    if((a[1]>y)!=(b[1]>y)&&x<(b[0]-a[0])*(y-a[1])/(b[1]-a[1])+a[0])inside=!inside;
   }return inside?distance:-distance;
  }
  double clearance(double x,double y){
   double distance=outerClearance(x,y);
   for(double[][] line:walls)for(int i=1;i<line.length;i++)distance=Math.min(distance,segmentDistance(x,y,line[i-1],line[i])-WALL_HALF);
   return distance;
  }
  // 只有趣味模式使用不可穿透碰撞；普通矩形保持原来的虚拟桌沿行为。
  double motionFraction(double x,double y,double yaw,float left,float right){
   double ds=((double)left+right)/2,dx=ds*Math.cos(yaw),dy=ds*Math.sin(yaw);
   if(Math.abs(ds)<1e-10||clearance(x+dx,y+dy)>=BODY_RADIUS-.001)return 1;
   double lo=0,hi=1;for(int i=0;i<24;i++){double mid=(lo+hi)/2;if(clearance(x+mid*dx,y+mid*dy)>=BODY_RADIUS)lo=mid;else hi=mid;}return lo;
  }
 }
 static class Surface extends PImage {
  final ParkingCore.MapDescription map=new ParkingCore.MapDescription();final double tx,ty,cs,sn;
  Surface(JSONObject c){width=height=1000000;format=PConstants.RGB;tx=c.getDouble("target_x");ty=c.getDouble("target_y");double a=c.getDouble("target_yaw");cs=Math.cos(a);sn=Math.sin(a);}
  @Override public int get(int px,int py){double x=px-width/2+.5-tx,y=height/2-py-.5-ty,u=cs*x+sn*y,v=-sn*x+cs*y;return Math.abs(u)<=77&&Math.abs(v)<=92&&map.black(u,v)?0xff000000:0xffffffff;}
 }
 static class Plant implements ParkingCore.Plant {
  final ParkingSimulation.RobotSimulation_c robot;final WheelDynamics dynamics=new WheelDynamics();final SupportRisk risk=new SupportRisk();final Arena arena;final double tx,ty,ta,margin;final String stopMode;
  boolean lost=false;long firstCross=-1;double minWheel=1e30,currentWheel=1e30;int crossings=0,collisions=0;boolean outside=false,colliding=false;
  Plant(JSONObject c){arena=new Arena(c);tx=c.getDouble("target_x");ty=c.getDouble("target_y");ta=c.getDouble("target_yaw");margin=c.getDouble("margin_mm");stopMode=c.getString("stop_mode");ParkingSimulation a=new ParkingSimulation();a.g=new PGraphics();a.g.colorMode(PConstants.RGB,255);robot=a.new RobotSimulation_c(16,90,new Surface(c),0,0);Random r=new Random(c.getLong("encoder_seed"));robot.resetForRun((float)c.getDouble("robot_x"),(float)c.getDouble("robot_y"),(float)c.getDouble("robot_yaw"),r.nextInt(18000)-9000,r.nextInt(18000)-9000);}
  public ParkingCore.Observation observe(){robot.getSurfaceSensors();float[] v=robot.sensorReadings().clone();ParkingSimulation.RobotPose_c q=robot.odometry.pose;for(int k=0;k<5;k++){ParkingSimulation.SurfaceSensorModel_c s=robot.sensors[k];double x=q.x+s.localX*Math.cos(q.theta)-s.localY*Math.sin(q.theta),y=q.y+s.localX*Math.sin(q.theta)+s.localY*Math.cos(q.theta);if(arena.clearance(x,y)<(arena.custom?Arena.SENSOR_BAND:0))v[k]=1000;}return new ParkingCore.Observation(robot.getMillis(),robot.odometry.leftCount(),robot.odometry.rightCount(),v);}
  public void step(ParkingCore.Command cmd){if(lost)return;ParkingCore.Command c=dynamics.apply(cmd);robot.setMotorPWM(c.left,c.right);
   if(!arena.custom)robot.updatePhysics(10);
   else{
    robot.motors.update();float l=robot.motors.left.speedMmPerMs*10,r=robot.motors.right.speedMmPerMs*10;
    double f=arena.motionFraction(robot.odometry.preciseX,robot.odometry.preciseY,robot.odometry.preciseTheta,l,r);boolean hit=f<.999999;
    if(hit&&!colliding)collisions++;colliding=hit;
    // 只积分实际允许的行程，编码器不会报告穿过墙的虚假位移。
    robot.odometry.update((float)(l*f),(float)(r*f),robot.wheelSeparationMm);
    if(hit){dynamics.left*=f;dynamics.right*=f;robot.motors.left.speedMmPerMs*=f;robot.motors.right.speedMmPerMs*=f;}
   }
   robot.advanceTime(10);double x=robot.odometry.preciseX,y=robot.odometry.preciseY,a=robot.odometry.preciseTheta;currentWheel=1e30;for(int sign:new int[]{-1,1})currentWheel=Math.min(currentWheel,arena.clearance(x-sign*45*Math.sin(a),y+sign*45*Math.cos(a)));minWheel=Math.min(minWheel,currentWheel);boolean now=currentWheel<0;if(now&&!outside){crossings++;if(firstCross<0)firstCross=robot.getMillis();}outside=now;risk.observe(this,.01);if(stopMode.equals("line")&&currentWheel<0||stopMode.equals("support")&&currentWheel < -margin){lost=true;stop();}}
  public ParkingCore.Pose truth(){double x=robot.odometry.preciseX-tx,y=robot.odometry.preciseY-ty;return new ParkingCore.Pose(Math.cos(ta)*x+Math.sin(ta)*y,-Math.sin(ta)*x+Math.cos(ta)*y,ParkingCore.wrap(robot.odometry.preciseTheta-ta));}
  public boolean usesLocalBoundaryEvidence(){return arena.custom;}
  public boolean supportLost(){return lost;}public void stop(){robot.setMotorPWM(0,0);robot.motors.update();}
 }
 static class Archive implements AutoCloseable {
  final Path dir;BufferedWriter writer;int part=0,rows=0;long bytes=0;Path active;
  static final String HEADER="time_ms,x_mm,y_mm,yaw_rad,state,guard,edge_events,minimum_wheel_clearance_mm,current_wheel_clearance_mm,left_count,right_count,left_pwm,right_pwm,s0,s1,s2,s3,s4,estimate_x_mm,estimate_y_mm,estimate_yaw_rad,body_speed_mm_s,left_speed_mm_s,right_speed_mm_s,left_unsupported_fraction,right_unsupported_fraction,critical_support,boundary_cost,critical_events,centre_clearance_mm";
  Archive(Path dir)throws IOException{this.dir=dir;}
  void write(String line)throws IOException{if(writer==null||rows==50000){close();active=dir.resolve(String.format(Locale.US,"part-%05d.csv.gz",++part));writer=new BufferedWriter(new OutputStreamWriter(new GZIPOutputStream(Files.newOutputStream(active,StandardOpenOption.CREATE_NEW),65536,true),java.nio.charset.StandardCharsets.UTF_8),65536);writer.write(HEADER);writer.newLine();rows=0;}writer.write(line);writer.newLine();rows++;}
  long size()throws IOException{if(writer!=null)writer.flush();return bytes+(active==null?0:Files.size(active));}
  public void close()throws IOException{if(writer!=null){writer.close();bytes+=Files.size(active);writer=null;active=null;}}
 }
 final JSONObject config;final Path directory;final Plant plant;final ParkingCore.Trial trial;final Archive archive;final long reserve,quota;long lastMotion=0,previousL=0,previousR=0;boolean finished=false;String result="running";
 SimulationRunner(JSONObject c,Path dir)throws Exception{config=c;directory=dir;reserve=(long)(c.getDouble("reserve_gb")*1e9);quota=(long)(c.getDouble("quota_gb")*1e9);plant=new Plant(c);trial=new ParkingCore.Trial(plant,new ParkingCore.MapDescription(),dir.getFileName().toString(),"panel",new ParkingCore.Calibration(1000));archive=new Archive(dir);}
 JSONArray frame(){ParkingCore.Observation o=plant.observe();double x=plant.robot.odometry.preciseX,y=plant.robot.odometry.preciseY,a=plant.robot.odometry.preciseTheta;ParkingCore.Pose e=trial.controller.estimatedMapPose();double ex=Double.NaN,ey=Double.NaN,ea=Double.NaN;
  if(e!=null){ex=plant.tx+Math.cos(plant.ta)*e.x-Math.sin(plant.ta)*e.y;ey=plant.ty+Math.sin(plant.ta)*e.x+Math.cos(plant.ta)*e.y;ea=e.a+plant.ta;}else{e=trial.controller.searchOdometryPose();if(e!=null){double a0=config.getDouble("robot_yaw");ex=config.getDouble("robot_x")+Math.cos(a0)*e.x-Math.sin(a0)*e.y;ey=config.getDouble("robot_y")+Math.sin(a0)*e.x+Math.cos(a0)*e.y;ea=e.a+a0;}}
  JSONArray f=new JSONArray();f.append(o.millis);f.append(x);f.append(y);f.append(a);f.append(trial.controller.state.toString());f.append(trial.controller.guardState);f.append(trial.controller.edgeEvents);f.append(plant.minWheel==1e30?0:plant.minWheel);f.append(plant.currentWheel==1e30?0:plant.currentWheel);f.append(o.leftCount);f.append(o.rightCount);f.append(trial.command.left);f.append(trial.command.right);for(int k=0;k<5;k++)f.append(o.sensor(k));if(Double.isFinite(ex)){f.append(ex);f.append(ey);f.append(ea);}else{f.append("unknown");f.append("unknown");f.append("unknown");}
  double left=plant.robot.motors.left.speedMmPerMs*1000.0,right=plant.robot.motors.right.speedMmPerMs*1000.0;f.append((left+right)/2);f.append(left);f.append(right);f.append(plant.risk.leftFraction);f.append(plant.risk.rightFraction);f.append(plant.risk.critical?1:0);f.append(plant.risk.totalCost);f.append(plant.risk.criticalEvents);f.append(plant.risk.centreClearance);return f;
 }
 String csv(JSONArray f){StringJoiner s=new StringJoiner(",");for(int i=0;i<f.size();i++)s.add(String.valueOf(f.get(i)));return s.toString();}
 // 关闭窗口与动画推进互斥，避免回放写到已经关闭的流。
 synchronized void finish(String reason)throws IOException{if(finished)return;finished=true;result=reason;plant.stop();archive.close();JSONObject s=new JSONObject();s.setString("status",result);s.setLong("time_ms",plant.robot.getMillis());s.setLong("first_black_ms",trial.controller.firstBlackMs);s.setInt("edges",trial.controller.edgeEvents);s.setInt("wheel_crossings",plant.crossings);s.setBoolean("custom_map",plant.arena.custom);s.setInt("map_collisions",plant.collisions);s.setDouble("min_wheel_clearance_mm",plant.minWheel==1e30?0:plant.minWheel);s.setDouble("path_mm",trial.pathLength);s.setBoolean("parked",trial.success);s.setDouble("centre_error_mm",Math.hypot(plant.truth().x,plant.truth().y));s.setDouble("heading_error_deg",Math.toDegrees(Math.abs(ParkingCore.wrap(plant.truth().a-Math.PI/2))));s.setLong("bytes",archive.bytes);s.setInt("parts",archive.part);s.setDouble("boundary_cost",plant.risk.totalCost);s.setInt("critical_events",plant.risk.criticalEvents);s.setDouble("full_wheel_seconds",plant.risk.fullWheelSeconds);s.setDouble("critical_seconds",plant.risk.criticalSeconds);Files.writeString(directory.resolve("summary.tmp"),s.toString());Files.move(directory.resolve("summary.tmp"),directory.resolve("summary.json"),StandardCopyOption.REPLACE_EXISTING);}
 synchronized JSONObject step(int count)throws Exception{JSONArray frames=new JSONArray();for(int i=0;i<count&&!finished;i++){
   trial.tick();ParkingCore.Observation o=plant.observe();if(o.leftCount!=previousL||o.rightCount!=previousR)lastMotion=o.millis;previousL=o.leftCount;previousR=o.rightCount;
   if(trial.controller.firstBlackMs>=0&&!trial.controller.done()&&o.millis-lastMotion>10000)trial.controller.fail("local_encoder_stall_10s");
   if(trial.controller.retainedHistorySize()>1000000)trial.controller.fail("local_history_capacity");
   JSONArray f=frame();archive.write(csv(f));if(i%10==0||trial.controller.done()||i==count-1)frames.append(f);
   if(trial.controller.done())finish(trial.success?"parked":trial.falsePositive?"false_completion":trial.controller.failure);
   if(!finished&&trial.steps%100==0){if(Files.getFileStore(directory).getUsableSpace()<reserve)finish("storage_reserve_stop");else if(archive.size()>quota)finish("run_storage_quota_stop");}
  }JSONObject out=new JSONObject();out.setJSONArray("frames",frames);out.setBoolean("done",finished);out.setString("status",result);out.setLong("time_ms",plant.robot.getMillis());out.setLong("first_black_ms",trial.controller.firstBlackMs);out.setDouble("path_mm",trial.pathLength);out.setLong("bytes",archive.size());return out;}
 public static JSONObject configureProject(Path root)throws Exception{
  JSONObject c=JSONObject.parse(Files.readString(root.resolve("configs/CURRENT_SEARCH.json")));
  JSONObject b=c.getJSONObject("boundary_motion");
  Path main=root.resolve(c.getString("policy")),motion=root.resolve(b.getString("policy"));
  checkHash(main,c.getString("sha256"));checkHash(motion,b.getString("sha256"));
  System.setProperty("parking.policy",main.toString());System.setProperty("parking.motionPolicy",motion.toString());
  System.setProperty("parking.boundaryMotion",b.getString("mode","learned"));System.setProperty("parking.parallelDegrees",String.valueOf(b.getDouble("parallel_threshold_degrees")));
  if(b.hasKey("safe_mm"))System.setProperty("parking.vectorSafeMm",String.valueOf(b.getDouble("safe_mm")));
  JSONObject g=c.hasKey("search_gate")?c.getJSONObject("search_gate"):new JSONObject();
  System.setProperty("parking.gate",g.getString("mode","hull"));
  if(g.hasKey("inward_deg"))System.setProperty("parking.gateInwardDeg",String.valueOf(g.getDouble("inward_deg")));
  if(g.hasKey("memory_mm"))System.setProperty("parking.gateMemoryMm",String.valueOf(g.getDouble("memory_mm")));
  if(g.hasKey("depart_mm"))System.setProperty("parking.gateDepartMm",String.valueOf(g.getDouble("depart_mm")));
  System.setProperty("parking.redundancyWeight",String.valueOf(c.hasKey("redundancy_weight")?c.getDouble("redundancy_weight"):0));
  JSONObject l=c.hasKey("local_parking")?c.getJSONObject("local_parking"):new JSONObject();
  System.setProperty("parking.adaptiveEntry",String.valueOf(l.getBoolean("adaptive_entry",false)));
  System.setProperty("parking.directApproach",String.valueOf(l.getBoolean("direct_approach",false)));
  System.setProperty("parking.learnBoundary","true");System.setProperty("parking.hardHeading","true");System.setProperty("parking.policyTemperature",String.valueOf(c.getDouble("temperature")));
  System.setProperty("parking.memory","lines_corridor_arc");System.setProperty("parking.reverseArc","true");System.setProperty("parking.localMethod","early_fit");
  return c;
 }
 static void checkHash(Path path,String expected)throws Exception{
  String actual=HexFormat.of().formatHex(java.security.MessageDigest.getInstance("SHA-256").digest(Files.readAllBytes(path)));
  if(!actual.equals(expected))throw new IOException("Model checksum mismatch: "+path);
 }

}
