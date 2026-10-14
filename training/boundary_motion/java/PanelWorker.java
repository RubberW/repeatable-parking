import java.io.*;import java.nio.file.*;import java.util.*;import java.util.zip.*;import processing.core.*;import processing.data.*;

/** One offline run per process. Only sensors/encoders reach the frozen controller. */
public final class PanelWorker {
 static class Arena {
  final String shape;final double w,h;final double[][] polygon;
  Arena(JSONObject c){shape=c.getString("shape");w=c.getDouble("width_mm");h=c.getDouble("height_mm");JSONArray a=c.getJSONArray("polygon");polygon=new double[a.size()][2];for(int i=0;i<a.size();i++){polygon[i][0]=a.getJSONArray(i).getDouble(0);polygon[i][1]=a.getJSONArray(i).getDouble(1);}}
  double clearance(double x,double y){
   if(shape.equals("rectangle"))return Math.min(w/2-Math.abs(x),h/2-Math.abs(y));
   if(shape.equals("circle"))return w/2-Math.hypot(x,y);
   // Ellipses are represented by a dense convex polygon, also used for drawing.
   boolean inside=false;double nearest=Double.POSITIVE_INFINITY;
   for(int i=0,j=polygon.length-1;i<polygon.length;j=i++){
    double ax=polygon[j][0],ay=polygon[j][1],bx=polygon[i][0],by=polygon[i][1],dx=bx-ax,dy=by-ay;
    if((ay>y)!=(by>y)&&x<(bx-ax)*(y-ay)/(by-ay)+ax)inside=!inside;
    double u=Math.max(0,Math.min(1,((x-ax)*dx+(y-ay)*dy)/(dx*dx+dy*dy)));nearest=Math.min(nearest,Math.hypot(x-ax-u*dx,y-ay-u*dy));
   }return inside?nearest:-nearest;
  }
 }
 static class Surface extends PImage {
  final ParkingCore.MapDescription map=new ParkingCore.MapDescription(true);final double tx,ty,cs,sn;
  Surface(JSONObject c){width=height=1000000;format=PConstants.RGB;tx=c.getDouble("target_x");ty=c.getDouble("target_y");double a=c.getDouble("target_yaw");cs=Math.cos(a);sn=Math.sin(a);}
  @Override public int get(int px,int py){double x=px-width/2+.5-tx,y=height/2-py-.5-ty,u=cs*x+sn*y,v=-sn*x+cs*y;return Math.abs(u)<=77&&Math.abs(v)<=92&&map.black(u,v)?0xff000000:0xffffffff;}
 }
 static class Plant implements ParkingCore.Plant {
  final DigitalTwinParkingPrecision.RobotSimulation_c robot;final WheelDynamics dynamics=new WheelDynamics();final SupportRisk risk=new SupportRisk();final Arena arena;final double tx,ty,ta,margin;final String stopMode;
  boolean lost=false;long firstCross=-1;double minWheel=1e30,currentWheel=1e30;int crossings=0;boolean outside=false;
  Plant(JSONObject c){arena=new Arena(c);tx=c.getDouble("target_x");ty=c.getDouble("target_y");ta=c.getDouble("target_yaw");margin=c.getDouble("margin_mm");stopMode=c.getString("stop_mode");DigitalTwinParkingPrecision a=new DigitalTwinParkingPrecision();a.g=new PGraphics();a.g.colorMode(PConstants.RGB,255);robot=a.new RobotSimulation_c(16,90,new Surface(c),0,0);Random r=new Random(c.getLong("encoder_seed"));robot.resetForRun((float)c.getDouble("robot_x"),(float)c.getDouble("robot_y"),(float)c.getDouble("robot_yaw"),r.nextInt(18000)-9000,r.nextInt(18000)-9000);}
  public ParkingCore.Observation observe(){robot.getSurfaceSensors();float[] v=robot.sensorReadings().clone();DigitalTwinParkingPrecision.RobotPose_c q=robot.odometry.pose;for(int k=0;k<5;k++){DigitalTwinParkingPrecision.SurfaceSensorModel_c s=robot.sensors[k];double x=q.x+s.localX*Math.cos(q.theta)-s.localY*Math.sin(q.theta),y=q.y+s.localX*Math.sin(q.theta)+s.localY*Math.cos(q.theta);if(arena.clearance(x,y)<0)v[k]=1000;}return new ParkingCore.Observation(robot.getMillis(),robot.odometry.leftCount(),robot.odometry.rightCount(),v);}
  public void step(ParkingCore.Command cmd){if(lost)return;ParkingCore.Command c=dynamics.apply(cmd);robot.setMotorPWM(c.left,c.right);robot.updatePhysics(10);robot.advanceTime(10);double x=robot.odometry.preciseX,y=robot.odometry.preciseY,a=robot.odometry.preciseTheta;currentWheel=1e30;for(int sign:new int[]{-1,1})currentWheel=Math.min(currentWheel,arena.clearance(x-sign*45*Math.sin(a),y+sign*45*Math.cos(a)));minWheel=Math.min(minWheel,currentWheel);boolean now=currentWheel<0;if(now&&!outside){crossings++;if(firstCross<0)firstCross=robot.getMillis();}outside=now;risk.observe(this,.01);if(stopMode.equals("line")&&currentWheel<0||stopMode.equals("support")&&currentWheel < -margin){lost=true;stop();}}
  public ParkingCore.Pose truth(){double x=robot.odometry.preciseX-tx,y=robot.odometry.preciseY-ty;return new ParkingCore.Pose(Math.cos(ta)*x+Math.sin(ta)*y,-Math.sin(ta)*x+Math.cos(ta)*y,ParkingCore.wrap(robot.odometry.preciseTheta-ta));}
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
 PanelWorker(JSONObject c,Path dir)throws Exception{config=c;directory=dir;reserve=(long)(c.getDouble("reserve_gb")*1e9);quota=(long)(c.getDouble("quota_gb")*1e9);plant=new Plant(c);trial=new ParkingCore.Trial(plant,new ParkingCore.MapDescription(true),dir.getFileName().toString(),"panel",null,"compact_trace",new ParkingCore.Calibration(1000));archive=new Archive(dir);}
 JSONArray frame(){ParkingCore.Observation o=plant.observe();double x=plant.robot.odometry.preciseX,y=plant.robot.odometry.preciseY,a=plant.robot.odometry.preciseTheta;ParkingCore.Pose e=trial.controller.estimatedMapPose();double ex=Double.NaN,ey=Double.NaN,ea=Double.NaN;
  if(e!=null){ex=plant.tx+Math.cos(plant.ta)*e.x-Math.sin(plant.ta)*e.y;ey=plant.ty+Math.sin(plant.ta)*e.x+Math.cos(plant.ta)*e.y;ea=e.a+plant.ta;}else{e=trial.controller.searchOdometryPose();if(e!=null){double a0=config.getDouble("robot_yaw");ex=config.getDouble("robot_x")+Math.cos(a0)*e.x-Math.sin(a0)*e.y;ey=config.getDouble("robot_y")+Math.sin(a0)*e.x+Math.cos(a0)*e.y;ea=e.a+a0;}}
  JSONArray f=new JSONArray();f.append(o.millis);f.append(x);f.append(y);f.append(a);f.append(trial.controller.state.toString());f.append(trial.controller.guardState);f.append(trial.controller.edgeEvents);f.append(plant.minWheel==1e30?0:plant.minWheel);f.append(plant.currentWheel==1e30?0:plant.currentWheel);f.append(o.leftCount);f.append(o.rightCount);f.append(trial.command.left);f.append(trial.command.right);for(int k=0;k<5;k++)f.append(o.sensor(k));if(Double.isFinite(ex)){f.append(ex);f.append(ey);f.append(ea);}else{f.append("unknown");f.append("unknown");f.append("unknown");}
  double left=plant.robot.motors.left.speedMmPerMs*1000.0,right=plant.robot.motors.right.speedMmPerMs*1000.0;f.append((left+right)/2);f.append(left);f.append(right);f.append(plant.risk.leftFraction);f.append(plant.risk.rightFraction);f.append(plant.risk.critical?1:0);f.append(plant.risk.totalCost);f.append(plant.risk.criticalEvents);f.append(plant.risk.centreClearance);return f;
 }
 String csv(JSONArray f){StringJoiner s=new StringJoiner(",");for(int i=0;i<f.size();i++)s.add(String.valueOf(f.get(i)));return s.toString();}
 void finish(String reason)throws IOException{if(finished)return;finished=true;result=reason;plant.stop();archive.close();JSONObject s=new JSONObject();s.setString("status",result);s.setLong("time_ms",plant.robot.getMillis());s.setLong("first_black_ms",trial.controller.firstBlackMs);s.setInt("edges",trial.controller.edgeEvents);s.setInt("wheel_crossings",plant.crossings);s.setDouble("min_wheel_clearance_mm",plant.minWheel==1e30?0:plant.minWheel);s.setDouble("path_mm",trial.pathLength);s.setBoolean("parked",trial.success);s.setDouble("centre_error_mm",Math.hypot(plant.truth().x,plant.truth().y));s.setDouble("heading_error_deg",Math.toDegrees(Math.abs(ParkingCore.wrap(plant.truth().a-Math.PI/2))));s.setLong("bytes",archive.bytes);s.setInt("parts",archive.part);s.setDouble("boundary_cost",plant.risk.totalCost);s.setInt("critical_events",plant.risk.criticalEvents);s.setDouble("full_wheel_seconds",plant.risk.fullWheelSeconds);s.setDouble("critical_seconds",plant.risk.criticalSeconds);Files.writeString(directory.resolve("summary.tmp"),s.toString());Files.move(directory.resolve("summary.tmp"),directory.resolve("summary.json"),StandardCopyOption.REPLACE_EXISTING);}
 JSONObject step(int count)throws Exception{JSONArray frames=new JSONArray();for(int i=0;i<count&&!finished;i++){
   trial.tick();ParkingCore.Observation o=plant.observe();if(o.leftCount!=previousL||o.rightCount!=previousR)lastMotion=o.millis;previousL=o.leftCount;previousR=o.rightCount;
   if(trial.controller.firstBlackMs>=0&&!trial.controller.done()&&o.millis-lastMotion>10000)trial.controller.fail("local_encoder_stall_10s");
   if(trial.controller.retainedHistorySize()>1000000)trial.controller.fail("local_history_capacity");
   JSONArray f=frame();archive.write(csv(f));if(i%10==0||trial.controller.done()||i==count-1)frames.append(f);
   if(trial.controller.done())finish(trial.success?"parked":trial.falsePositive?"false_completion":trial.controller.failure);
   if(!finished&&trial.steps%100==0){if(Files.getFileStore(directory).getUsableSpace()<reserve)finish("storage_reserve_stop");else if(archive.size()>quota)finish("run_storage_quota_stop");}
  }trial.path.clear();JSONObject out=new JSONObject();out.setJSONArray("frames",frames);out.setBoolean("done",finished);out.setString("status",result);out.setLong("time_ms",plant.robot.getMillis());out.setLong("first_black_ms",trial.controller.firstBlackMs);out.setDouble("path_mm",trial.pathLength);out.setLong("bytes",archive.size());return out;}
 public static void main(String[] args)throws Exception{
  Path cfg=Paths.get(args[0]),dir=cfg.getParent();JSONObject c=JSONObject.parse(Files.readString(cfg));
  System.setProperty("parking.policy",c.getString("policy"));System.setProperty("parking.policySeed",String.valueOf(c.getLong("policy_seed")));System.setProperty("parking.memorySide",String.valueOf(c.getInt("memory_side")));System.setProperty("parking.runawayMm",String.valueOf(Math.max(20000,Math.hypot(c.getDouble("width_mm"),c.getDouble("height_mm"))*4)));
  System.setProperty("parking.offline","true");System.setProperty("parking.legacyBoundary","false");System.setProperty("parking.virtualBoundary","true");System.setProperty("parking.memory",c.getString("shape").equals("rectangle")?"lines_corridor_arc":"corridor_audit_arc");System.setProperty("parking.reverseArc","true");System.setProperty("parking.localMethod","early_fit");System.setProperty("parking.heuristicScore","false");System.setProperty("smooth.steering","smooth");
  PanelWorker worker=new PanelWorker(c,dir);try(BufferedReader in=new BufferedReader(new InputStreamReader(System.in))){System.out.println("{\"ready\":true}");System.out.flush();String line;while((line=in.readLine())!=null){if(line.equals("STOP")){worker.finish("user_stopped");System.out.println(worker.step(0).toString().replace("\n",""));System.out.flush();break;}int n=Math.max(1,Math.min(1000,Integer.parseInt(line)));JSONObject out=worker.step(n);System.out.println(out.toString().replace("\n",""));System.out.flush();if(worker.finished)break;}}finally{if(!worker.finished)worker.finish("window_closed");}
 }
}
