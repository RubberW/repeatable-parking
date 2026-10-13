import java.nio.file.*;
import java.util.*;
import java.io.*;

// Teacher plant tabs remain unchanged; table readings are an explicit
// environment adapter. Only reflectance and encoders reach the controller.
PImage floorMap;
ParkingCore.MapDescription printedMap;
ParkingCore.Trial current;
SchoolTablePlant demoPlant;
String outputRoot;
boolean paused=false,finishedImage=false;
int simulationStepsPerFrame=20;
long demoSeed=2026101101L;
void settings(){size(1180,830);}
void setup(){
  surface.setTitle("SEMTM0043 | Frozen policy / precise odometry: "+System.getProperty("parking.modelLabel","selected")+" | OFFLINE");
  outputRoot=System.getProperty("parking.root",sketchPath(".."));
  printedMap=new ParkingCore.MapDescription(true);floorMap=generateMap(printedMap);
  demoSeed=Long.getLong("parking.demoSeed",2026101101L);
  try{startDemo();}catch(Exception e){throw new RuntimeException(e);}frameRate(60);
}
PImage generateMap(ParkingCore.MapDescription description){
  PImage result=new PImage(3000,3000,RGB);result.loadPixels();
  for(int y=0;y<3000;y++)for(int x=0;x<3000;x++)result.pixels[y*3000+x]=description.black(x-1500+.5,1500-y-.5)?0xff000000:0xffffffff;
  result.updatePixels();return result;
}
class Table {
  final double widthMm,heightMm,x,y,yaw;
  Table(int index){widthMm=Integer.getInteger("parking.arenaMetres",3)*1000;heightMm=widthMm;x=0;y=0;yaw=0;}
  Table(double w,double h){widthMm=w;heightMm=h;x=0;y=0;yaw=0;}
  boolean contains(double px,double py){return clearance(px,py)>=0;}
  double clearance(double px,double py){
    double dx=px-x,dy=py-y,c=Math.cos(yaw),s=Math.sin(yaw);
    return Math.min(widthMm/2-Math.abs(c*dx+s*dy),heightMm/2-Math.abs(-s*dx+c*dy));
  }
  double[] world(double px,double py){double c=Math.cos(yaw),s=Math.sin(yaw);return new double[]{x+c*px-s*py,y+s*px+c*py};}
}
class SchoolTablePlant implements ParkingCore.Plant {
  private final RobotSimulation_c robot;
  final WheelDynamics dynamics=new WheelDynamics();
  final Table table;
  final double edgeReading;
  boolean lost=false;
  double minimumWheelClearance=Double.POSITIVE_INFINITY;
  long firstSupportLoss=-1;
  SchoolTablePlant(PImage image,Table table,double x,double y,double yaw,long seed,double edgeReading){
    this.table=table;this.edgeReading=edgeReading;robot=new RobotSimulation_c(16,90,image,0,0);
    Random r=new Random(seed);robot.resetForRun((float)x,(float)y,(float)yaw,r.nextInt(18000)-9000,r.nextInt(18000)-9000);
  }
  public ParkingCore.Observation observe(){
    robot.getSurfaceSensors();float[] readings=robot.sensorReadings().clone();RobotPose_c p=robot.odometry.pose;
    for(int i=0;i<5;i++){
      SurfaceSensorModel_c sensor=robot.sensors[i];
      double sx=p.x+sensor.localX*Math.cos(p.theta)-sensor.localY*Math.sin(p.theta);
      double sy=p.y+sensor.localX*Math.sin(p.theta)+sensor.localY*Math.cos(p.theta);
      if(!table.contains(sx,sy))readings[i]=(float)edgeReading;
    }
    return new ParkingCore.Observation(robot.getMillis(),robot.odometry.leftCount(),robot.odometry.rightCount(),readings);
  }
  public void step(ParkingCore.Command c){
    ParkingCore.Command actual=dynamics.apply(c);robot.setMotorPWM(actual.left,actual.right);robot.updatePhysics(10);robot.advanceTime(10);
    RobotPose_c p=robot.odometry.pose;
    // Independent diagnostic: two ideal wheel contact centres, not 3-D tipping.
    for(int sign:new int[]{-1,1}){
      double wx=p.x-sign*45*Math.sin(p.theta),wy=p.y+sign*45*Math.cos(p.theta);
      minimumWheelClearance=Math.min(minimumWheelClearance,table.clearance(wx,wy));
    }
    if(minimumWheelClearance<0 && !Boolean.parseBoolean(System.getProperty("parking.virtualBoundary","true"))){lost=true;if(firstSupportLoss<0)firstSupportLoss=robot.getMillis();stop();}
  }
  public boolean supportLost(){return lost;}
  public ParkingCore.Pose truth(){RobotPose_c p=robot.odometry.pose;return new ParkingCore.Pose(p.x,p.y,p.theta);}
  public void stop(){robot.setMotorPWM(0,0);robot.motors.update();}
}
void startDemo()throws Exception{
  Random random=new Random(demoSeed);Table table=new Table(0);
  double[] pos=table.world(-table.widthMm/2+70+(table.widthMm-140)*random.nextDouble(),-table.heightMm/2+70+(table.heightMm-140)*random.nextDouble());
  double a=-Math.PI+2*Math.PI*random.nextDouble();
  demoPlant=new SchoolTablePlant(floorMap,table,pos[0],pos[1],a,demoSeed,1000);
  Path out=Paths.get(outputRoot,"data","precision_demo","demo_"+demoSeed);
  if(Files.exists(out.resolve("trajectory.csv")))out=Paths.get(outputRoot,"data","precision_demo","demo_"+demoSeed+"_"+System.currentTimeMillis());
  current=new ParkingCore.Trial(demoPlant,printedMap,"demo","unknown_table",out.resolve("trajectory.csv"),System.getProperty("parking.search","compact_trace"),new ParkingCore.Calibration(1000));finishedImage=false;
}
void draw(){
  if(!paused && !current.controller.done())for(int i=0;i<simulationStepsPerFrame && !current.controller.done();i++)current.tick();
  background(232,236,242);noStroke();fill(155);rect(16,16,770,height-32,12);
  Table table=demoPlant.table;float zoom=(float)Math.min(710/(table.widthMm+100),700/(table.heightMm+100));
  pushMatrix();translate(400,height/2);scale(zoom,-zoom);
  pushMatrix();translate((float)table.x,(float)table.y);rotate((float)table.yaw);
  fill(255);stroke(80);strokeWeight(2/zoom);rectMode(CENTER);rect(0,0,(float)table.widthMm,(float)table.heightMm);popMatrix();
  noFill();stroke(218);strokeWeight(1/zoom);rect(0,0,210,297);rectMode(CORNER);
  fill(0);noStroke();for(ParkingCore.Rect ink:printedMap.ink)rect((float)ink.x0,(float)ink.y0,(float)(ink.x1-ink.x0),(float)(ink.y1-ink.y0));
  noFill();stroke(40,135,210);strokeWeight(1.4/zoom);beginShape();for(ParkingCore.Pose p:current.path)vertex((float)p.x,(float)p.y);endShape();
  stroke(30,165,90);strokeWeight(2/zoom);line(60,0,60,50);line(60,50,53,38);line(60,50,67,38);ellipse(0,0,6,6);
  ParkingCore.Pose truth=current.plant.truth();pushMatrix();translate((float)truth.x,(float)truth.y);rotate((float)truth.a);
  fill(current.success?color(60,180,110):color(100,155,215));stroke(25,55,90);ellipse(0,0,100,100);line(0,0,45,0);
  ParkingCore.Observation o=current.plant.observe();for(int i=0;i<5;i++){float a=radians((float)ParkingCore.SENSOR_ANGLES[i]);int k=current.controller.calibration.kind(i,o.sensor(i));fill(k==2?color(235,90,30):(k==1?0:255));ellipse(45*cos(a),45*sin(a),6,6);}popMatrix();
  ParkingCore.Pose p=current.controller.estimatedMapPose();if(p!=null){noFill();stroke(230,135,35);ellipse((float)p.x,(float)p.y,108,108);}popMatrix();
  fill(25,40,60);textSize(22);text(((int)(table.widthMm/1000))+" m SQUARE / A4",808,52);textSize(14);
  text("Assumed motor lag / smooth turns / 10 ms",808,81);text("OFFLINE - unknown initial pose",808,105);
  text("Printed bay: 140 x 170 mm",808,147);text("A4: 210 x 297 mm at 100%",808,170);
  text("Virtual region: "+(int)table.widthMm+" x "+(int)table.heightMm+" mm",808,195);text("Controller does not know table size",808,218);
  text("State: "+current.controller.state,808,263);text("Edge guard: "+current.controller.guardState,808,286);text("Edge events: "+current.controller.edgeEvents,808,310);
  text("Policy: "+System.getProperty("parking.modelLabel","selected"),808,351);text(String.format(Locale.US,"Time: %.1f s / Path: %.0f mm",current.steps*.01,current.pathLength),808,377);
  text("Direction bars: "+current.controller.codeEvents+" / 3",808,413);text("Centreline confirmed: "+current.controller.lateralCalibrated,808,437);
  if(current.controller.done()){
    fill(current.success?color(15,130,60):color(185,40,35));textSize(19);text(current.success?"PASS":"STOP / FAILURE RECORDED",808,494);textSize(13);
    text(current.controller.failure,808,522);fill(25,40,60);text(String.format(Locale.US,"Centre error: %.2f mm",current.finalError),808,550);text(String.format(Locale.US,"Heading error: %.2f deg",current.headingError),808,575);
  }
  textSize(13);text("Synthetic edge signal; no height ranging",808,632);text("Front sensors cannot guarantee no falls",808,657);
  text("Space: pause / R: next random start",808,height-62);text("+/-: playback speed / S: screenshot",808,height-38);
  if(current.controller.done() && !finishedImage){saveFrame(Paths.get(outputRoot,"results","precision_demo","demo_final.png").toString());finishedImage=true;if(Boolean.getBoolean("parking.autoExit"))exit();}
}
void keyPressed(){
  if(key==' ')paused=!paused;
  if(key=='+'||key=='=')simulationStepsPerFrame=Math.min(500,simulationStepsPerFrame*2);
  if(key=='-')simulationStepsPerFrame=Math.max(1,simulationStepsPerFrame/2);
  if(key=='s'||key=='S')saveFrame(Paths.get(outputRoot,"results","precision_demo","manual_view.png").toString());
  if(key=='r'||key=='R'){if(current.log!=null)current.log.close();demoSeed++;try{startDemo();}catch(Exception e){throw new RuntimeException(e);}}
}
public static void main(String[] argv){
  try{
    if(argv.length>0 && (argv[0].equals("--batch") || argv[0].equals("--edge-fixtures"))){
      DigitalTwinParkingPrecision app=new DigitalTwinParkingPrecision();app.g=new PGraphics();app.g.colorMode(RGB,255);app.outputRoot=System.getProperty("parking.root","");
      if(argv[0].equals("--edge-fixtures"))app.edgeFixtures(Paths.get(argv[1]));else app.batch(argv);
    }else PApplet.main(DigitalTwinParkingPrecision.class.getName(),argv);
  }catch(Exception e){throw new RuntimeException(e);}
}
void batch(String[] argv)throws Exception{
  long seed=Long.parseLong(argv[1]);int count=Integer.parseInt(argv[2]);Path out=Paths.get(argv[3]);
  String method=argv.length>4?argv[4]:"compact_trace";double edge=argv.length>5?Double.parseDouble(argv[5]):1000;
  boolean logs=argv.length<=6 || !argv[6].equals("no-logs");
  if(Files.exists(out.resolve("summary.csv")))throw new IllegalStateException("Existing results are immutable; choose a new output folder.");
  Files.createDirectories(out);printedMap=new ParkingCore.MapDescription(true);floorMap=generateMap(printedMap);floorMap.save(out.resolve("surface.png").toString());
  Random random=new Random(seed);PrintWriter cases=new PrintWriter(Files.newBufferedWriter(out.resolve("cases.csv")));
  PrintWriter summary=new PrintWriter(Files.newBufferedWriter(out.resolve("summary.csv")));
  cases.println("id,table_width,table_height,table_x,table_y,table_yaw,initial_x,initial_y,initial_yaw,encoder_seed");
  summary.println("id,method,table_width,table_height,success,false_positive,state,failure,time_s,path_mm,first_black_s,first_fit_s,localization_s,centre_error_mm,heading_error_deg,parking_clearance_mm,stationary_ms,edge_events,wheel_support_lost,minimum_wheel_clearance_mm,heading_calibrated,lateral_calibrated,lateral_correction_mm,search_path_mm,search_repeated_mm,search_unique_cells");
  int passed=0,lost=0;
  for(int i=0;i<count;i++){
    Table table=new Table(i);double[] position=table.world(-table.widthMm/2+70+(table.widthMm-140)*random.nextDouble(),-table.heightMm/2+70+(table.heightMm-140)*random.nextDouble());
    double a=-Math.PI+2*Math.PI*((i%8)+random.nextDouble())/8;long encoderSeed=random.nextLong();String id=String.format(Locale.US,"run_%03d",i);
    cases.printf(Locale.US,"%s,%.0f,%.0f,%.3f,%.3f,%.8f,%.8f,%.8f,%.12f,%d%n",id,table.widthMm,table.heightMm,table.x,table.y,table.yaw,position[0],position[1],a,encoderSeed);cases.flush();
    SchoolTablePlant plant=new SchoolTablePlant(floorMap,table,position[0],position[1],a,encoderSeed,edge);
    ParkingCore.Trial trial=new ParkingCore.Trial(plant,printedMap,id,"unknown_table",logs?out.resolve("trajectories").resolve(id+".csv"):null,method,new ParkingCore.Calibration(edge));trial.run();
    ParkingCore.Controller c=trial.controller;if(trial.success)passed++;if(plant.lost)lost++;
    summary.printf(Locale.US,"%s,%s,%.0f,%.0f,%b,%b,%s,%s,%.2f,%.3f,%.2f,%.2f,%.2f,%.4f,%.5f,%.3f,%d,%d,%b,%.3f,%b,%b,%.4f,%.4f,%.4f,%d%n",id,method,table.widthMm,table.heightMm,trial.success,trial.falsePositive,c.state,c.failure,trial.steps*.01,trial.pathLength,c.firstBlackMs*.001,c.firstFitMs*.001,c.localizationMs*.001,trial.finalError,trial.headingError,trial.margin,trial.stationaryMs,c.edgeEvents,plant.lost,plant.minimumWheelClearance,c.headingCalibrated,c.lateralCalibrated,c.lateralCorrection,c.searchPathMm,c.searchRepeatedMm,c.searchUnique);summary.flush();
    System.out.printf(Locale.US,"%s %s %s %.1fs %.2fmm edges=%d%n",method,id,trial.success?"PASS":("FAIL:"+c.failure),trial.steps*.01,trial.finalError,c.edgeEvents);
  }
  cases.close();summary.close();Files.writeString(out.resolve("aggregate.json"),"{\"seed\":"+seed+",\"runs\":"+count+",\"passed\":"+passed+",\"support_losses\":"+lost+",\"method\":\""+method+"\",\"synthetic_edge_reading\":"+edge+"}\n");
  System.out.println("RESULT "+passed+"/"+count+" support_losses="+lost);
}
void edgeFixtures(Path out)throws Exception{
  if(Files.exists(out.resolve("summary.csv")))throw new IllegalStateException("Existing fixtures cannot be overwritten");
  Files.createDirectories(out);printedMap=new ParkingCore.MapDescription(true);floorMap=generateMap(printedMap);
  String[] names={"frontal_gray","frontal_dark_void","grazing_80deg","black_marker","overlapping_ink_void","corner_approach"};
  double[][] parameters={{600,450,220,0,0,1000},{600,450,220,0,0,2500},{600,2000,220,0,Math.toRadians(80),1000},{600,450,0,-150,Math.PI/2,1000},{600,450,220,0,0,2000},{600,450,220,145,Math.PI/4,1000}};
  PrintWriter summary=new PrintWriter(Files.newBufferedWriter(out.resolve("summary.csv")));
  summary.println("fixture,edge_events,first_edge_s,support_lost,support_loss_s,minimum_wheel_clearance_mm,state,failure");
  for(int i=0;i<names.length;i++){
    double[] p=parameters[i];SchoolTablePlant plant=new SchoolTablePlant(floorMap,new Table(p[0],p[1]),p[2],p[3],p[4],7,p[5]);
    ParkingCore.Trial trial=new ParkingCore.Trial(plant,printedMap,names[i],"edge_fixture",out.resolve(names[i]+".csv"),"compact_trace",new ParkingCore.Calibration(p[5]));
    long firstEdge=-1;
    while(!trial.controller.done() && trial.steps<2000){trial.tick();if(firstEdge<0 && trial.controller.edgeEvents>0)firstEdge=(trial.steps-1)*10;}
    if(!trial.controller.done()){trial.controller.fail("fixture_observation_window_end");trial.tick();}
    summary.printf(Locale.US,"%s,%d,%.3f,%b,%.3f,%.3f,%s,%s%n",names[i],trial.controller.edgeEvents,firstEdge*.001,plant.lost,plant.firstSupportLoss*.001,plant.minimumWheelClearance,trial.controller.state,trial.controller.failure);
  }
  summary.close();System.out.println("Edge fixtures saved: "+out);
}

