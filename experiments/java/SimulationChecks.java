/**
 * 文件职责：核对模型、方向约束及趣味地图几何；--editor-ui 实际点击原生窗口验收。
 * 输入：模型文件与 Python 参考输出；输出：通过或明确失败原因。
 * 调用关系：仅由 scripts/verify.py 使用，不参与正常停车。
 */
import java.io.*;import java.nio.file.*;import java.util.*;
public final class SimulationChecks {
 static void check(boolean value,String reason){if(!value)throw new AssertionError(reason);}
 public static void main(String[] args)throws Exception{
  if(args.length>0&&args[0].equals("--editor-ui")){editorUi(Paths.get(args[1]));return;}
  BoundaryMotion.Network motion=new BoundaryMotion.Network(Paths.get(args[1]));double max=0;
  List<String> rows=Files.readAllLines(Paths.get(args[2]));
  for(String row:rows.subList(1,rows.size())){
   String[] c=row.split(",");double chosen=Double.parseDouble(c[0]),original=Double.parseDouble(c[1]);
   double[] p=motion.predict(chosen,original),rotated=motion.predict(chosen+1,original+1),reflected=motion.predict(-chosen,-original);
   for(int j=0;j<3;j++){max=Math.max(max,Math.abs(p[j]-Double.parseDouble(c[j+2])));check(Math.abs(p[j]-rotated[j])<1e-7&&Math.abs(p[j]-reflected[j])<1e-7,"heading invariance");}
  }check(max<.0002,"motion parity "+max);
  EvaluationScenes.settings();System.setProperty("parking.boundaryMotion","conditional");
  for(double angle:new double[]{10,60,90,170}){
   CompactSearch m=new CompactSearch();BoundaryMotion b=m.boundaryMotion;
   b.edge.reliable=true;b.edge.nx=Math.cos(Math.toRadians(angle));b.edge.ny=Math.sin(Math.toRadians(angle));
   b.edge.alpha=Math.toDegrees(Math.acos(Math.abs(b.edge.nx)));m.goalX=-400;m.lastMs=0;b.beginEstimated(m,0);
   check(b.active==(angle==10),"conditional branch "+angle);
  }
  // 用实际传感器触边构造状态；真实几何只在本检查中判断方向。
  System.setProperty("parking.boundaryMotion","baseline");CompactSearch.Network net=new CompactSearch.Network(Paths.get(args[0]));int accepted=0;
  for(double yaw:new double[]{-1.3,-.8,-.4,0,.4,.8,1.3,Math.PI/4}){
   SimulationRunner.Plant p=new SimulationRunner.Plant(EvaluationScenes.config(1,300,yaw==Math.PI/4?300:0,yaw,72,0,0,0));
   CompactSearch m=new CompactSearch();m.observe(p.observe());
   for(int k=0;k<2000&&!m.headingGate.active;k++){p.step(CompactSearch.velocity(200,0));m.observe(p.observe());}
   check(m.headingGate.active,"missing edge observation");float[] f=SearchDecision.observation(m);int allowed=0;
   for(int a=0;a<32;a++)if(f[CompactSearch.MAP+a*CompactSearch.FEATURES+26]==0){
    double heading=p.robot.odometry.preciseTheta+m.turn(a),x=p.robot.odometry.preciseX,y=p.robot.odometry.preciseY;
    check(p.arena.clearance(x+m.length(a)*Math.cos(heading),y+m.length(a)*Math.sin(heading))>=0,"outside subgoal admitted");allowed++;
   }
   if(allowed>0){accepted++;for(int i=0;i<100;i++){int a=net.choose(f,new Random(i),false);check(f[CompactSearch.MAP+a*CompactSearch.FEATURES+26]==0,"mask bypassed");}}
   else check(net.choose(f,new Random(),false)==-1,"empty mask did not stop");
  }check(accepted>=6,"excessive gate stop");
  mapGeometry();
  localBoundaryEvidence(args[0]);
  System.out.println("PASS motion parity="+max+", conditional branches, direction gate, custom-map geometry/sensors/collision, per-scene finite boundary evidence");
 }
 static void localBoundaryEvidence(String mainPolicy)throws Exception{
  SearchMemory rectangular=new SearchMemory(),local=new SearchMemory(false);
  for(int i=0;i<3;i++){rectangular.point(100,i*250,0,i*250,i+1);local.point(100,i*250,0,i*250,i+1);}rectangular.fit();local.fit();
  check(rectangular.lines==1&&local.lines==0&&local.points==0,"3 contacts extrapolated a global line in complex mode / lost rectangle evidence");
  CompactSearch m=new CompactSearch(true);float[] white={400,400,400,400,400};
  m.observe(new ParkingCore.Observation(0,0,0,white));m.pose.x=100;m.distance=100;m.observe(new ParkingCore.Observation(10,0,0,white));
  check(m.memory.segments>0&&m.unique>0,"complex mode lost visitation/corridor memory");
  m=new CompactSearch(true);m.observe(new ParkingCore.Observation(0,0,0,new float[]{400,400,1000,400,400}));
  check(m.headingGate.hull.isEmpty()&&m.headingGate.allowed(m,16)&&!m.headingGate.allowed(m,0),"local contact used a convex hull or allowed movement into grey probe");
  m.outwardX=-1;m.outwardY=0;check(m.headingGate.localRayLength(m,0)==0,"recent local obstacle point ignored");
  m.lastMs=2001;check(m.headingGate.localRayLength(m,0)>0,"expired obstacle became a permanent wall");
  System.setProperty("parking.policy",mainPolicy);java.lang.reflect.Field field=ParkingCore.Controller.class.getDeclaredField("compact");field.setAccessible(true);
  SimulationRunner.Plant p=new SimulationRunner.Plant(custom(new double[][]{{-500,-500},{500,-500},{500,500},{-500,500}}));
  ParkingCore.Trial t=new ParkingCore.Trial(p,new ParkingCore.MapDescription(),"custom","check",new ParkingCore.Calibration(1000));t.tick();
  check(((CompactSearch)field.get(t.controller)).localBoundary,"custom map did not select local evidence");
  p=new SimulationRunner.Plant(EvaluationScenes.config(1,300,200,0,71,0,0,0));t=new ParkingCore.Trial(p,new ParkingCore.MapDescription(),"rectangle","check",new ParkingCore.Calibration(1000));t.tick();
  check(!((CompactSearch)field.get(t.controller)).localBoundary,"complex mode leaked into next rectangular trial");
  System.setProperty("parking.boundaryMotion","conditional");m=new CompactSearch(true);m.goalX=-100;m.lastMs=401;
  BoundaryMotion b=m.boundaryMotion;b.active=true;b.phase=1;b.command(m);
  check(b.blockedPivotRecoveries==1&&!m.invalid&&b.phase==4,"encoder-stalled local pivot did not recover to direct turn");
 }
 static processing.data.JSONArray points(double[][] p){processing.data.JSONArray a=new processing.data.JSONArray();for(double[] v:p){processing.data.JSONArray q=new processing.data.JSONArray();q.append(v[0]);q.append(v[1]);a.append(q);}return a;}
 static processing.data.JSONObject custom(double[][] boundary){
  processing.data.JSONObject c=EvaluationScenes.config(1,-100,200,0,71,200,0,0);
  c.setString("shape","polygon");c.setBoolean("custom_map",true);c.setJSONArray("polygon",points(boundary));c.setJSONArray("walls",new processing.data.JSONArray());return c;
 }
 static void mapGeometry(){
  double[][] ellipse=new double[96][2];for(int i=0;i<96;i++){ellipse[i][0]=450*Math.cos(i*Math.PI/48);ellipse[i][1]=300*Math.sin(i*Math.PI/48);}
  SimulationRunner.Arena a=new SimulationRunner.Arena(custom(ellipse));check(a.clearance(0,0)>299&&a.clearance(0,350)<0&&a.clearance(460,0)<0,"ellipse signed geometry");
  a=new SimulationRunner.Arena(custom(new double[][]{{-450,-450},{450,-450},{450,-100},{-100,-100},{-100,450},{-450,450}}));
  check(a.clearance(-250,250)>0&&a.clearance(250,250)<0,"concave region membership");
  boolean rejected=false;try{new SimulationRunner.Arena(custom(new double[][]{{-450,-450},{450,450},{-450,450},{450,-450}}));}catch(IllegalArgumentException expected){rejected=true;}
  check(rejected,"self-crossing boundary accepted");
  processing.data.JSONObject c=custom(new double[][]{{-500,-500},{500,-500},{500,500},{-500,500}});
  processing.data.JSONArray walls=new processing.data.JSONArray();walls.append(points(new double[][]{{0,-300},{0,300}}));c.setJSONArray("walls",walls);
  SimulationRunner.Plant p=new SimulationRunner.Plant(c);
  double initialCount=p.robot.odometry.preciseLeft;
  for(int i=0;i<300;i++)p.step(CompactSearch.velocity(200,0));
  check(p.robot.odometry.preciseX<=-54.99&&p.robot.odometry.preciseX>=-56&&p.collisions>0,"car drove through internal wall");
  boolean grey=false;ParkingCore.Observation observation=p.observe();for(int i=0;i<5;i++)grey|=observation.sensor(i)==1000;check(grey,"wall did not reach reflectance sensors");
  double encoderTravel=(p.robot.odometry.preciseLeft-initialCount)*p.robot.odometry.mmPerEncoderCount;
  check(encoderTravel>44&&encoderTravel<46,"blocked wheel encoder reported travel through wall: "+encoderTravel);
 }
 static void click(java.awt.Robot robot,java.awt.Point origin,int x,int y)throws Exception{
  robot.mouseMove(origin.x+x,origin.y+y);robot.mousePress(java.awt.event.InputEvent.BUTTON1_DOWN_MASK);robot.mouseRelease(java.awt.event.InputEvent.BUTTON1_DOWN_MASK);Thread.sleep(250);
 }
 static void stroke(java.awt.Robot robot,java.awt.Point origin,int[][] p)throws Exception{
  robot.mouseMove(origin.x+p[0][0],origin.y+p[0][1]);robot.mousePress(java.awt.event.InputEvent.BUTTON1_DOWN_MASK);
  for(int i=1;i<p.length;i++){robot.mouseMove(origin.x+p[i][0],origin.y+p[i][1]);Thread.sleep(60);}robot.mouseRelease(java.awt.event.InputEvent.BUTTON1_DOWN_MASK);Thread.sleep(400);
 }
 static void editorUi(Path root)throws Exception{
  System.setProperty("parking.root",root.toAbsolutePath().toString());System.setProperty("parking.arenaMetres","3");System.setProperty("parking.smokeFrames","0");
  Path draft=root.resolve(".cache/map-test-"+UUID.randomUUID()+".json");System.setProperty("parking.mapDraft",draft.toAbsolutePath().toString());
  ParkingSimulation app=new ParkingSimulation();processing.core.PApplet.runSketch(new String[]{"--location=20,0","ParkingSimulation"},app);
  boolean failed=false;
  try{
   long end=System.currentTimeMillis()+15000;while(app.editor==null||app.session==null){if(System.currentTimeMillis()>end)throw new AssertionError("window startup timed out");Thread.sleep(100);}
   Thread.sleep(700);java.awt.Component canvas=(java.awt.Component)app.getSurface().getNative();java.awt.Point origin=canvas.getLocationOnScreen();java.awt.Robot robot=new java.awt.Robot();robot.setAutoDelay(60);
   System.out.println("UI canvas origin="+origin+", size="+canvas.getSize()+", screen="+java.awt.Toolkit.getDefaultToolkit().getScreenSize());
   click(robot,origin,118,821);check(app.editor.metres==1&&app.session.plant.arena.w==1000,"size button: mouse="+app.mouseX+","+app.mouseY+", metres="+app.editor.metres+", notice="+app.editor.notice);
   click(robot,origin,419,821);check(app.editor.randomFrame&&Math.hypot(app.session.plant.tx,app.session.plant.ty)>1,"random parking frame switch");
   click(robot,origin,599,821);check(app.editor.editing&&app.paused,"editor switch did not pause");
   click(robot,origin,325,865);stroke(robot,origin,new int[][]{{115,130},{685,700}});check(app.editor.boundary.size()==96,"circle mouse tool");
   click(robot,origin,395,865);stroke(robot,origin,new int[][]{{100,130},{700,700}});check(app.editor.boundary.size()==96&&Math.abs(app.editor.boundary.get(0)[0]-.4714)<.02,"ellipse mouse tool");
   click(robot,origin,255,865);stroke(robot,origin,new int[][]{{110,130},{690,130},{690,400},{500,400},{500,700},{110,700},{110,130}});
   check(new SimulationRunner.Arena(app.editor.geometry()).outerClearance(300,-300)<0,"freehand concave boundary");
   click(robot,origin,613,865);check(app.editor.boundary.size()==96,"undo boundary");
   click(robot,origin,165,865);click(robot,origin,535,865);stroke(robot,origin,new int[][]{{470,300},{470,350}});check(app.editor.walls.size()==1,"wall mouse tool");
   click(robot,origin,613,865);check(app.editor.walls.isEmpty(),"undo wall");stroke(robot,origin,new int[][]{{470,300},{470,350}});
   click(robot,origin,904,865);check(!app.editor.editing&&app.session.plant.arena.custom&&app.session.plant.arena.walls.length==1,"apply map: "+app.editor.notice);
   java.lang.reflect.Field context=ParkingCore.Controller.class.getDeclaredField("localBoundary");context.setAccessible(true);
   check(context.getBoolean(app.session.trial.controller)&&app.session.config.getString("boundary_evidence").equals("local_obstacle_points"),"native custom run retained straight-line memory mode");
   check(Files.isRegularFile(draft),"map not saved");
   click(robot,origin,894,821); // 暂停后保存真实窗口，避免截图随探索改变。
   app.saveFrame(root.resolve(".cache/map-editor-window.png").toString());Thread.sleep(300);
   SimulationRunner previous=app.session;click(robot,origin,419,821);
   check(app.editor.randomFrame&&app.session==previous&&app.editor.notice.contains("中心停车框"),"blocked centre frame did not roll back switch / preserve run: "+app.editor.notice);
   click(robot,origin,599,821);check(!app.editor.enabled&&!app.session.plant.arena.custom&&app.session.plant.arena.shape.equals("rectangle"),"editor off did not restore rectangle");
   check(!context.getBoolean(app.session.trial.controller),"native rectangle run retained complex-mode context");
   click(robot,origin,419,821);check(!app.editor.randomFrame&&app.session.plant.tx==0&&app.session.plant.ty==0,"random switch off did not restore centre");
   click(robot,origin,599,821);click(robot,origin,174,821);click(robot,origin,599,821);
   check(!app.editor.enabled&&app.editor.metres==2&&app.session.plant.arena.w==2000&&!app.session.plant.arena.custom,"closing editor left staged size inconsistent");
   System.out.println("PASS native mouse controls: size, random frame, circle, ellipse, freehand concave boundary, wall, undo, apply, save, invalid-layout rollback, independent switches off");
  }catch(Throwable failure){failed=true;app.saveFrame(root.resolve(".cache/map-editor-failure.png").toString());Thread.sleep(400);failure.printStackTrace();}
  finally{try{Files.deleteIfExists(draft);app.dispose();}finally{System.exit(failed?1:0);}}
 }
}
