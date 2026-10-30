/**
 * 文件职责：触边后控制轮子。当前默认 vector：先离边到安全距离，再原地转到目标方向，从新位置沿该方向出发；
 * 自定义地图沿用原条件动作（小夹角单轮调整，其余制动转向，受阻回退）。
 * 输入：目标方向、触边边线估计、灰色格与走过的格子、编码器；输出：左右轮指令。
 * 调用关系：CompactSearch 调用；读取 EdgeEstimate 和触边网络权重。
 */
import java.io.*;import java.nio.file.*;
/** Conditional outer-wheel hold and inner-wheel reverse. Network gets two headings only. */
public final class BoundaryMotion {
 public final EdgeEstimate edge=new EdgeEstimate();
 final String mode;
 final double threshold=Double.parseDouble(System.getProperty("parking.parallelDegrees","25"));
 public boolean active=false,outerLeft=false;public int phase=0,starts=0,completed=0,lastEvent=-1;
 public double original=0,target=0,pivotYaw=0,advanceStart=0,holdCount=0,holdMaximum=0;
 public double speed=100,tilt=20,advance=50;public long startMs=0,resumeMillis=2000;Network network;
 long lastPivotProgress=0,progressLeft=0,progressRight=0;public int blockedPivotRecoveries=0;
 // "vector"：主模型方向当作向量保留，起点可平移。先离边到中心距边 safeMm（原地转时轮子绕中心画约50毫米的圆，
 // 中心够远则任何朝向都安全），再原地转到目标方向，从新位置沿该方向出发，不回原目标点。
 // 正面或斜撞：沿来路直线倒车（刚走过，必在桌内）；贴边：外侧轮不动、内侧轮后退转向朝内（任何轮不比外侧轮更靠外），再前进离边。
 final boolean vector;
 final double safeMm=Double.parseDouble(System.getProperty("parking.vectorSafeMm","90"));
 final double steep=Double.parseDouble(System.getProperty("parking.vectorSteep","0.35"));
 final double pivotIn=Math.sin(Math.toRadians(Double.parseDouble(System.getProperty("parking.vectorPivotDeg","30"))));
 static final double MAX_REVERSE=300,MAX_FORWARD=200;
 double remaining=0,escapeStart=0;
 public int reverses=0,pivots=0,directSpins=0,aborts=0;boolean restart=false;
 public BoundaryMotion(){this(false);}
 /** 自定义地图（曲线、墙）未验证向量动作，沿用原条件动作。 */
 public BoundaryMotion(boolean localBoundary){
  String requested=System.getProperty("parking.boundaryMotion","baseline");
  mode=requested.equals("vector")&&localBoundary?"learned":requested;vector=mode.equals("vector");
  if(mode.equals("learned"))try{network=new Network(Paths.get(System.getProperty("parking.motionPolicy")));}catch(IOException e){throw new RuntimeException(e);}
 }
 public void observe(CompactSearch m,ParkingCore.Observation o){edge.observe(m,o);}
 public void begin(CompactSearch m,boolean wasEdge,long now){
  if(vector){
   // 只在真实触边后启动；已知直边的提前打断没有灰色读数，不需要离边动作。
   if(active||!wasEdge||m.edges==lastEvent&&!restart||m.lastMs-m.lastGrey>300)return;
   lastEvent=m.edges;restart=false;edge.fit(m);
   target=Math.atan2(m.goalY-m.pose.y,m.goalX-m.pose.x);remaining=Math.hypot(m.goalX-m.pose.x,m.goalY-m.pose.y);
   startMs=now;resumeMillis=Math.max(10,m.actionUntil-now);phase=10;active=true;starts++;return;
  }
  if(mode.equals("baseline")||active||!wasEdge||m.edges==lastEvent)return;
  lastEvent=m.edges;edge.fit(m);
  beginEstimated(m,now);
  // 不确定或不满足小夹角条件，也不能退回带前进速度的圆弧。
  // 目标点在桌内只说明终点可行，不说明转到这个方向的过程可行。
  if(!active){original=m.pose.a;target=Math.atan2(m.goalY-m.pose.y,m.goalX-m.pose.x);
   startMs=now;resumeMillis=Math.max(10,m.actionUntil-now);phase=3;active=true;starts++;}
 }
 /** Replay uses the sensor-derived estimate captured at contact, never the true edge. */
 public void beginEstimated(CompactSearch m,long now){
  if(mode.equals("baseline")||!edge.reliable||!mode.equals("always")&&edge.alpha>threshold)return;
  original=m.pose.a;target=Math.atan2(m.goalY-m.pose.y,m.goalX-m.pose.x);
  // An inward-facing robot and a symmetric head-on contact need no one-wheel manoeuvre.
  double lateral=-edge.nx*Math.sin(original)+edge.ny*Math.cos(original);
  if(!mode.equals("always")&&(edge.nx*Math.cos(original)+edge.ny*Math.sin(original)<=0||Math.abs(lateral)<Math.sin(Math.toRadians(3))))return;
  if(network!=null){double[] p=network.predict(target,original);speed=p[0];tilt=p[1];advance=p[2];}
  else{speed=Double.parseDouble(System.getProperty("parking.pivotSpeed","100"));tilt=Double.parseDouble(System.getProperty("parking.pivotTilt","20"));advance=Double.parseDouble(System.getProperty("parking.pivotAdvance","50"));}
  outerLeft=Math.abs(lateral)>.04?lateral>0:ParkingCore.wrap(target-original)<0;
  double sign=outerLeft?-1:1;
  pivotYaw=ParkingCore.wrap(Math.atan2(edge.ny,edge.nx)+sign*Math.toRadians(90+tilt));
  holdCount=outerLeft?m.left:m.right;holdMaximum=0;startMs=now;resumeMillis=Math.max(10,m.actionUntil-now);phase=1;active=true;starts++;
  lastPivotProgress=now;progressLeft=m.left;progressRight=m.right;
 }
 double edgeDistance(CompactSearch m){return edge.offset-(edge.nx*m.pose.x+edge.ny*m.pose.y);}
 /** 中心到最近边界证据的距离：可靠时的触边直线，以及所有见过灰色的格子（角落的另一条边也计入）。 */
 double clearance(CompactSearch m){
  double best=edge.reliable?edgeDistance(m):1e9,r=safeMm+CompactSearch.CELL;
  for(double dx=-r;dx<=r;dx+=CompactSearch.CELL)for(double dy=-r;dy<=r;dy+=CompactSearch.CELL)
   if(CompactSearch.bit(m.blocked,m.fine(m.pose.x+dx,m.pose.y+dy)))best=Math.min(best,Math.hypot(dx,dy)-CompactSearch.CELL/2);
  return best;
 }
 /** 倒车只沿走过的路：车尾方向60毫米处须是车身中心走过的格子，否则后方未知（传感器都在车头）。 */
 boolean behindVisited(CompactSearch m){return CompactSearch.bit(m.body,m.fine(m.pose.x-60*Math.cos(m.pose.a),m.pose.y-60*Math.sin(m.pose.a)));}
 long forwardStartMs=0;
 ParkingCore.Command vectorCommand(CompactSearch m){
  if(m.lastMs-startMs>8000){m.invalid=true;m.guard="BOUNDARY_MOTION_STALL";return CompactSearch.velocity(0,0);}
  double wheelSpeed=Math.max(Math.abs(m.filteredV-45*m.filteredW),Math.abs(m.filteredV+45*m.filteredW));
  // 动作途中触到新的边（如角落另一条边）：原方向依据已过时，停止并让主模型按新证据重新选择，开始新的离边动作。
  if(phase!=10&&(m.edges!=lastEvent||phase==13&&m.lastGrey>=forwardStartMs)){active=false;phase=0;aborts++;restart=true;m.actionUntil=-1;m.edgeChoice=true;m.turnPhase=0;return CompactSearch.velocity(0,0);}
  if(phase==10){
   m.guard="VECTOR_BRAKE";if(wheelSpeed>10)return CompactSearch.velocity(0,0);
   original=m.pose.a;escapeStart=m.signedDistance;
   if(clearance(m)>=safeMm){phase=14;directSpins++;}
   else if(!edge.reliable||edge.nx*Math.cos(m.pose.a)+edge.ny*Math.sin(m.pose.a)>=steep){phase=11;reverses++;}
   else{
    double lateral=-edge.nx*Math.sin(m.pose.a)+edge.ny*Math.cos(m.pose.a);
    outerLeft=Math.abs(lateral)>.04?lateral>0:ParkingCore.wrap(target-m.pose.a)<0;
    holdCount=outerLeft?m.left:m.right;holdMaximum=0;phase=12;pivots++;
   }
  }
  if(phase==11){
   double back=escapeStart-m.signedDistance;
   boolean stop=clearance(m)>=safeMm||back>=MAX_REVERSE||!behindVisited(m);
   if(!stop){m.guard="VECTOR_REVERSE";return CompactSearch.velocity(-100,ParkingCore.clip(2.5*ParkingCore.wrap(original-m.pose.a),-1,1));}
   phase=14;
  }
  if(phase==12){
   double inward=-(edge.nx*Math.cos(m.pose.a)+edge.ny*Math.sin(m.pose.a));
   if(inward<pivotIn){
    double d=((outerLeft?m.left:m.right)-holdCount)*ParkingCore.MM_COUNT;holdMaximum=Math.max(holdMaximum,Math.abs(d));
    double v=m.filteredV+(outerLeft?-45:45)*m.filteredW,held=ParkingCore.clip(-8*d-.8*v,-200,200);
    double inner=-Math.min(100,Math.max(35,160*(pivotIn-inward)));
    m.guard=outerLeft?"VECTOR_HOLD_LEFT":"VECTOR_HOLD_RIGHT";
    return new ParkingCore.Command((outerLeft?held:inner)/1.25,(outerLeft?inner:held)/1.25);
   }
   phase=13;escapeStart=m.signedDistance;original=m.pose.a;forwardStartMs=m.lastMs;
  }
  if(phase==13){
   if(clearance(m)<safeMm&&m.signedDistance-escapeStart<MAX_FORWARD){m.guard="VECTOR_FORWARD";return CompactSearch.velocity(120,ParkingCore.clip(2.5*ParkingCore.wrap(original-m.pose.a),-1,1));}
   phase=14;
  }
  if(phase==14){
   double e=ParkingCore.wrap(target-m.pose.a);
   if(Math.abs(e)>Math.toRadians(5)){m.guard="VECTOR_SPIN";if(Math.abs(m.filteredV)>15)return CompactSearch.velocity(0,0);return CompactSearch.velocity(0,ParkingCore.clip(2.5*e,-1.5,1.5));}
   // 向量语义：从当前位置沿目标方向走剩余长度，起点不必与原路径对齐。
   active=false;phase=0;completed++;
   m.pathStartX=m.pose.x;m.pathStartY=m.pose.y;m.goalX=m.pose.x+remaining*Math.cos(target);m.goalY=m.pose.y+remaining*Math.sin(target);
   m.actionUntil=m.lastMs+resumeMillis;m.turnPhase=0;return null;
  }
  return null;
 }
 public ParkingCore.Command command(CompactSearch m){
  if(!active)return null;
  if(vector)return vectorCommand(m);
  // 曲线或邻墙可能使单轮动作受阻；用编码器检验实际进展，不等待8秒后判失败。
  if(m.localBoundary&&phase==1){
   if(m.left!=progressLeft||m.right!=progressRight){lastPivotProgress=m.lastMs;progressLeft=m.left;progressRight=m.right;}
   else if(m.lastMs-lastPivotProgress>=400){phase=3;startMs=m.lastMs;blockedPivotRecoveries++;}
  }
  if(m.lastMs-startMs>8000){m.invalid=true;m.guard="BOUNDARY_MOTION_STALL";return CompactSearch.velocity(0,0);}
  if(phase==3){
   m.guard="BOUNDARY_BRAKE";
   // 编码器估计两个轮子的残余速度；PWM归零不会使正在前进的车瞬间停止。
   if(Math.max(Math.abs(m.filteredV-45*m.filteredW),Math.abs(m.filteredV+45*m.filteredW))>10)return CompactSearch.velocity(0,0);
   phase=4;
  }
  if(phase==4){
   double e=ParkingCore.wrap(Math.atan2(m.goalY-m.pose.y,m.goalX-m.pose.x)-m.pose.a);
   if(Math.abs(e)>Math.toRadians(5)){m.guard="BOUNDARY_DIRECT_TURN";return CompactSearch.velocity(0,ParkingCore.clip(2.5*e,-1.5,1.5));}
   active=false;phase=0;completed++;m.actionUntil=m.lastMs+resumeMillis;m.turnPhase=0;return null;
  }
  double d=((outerLeft?m.left:m.right)-holdCount)*ParkingCore.MM_COUNT;
  if(phase==1){
   holdMaximum=Math.max(holdMaximum,Math.abs(d));
   double e=ParkingCore.wrap(pivotYaw-m.pose.a);
   if(Math.abs(e)<Math.toRadians(5)){phase=2;advanceStart=m.signedDistance;}
   else{
    // Position feedback counters residual actuator coasting; zero PWM alone is not a lock.
    double v=m.filteredV+(outerLeft?-45:45)*m.filteredW;
    double held=ParkingCore.clip(-8*d-.8*v,-200,200);
    double inner=-Math.min(speed,Math.max(35,160*Math.abs(e)));
    m.guard=outerLeft?"HOLD_LEFT_REVERSE_RIGHT":"HOLD_RIGHT_REVERSE_LEFT";
    return new ParkingCore.Command((outerLeft?held:inner)/1.25,(outerLeft?inner:held)/1.25);
   }
  }
  if(phase==2){
   if(m.signedDistance-advanceStart<advance){m.guard="BOUNDARY_INWARD_REJOIN";return CompactSearch.velocity(120,ParkingCore.clip(2.5*ParkingCore.wrap(pivotYaw-m.pose.a),-1.3,1.3));}
   active=false;phase=0;completed++;m.actionUntil=m.lastMs+resumeMillis;
   m.turnPhase=Math.abs(ParkingCore.wrap(Math.atan2(m.goalY-m.pose.y,m.goalX-m.pose.x)-m.pose.a))>Math.toRadians(135)?1:0;
  }
  return null;
 }
 public static final class Network {
  final float[] w1=new float[24],b1=new float[12],w2=new float[36],b2=new float[3];
  public Network(Path p)throws IOException{try(DataInputStream in=new DataInputStream(Files.newInputStream(p))){if(in.readInt()!=0x424d4d31)throw new IOException("Motion model format");for(float[] a:new float[][]{w1,b1,w2,b2})for(int i=0;i<a.length;i++){a[i]=in.readFloat();if(!Float.isFinite(a[i]))throw new IOException("Non-finite motion weight");}if(in.read()!=-1)throw new IOException("Trailing motion weights");}}
  public double[] predict(double chosenHeading,double originalHeading){
   double delta=ParkingCore.wrap(chosenHeading-originalHeading);double[] f={Math.cos(delta),Math.abs(Math.sin(delta))},h=new double[12],p=new double[3];
   for(int i=0;i<12;i++)h[i]=Math.tanh(w1[2*i]*f[0]+w1[2*i+1]*f[1]+b1[i]);
   for(int j=0;j<3;j++){double z=b2[j];for(int i=0;i<12;i++)z+=w2[12*j+i]*h[i];p[j]=1/(1+Math.exp(-z));}
   p[0]=50+110*p[0];p[1]=8+32*p[1];p[2]=p[2]<.05?0:100*p[2];return p;
  }
 }
}
