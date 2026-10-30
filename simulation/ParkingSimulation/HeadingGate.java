/**
 * 文件职责：普通地图使用桌内凸包约束；复杂地图使用有限局部障碍点判断触边方向。
 * 输入：桌内反射率点和编码器位姿；输出：允许的方向与有限目标距离。
 * 调用关系：CompactSearch 更新，SearchDecision 应用。
 */
import java.util.*;
/** Sensor/encoder-only direction screening. Ordinary convex scenes use the
 * interior witness; complex scenes use expiring local points, without a hull.
 * Neither branch certifies the wheel-swept path.
 */
public final class HeadingGate {
 static final int CAP=256;static final double PAD=8,MAX_AGE_MM=800,MIN_GOAL=40;
 final boolean enabled=Boolean.parseBoolean(System.getProperty("parking.hardHeading","true"));
 final boolean localBoundary;
 // "hull"：目标点须落在最近800毫米的桌内点凸包内（原方案）。
 // "direction"：出发段须在较长路程的桌内点凸包内（凸场地＋里程计误差小的假设），
 // 方向至少朝桌内偏 gateInwardDeg；终点只受已拟合直边限制，不必落在已探索区域内。
 final boolean directionMode=System.getProperty("parking.gate","hull").equals("direction");
 final double inwardDeg=Double.parseDouble(System.getProperty("parking.gateInwardDeg","20"));
 final double memoryMm=Double.parseDouble(System.getProperty("parking.gateMemoryMm","4000"));
 final double departMm=Double.parseDouble(System.getProperty("parking.gateDepartMm","150"));
 static final double LINE_PAD=60;static final int WIDE=1024;
 // 每次可靠触边记下局部边线（沿边线±400毫米有效）。之后任何方向的子目标都截断在离线80毫米处：
 // 外侧传感器横向约32毫米、轮子45毫米，小角度贴边时轮子会先于传感器越线，不能等传感器发现。
 static final double CONTACT_EXTENT=400,CONTACT_PAD=80;
 final ArrayList<double[]> contactLines=new ArrayList<>();
 /** 诊断：最近一次决策采用的约束层。0原凸包规则 1可靠边线 2桌内证据 3原规则兜底 4未触边截断 5未触边不限。 */
 public int branch=0;
 final double[] wx=new double[WIDE],wy=new double[WIDE],wtravel=new double[WIDE],wLastX=new double[5],wLastY=new double[5];int wCount=0,wCursor=0;
 final double[] hx=new double[CAP],hy=new double[CAP],travel=new double[CAP],lastX=new double[5],lastY=new double[5];
 final long[] stamp=new long[CAP];
 final double[] lengths=new double[32];final boolean[] valid=new boolean[32];
 int count=0,cursor=0;long clearSince=-1;boolean active=false,blocked=false;
 ArrayList<double[]> hull=new ArrayList<>();
 // 复杂地图的灰色点只在最近2秒、240毫米内使用，不延伸为直线或半平面。
 final double[] obstacleX=new double[CAP],obstacleY=new double[CAP],obstacleTravel=new double[CAP];
 final long[] obstacleStamp=new long[CAP];int obstacleCount=0,obstacleCursor=0;
 public HeadingGate(){this(false);}
 public HeadingGate(boolean localBoundary){this.localBoundary=localBoundary;Arrays.fill(lastX,1e30);Arrays.fill(lastY,1e30);Arrays.fill(wLastX,1e30);Arrays.fill(wLastY,1e30);}
 static double cross(double[] a,double[] b,double[] c){return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]);}
 void build(CompactSearch m,long now){
  ArrayList<double[]> p=new ArrayList<>();
  for(int i=0;i<count;i++)if(m.distance-travel[i]<=MAX_AGE_MM&&now-stamp[i]<=10000)p.add(new double[]{hx[i],hy[i]});
  if(directionMode)for(int i=0;i<wCount;i++)if(m.distance-wtravel[i]<=memoryMm)p.add(new double[]{wx[i],wy[i]});
  p.sort((a,b)->a[0]==b[0]?Double.compare(a[1],b[1]):Double.compare(a[0],b[0]));
  ArrayList<double[]> lower=new ArrayList<>(),upper=new ArrayList<>();
  for(double[] v:p){while(lower.size()>1&&cross(lower.get(lower.size()-2),lower.get(lower.size()-1),v)<=0)lower.remove(lower.size()-1);lower.add(v);}
  for(int i=p.size()-1;i>=0;i--){double[] v=p.get(i);while(upper.size()>1&&cross(upper.get(upper.size()-2),upper.get(upper.size()-1),v)<=0)upper.remove(upper.size()-1);upper.add(v);}
  hull=lower;if(!hull.isEmpty())hull.remove(hull.size()-1);if(!upper.isEmpty())upper.remove(upper.size()-1);hull.addAll(upper);
 }
 double clearance(double x,double y){
  if(hull.size()<3)return -1e30;double d=1e30;
  for(int i=0;i<hull.size();i++){double[] a=hull.get(i),b=hull.get((i+1)%hull.size());double dx=b[0]-a[0],dy=b[1]-a[1],len=Math.hypot(dx,dy);if(len<1e-8)continue;d=Math.min(d,(dx*(y-a[1])-dy*(x-a[0]))/len);}
  return d;
 }
 double rayLength(CompactSearch m,int action){return rayLength(m,action,PAD);}
 double rayLength(CompactSearch m,int action,double PAD){
  if(hull.size()<3)return 0;double angle=m.pose.a+m.turn(action),ux=Math.cos(angle),uy=Math.sin(angle),lo=MIN_GOAL,hi=action%2==0?400:1000;
  for(int i=0;i<hull.size();i++){
   double[] a=hull.get(i),b=hull.get((i+1)%hull.size());double dx=b[0]-a[0],dy=b[1]-a[1],len=Math.hypot(dx,dy);if(len<1e-8)continue;
   double d=(dx*(m.pose.y-a[1])-dy*(m.pose.x-a[0]))/len,slope=(dx*uy-dy*ux)/len;
   if(Math.abs(slope)<1e-10){if(d<PAD)return 0;}
   else if(slope>0)lo=Math.max(lo,(PAD-d)/slope);else hi=Math.min(hi,(PAD-d)/slope);
  }
  return hi>=lo+1?hi-.5:0;
 }
 void prepare(CompactSearch m){
  int n=0;for(int a=0;a<32;a++){lengths[a]=rayLength(m,a);valid[a]=lengths[a]>0;if(valid[a])n++;}
  if(n==0&&hull.size()>=3){
   double x=0,y=0;for(double[] p:hull){x+=p[0];y+=p[1];}x/=hull.size();y/=hull.size();
   m.headingOffset=ParkingCore.wrap(Math.atan2(y-m.pose.y,x-m.pose.x)-m.pose.a);
   for(int a=0;a<32;a++){lengths[a]=rayLength(m,a);valid[a]=lengths[a]>0;if(valid[a])n++;}
  }
  if(n==0){blocked=true;m.invalid=true;m.guard="DIRECTION_UNCERTAIN_STOP";}
 }
 void observe(CompactSearch m,ParkingCore.Observation o){
  if(localBoundary){observeLocal(m,o);return;}
  if(!enabled||blocked)return;
  boolean edge=false;
  for(int i=0;i<5;i++){
   float v=o.sensor(i);boolean g=v>=900&&v<=1100,w=(v>=300&&v<=500)||(v>=1850&&v<=2150);edge|=g;
   if(w){double a=m.pose.a+Math.toRadians(ParkingCore.SENSOR_ANGLES[i]),x=m.pose.x+45*Math.cos(a),y=m.pose.y+45*Math.sin(a);
    if(Math.hypot(x-lastX[i],y-lastY[i])>=10){int k=cursor++%CAP;hx[k]=x;hy[k]=y;travel[k]=m.distance;stamp[k]=o.millis;count=Math.min(CAP,count+1);lastX[i]=x;lastY[i]=y;}
    if(directionMode&&Math.hypot(x-wLastX[i],y-wLastY[i])>=40){int k=wCursor++%WIDE;wx[k]=x;wy[k]=y;wtravel[k]=m.distance;wCount=Math.min(WIDE,wCount+1);wLastX[i]=x;wLastY[i]=y;}
   }
  }
  if(edge&&!active){active=true;m.actionUntil=-1;}
  if(!active){if(directionMode&&m.needsAction(o.millis))prepareFree(m);return;}
  if(edge)clearSince=-1;else if(clearSince<0)clearSince=o.millis;
  if(directionMode){
   if(clearSince>=0&&o.millis-clearSince>=300){active=false;m.headingOffset=0;return;}
   if(m.needsAction(o.millis)){build(m,o.millis);prepareDirection(m);}
   return;
  }
  if(m.needsAction(o.millis)||clearSince>=0&&o.millis-clearSince>=300){
   build(m,o.millis);
   // Front sensors being inside alone does not certify the body origin.
   if(clearSince>=0&&o.millis-clearSince>=300&&clearance(m.pose.x,m.pose.y)>=PAD){active=false;m.headingOffset=0;return;}
   if(m.needsAction(o.millis))prepare(m);
  }
 }
 double cappedLength(CompactSearch m,double ux,double uy,int action){
  double length=action%2==0?400:1000;
  SearchMemory s=m.memory;
  for(int j=0;j<s.lines;j++){
   double toward=ux*s.nx[j]+uy*s.ny[j];if(toward<=1e-9)continue;
   length=Math.min(length,(s.b[j]-LINE_PAD-(s.nx[j]*m.pose.x+s.ny[j]*m.pose.y))/toward);
  }
  for(double[] c:contactLines){
   double toward=ux*c[2]+uy*c[3];if(toward<=1e-9)continue;
   double t=(c[4]-CONTACT_PAD-(c[2]*m.pose.x+c[3]*m.pose.y))/toward;
   double qx=m.pose.x+Math.max(0,t)*ux-c[0],qy=m.pose.y+Math.max(0,t)*uy-c[1];
   if(Math.abs(-c[3]*qx+c[2]*qy)<=CONTACT_EXTENT)length=Math.min(length,t);
  }
  return length>=MIN_GOAL?length:0;
 }
 /** 未触边时：仍按已知直边和局部触边线截断子目标；全部被截断时不加限制，沿用原方案。 */
 void prepareFree(CompactSearch m){
  int n=0;for(int a=0;a<32;a++){double angle=m.pose.a+m.turn(a);lengths[a]=cappedLength(m,Math.cos(angle),Math.sin(angle),a);valid[a]=lengths[a]>0;if(valid[a])n++;}
  branch=4;
  if(n==0){branch=5;for(int a=0;a<32;a++){lengths[a]=a%2==0?400:1000;valid[a]=true;}}
 }
 /** 可靠直边：方向须对拟合相容范围内每个可能法向都朝内 inwardDeg（余量=半范围+inwardDeg）；
  * 出发段中线及两轮位置不经过任何见过灰色的格子（覆盖角落处单线拟合失效的情况）。 */
 double edgeDirectionLength(CompactSearch m,int action,EdgeEstimate edge){
  double angle=m.pose.a+m.turn(action),ux=Math.cos(angle),uy=Math.sin(angle);
  if(-(ux*edge.nx+uy*edge.ny)<Math.sin(Math.toRadians(Math.min(80,edge.spread/2+inwardDeg))))return 0;
  for(double d=20;d<=departMm;d+=20)for(double side=-45;side<=45;side+=45)
   if(CompactSearch.bit(m.blocked,m.fine(m.pose.x+d*ux-side*uy,m.pose.y+d*uy+side*ux)))return 0;
  return cappedLength(m,ux,uy,action);
 }
 /** 边线不可靠：不知道边在哪里时终点不能自由，整段子目标须在桌内证据凸包内（原规则，证据范围更大）。 */
 double evidenceDirectionLength(CompactSearch m,int action){
  // 车身半宽约45毫米：目标中心须离证据边界 LINE_PAD，而不是原规则的8毫米。
  double angle=m.pose.a+m.turn(action),length=rayLength(m,action,LINE_PAD);
  return length>0?Math.min(length,cappedLength(m,Math.cos(angle),Math.sin(angle),action)):0;
 }
 void prepareDirection(CompactSearch m){
  EdgeEstimate edge=m.boundaryMotion.edge;edge.fit(m);int n=0;
  if(edge.reliable){contactLines.add(new double[]{m.pose.x,m.pose.y,edge.nx,edge.ny,edge.offset});if(contactLines.size()>64)contactLines.remove(0);}
  branch=1;
  if(edge.reliable)for(int a=0;a<32;a++){lengths[a]=edgeDirectionLength(m,a,edge);valid[a]=lengths[a]>0;if(valid[a])n++;}
  if(n==0){branch=2;for(int a=0;a<32;a++){lengths[a]=evidenceDirectionLength(m,a);valid[a]=lengths[a]>0;if(valid[a])n++;}}
  // 证据仍不足时完全沿用原终点规则（凸包为原方案的超集），不在原方案能继续的地方停车。
  if(n==0){branch=3;prepare(m);}
 }
 void observeLocal(CompactSearch m,ParkingCore.Observation o){
  if(!enabled||blocked)return;
  int grey=0,white=0;double gx=0,gy=0,wx=0,wy=0;
  for(int i=0;i<5;i++){
   float v=o.sensor(i);double a=m.pose.a+Math.toRadians(ParkingCore.SENSOR_ANGLES[i]),x=m.pose.x+45*Math.cos(a),y=m.pose.y+45*Math.sin(a);
   if(v>=900&&v<=1100){
    grey++;gx+=x;gy+=y;int k=obstacleCursor++%CAP;obstacleCount=Math.min(CAP,obstacleCount+1);
    obstacleX[k]=x;obstacleY[k]=y;obstacleTravel[k]=m.distance;obstacleStamp[k]=o.millis;
   }else if(v>=300&&v<=500||v>=1850&&v<=2150){
    white++;wx+=x;wy+=y;
    if(Math.hypot(x-lastX[i],y-lastY[i])>=10){int k=cursor++%CAP;hx[k]=x;hy[k]=y;travel[k]=m.distance;stamp[k]=o.millis;count=Math.min(CAP,count+1);lastX[i]=x;lastY[i]=y;}
   }
  }
  if(grey>0){
   clearSince=-1;
   // 按这一帧触到的局部位置更新离开方向，不沿用另一段曲线或另一道墙的法向。
   double x=gx/grey-(white>0?wx/white:m.pose.x),y=gy/grey-(white>0?wy/white:m.pose.y),n=Math.hypot(x,y);
   if(n>1){m.outwardX=x/n;m.outwardY=y/n;}
   if(!active){active=true;m.actionUntil=-1;}
  }else if(active){
   if(clearSince<0)clearSince=o.millis;
   if(o.millis-clearSince>=300){active=false;m.headingOffset=0;return;}
  }
  if(active&&m.needsAction(o.millis))prepareLocal(m);
 }
 double localRayLength(CompactSearch m,int action){
  double angle=m.pose.a+m.turn(action),ux=Math.cos(angle),uy=Math.sin(angle);
  if(ux*m.outwardX+uy*m.outwardY>.15)return 0;
  double length=action%2==0?120:240;
  for(double d=10;d<=length;d+=10){
   double x=m.pose.x+d*ux,y=m.pose.y+d*uy;
   for(int i=0;i<obstacleCount;i++)if(m.lastMs-obstacleStamp[i]<=2000&&m.distance-obstacleTravel[i]<=240){
    double dx=x-obstacleX[i],dy=y-obstacleY[i];if(dx*dx+dy*dy<400)return d-15>=MIN_GOAL?d-15:0;
   }
  }return length;
 }
 void prepareLocal(CompactSearch m){
  int n=0;for(int a=0;a<32;a++){lengths[a]=localRayLength(m,a);valid[a]=lengths[a]>=MIN_GOAL;if(valid[a])n++;}
  if(n==0){
   double x=0,y=0;int samples=0;
   for(int i=0;i<count;i++)if(m.lastMs-stamp[i]<=4000&&m.distance-travel[i]<=400){x+=hx[i];y+=hy[i];samples++;}
   // 正观测仅用于旋转候选方向网格，不填充凸包，也不宣称两点之间全部可走。
   if(samples>0){m.headingOffset=ParkingCore.wrap(Math.atan2(y/samples-m.pose.y,x/samples-m.pose.x)-m.pose.a);
    for(int a=0;a<32;a++){lengths[a]=localRayLength(m,a);valid[a]=lengths[a]>=MIN_GOAL;if(valid[a])n++;}
   }
  }
  if(n==0){blocked=true;m.invalid=true;m.guard="LOCAL_DIRECTION_UNCERTAIN_STOP";}
 }
 boolean constrains(){return enabled&&(active||directionMode&&!localBoundary);}
 boolean allowed(CompactSearch m,int a){return a>=0&&a<32&&(!constrains()||!blocked&&valid[a]);}
 void mask(CompactSearch m,float[] f){if(constrains())for(int a=0;a<32;a++)if(!allowed(m,a))f[CompactSearch.MAP+a*CompactSearch.FEATURES+26]=1;}
}
