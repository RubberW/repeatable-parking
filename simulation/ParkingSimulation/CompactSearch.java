/**
 * 文件职责：探索主模型：编码器里程计、观测特征、访问记忆和子目标执行
 * 输入：反射率、编码器、固定权重与用户选择的边界处理模式；输出：目标方向与轮子指令。
 * 调用关系：ParkingCore 调用；内部使用 SearchDecision、HeadingGate、BoundaryMotion。
 */
import java.io.*;
import java.nio.file.*;
import java.util.*;

/** Sparse surface memory in encoder coordinates. No simulator/target/map access. */
public final class CompactSearch {
 public final SearchMemory memory;
 public final HeadingGate headingGate;
 public final BoundaryMotion boundaryMotion;
 public final boolean localBoundary;
 public CompactSearch(){this(false);}
 public CompactSearch(boolean localBoundary){this.localBoundary=localBoundary;memory=new SearchMemory(!localBoundary);headingGate=new HeadingGate(localBoundary);boundaryMotion=new BoundaryMotion(localBoundary);}
 boolean edgeLatched=false; long lastGrey=-10000;
 public static final int SIDE=Integer.getInteger("parking.memorySide",192),COARSE=SIDE/4,MAP=294,ACTIONS=32,FEATURES=27,OBS=MAP+ACTIONS*FEATURES;
 public static final double CELL=50,SPEED=200;
 final byte[] seen=new byte[SIDE*SIDE/8],body=new byte[SIDE*SIDE/8],blocked=new byte[SIDE*SIDE/8];
 final byte[] visits=new byte[COARSE*COARSE],sampled=new byte[COARSE*COARSE],coarseBlocked=new byte[(COARSE*COARSE+7)/8];
 final int[] lastVisit=new int[COARSE*COARSE];
 final double[] hx=new double[16],hy=new double[16],hd=new double[16];
 int historyN=0,lastCell=-1,lastRegion=-1;long historySecond=-1;
 public final ParkingCore.Pose pose=new ParkingCore.Pose(0,0,0);
 public long lastMs=-1,left,right,actionUntil=-1;
 public int unique=0,regions=0,edges=0,action=-1,decisions=0;
 public double signedDistance=0,filteredV=0,filteredW=0; public int turnPhase=0;
 public double distance=0,repeatedDistance=0,lastVelocity=0,loopProgress=1;
 double goalX=0,goalY=0,pathStartX=0,pathStartY=0,outwardX=1,outwardY=0;
 boolean repeatedCell=false,edgeChoice=false;
 public boolean contact=false,invalid=false;
 public String guard="CLEAR";
 static boolean bit(byte[] b,int k){return k>=0 && (b[k>>3]&(1<<(k&7)))!=0;}
 static void set(byte[] b,int k){if(k>=0)b[k>>3]|=(byte)(1<<(k&7));}
 static int index(double x,double y,int side,double cell){int a=(int)Math.floor(x/cell+side/2),b=(int)Math.floor(y/cell+side/2);return a<0||b<0||a>=side||b>=side?-1:b*side+a;}
 int fine(double x,double y){return index(x,y,SIDE,50);}
 int region(double x,double y){return index(x,y,COARSE,200);}
 public double headingOffset=0;
 double turn(int a){return ParkingCore.wrap((a/2)*Math.PI/8+headingOffset);}
 double length(int a){return headingGate.constrains()?headingGate.lengths[a]:(a%2==0?400:1000);}
 public void observe(ParkingCore.Observation o){
  double ds=0;if(lastMs>=0){double l=(o.leftCount-left)*ParkingCore.MM_COUNT,r=(o.rightCount-right)*ParkingCore.MM_COUNT;ds=(l+r)/2;
   pose.x+=ds*Math.cos(pose.a);pose.y+=ds*Math.sin(pose.a);pose.a=ParkingCore.wrap(pose.a+(r-l)/ParkingCore.B);distance+=Math.abs(ds);signedDistance+=ds;
   double dt=Math.max(.001,(o.millis-lastMs)*.001),blend=1-Math.exp(-dt/.05);filteredV+=blend*(ds/dt-filteredV);filteredW+=blend*((r-l)/ParkingCore.B/dt-filteredW);
   lastVelocity=Math.abs(ds)/Math.max(.001,(o.millis-lastMs)*.001);}
  lastMs=o.millis;left=o.leftCount;right=o.rightCount;
  int f=fine(pose.x,pose.y),c=region(pose.x,pose.y);
  if(f!=lastCell){repeatedCell=bit(body,f);set(body,f);lastCell=f;}if(repeatedCell)repeatedDistance+=Math.abs(ds);
  if(c>=0&&c!=lastRegion){int n=visits[c]&255;if(n==0)regions++;visits[c]=(byte)Math.min(255,n+1);lastRegion=c;}if(c>=0)lastVisit[c]=(int)(lastMs/1000+1);
  double gx=0,gy=0,wx=0,wy=0;int ng=0,nw=0;
  for(int i=0;i<5;i++){
   float v=o.sensor(i);double a=pose.a+Math.toRadians(ParkingCore.SENSOR_ANGLES[i]);double x=pose.x+45*Math.cos(a),y=pose.y+45*Math.sin(a);int k=fine(x,y),r=region(x,y);
   if(v>=1850&&v<=2150)contact=true;
   if(v>=900&&v<=1100){set(blocked,k);set(coarseBlocked,r);gx+=x;gy+=y;ng++;}
   else if((v>=300&&v<=500)||(v>=1850&&v<=2150)){
    if(k>=0&&!bit(seen,k)){set(seen,k);unique++;if(r>=0)sampled[r]=(byte)Math.min(16,(sampled[r]&255)+1);}wx+=x;wy+=y;nw++;
   }else invalid=true;
  }
  {
   if(ng>0){
    if(!edgeLatched){edges++;actionUntil=-1;edgeLatched=true;
     outwardX=gx/ng-(nw>0?wx/nw:pose.x);outwardY=gy/ng-(nw>0?wy/nw:pose.y);double norm=Math.hypot(outwardX,outwardY);
     if(norm>1){outwardX/=norm;outwardY/=norm;}else{outwardX=Math.cos(pose.a);outwardY=Math.sin(pose.a);}
    }
    lastGrey=o.millis;edgeChoice=true;guard="POLICY_BOUNDARY";
    // Reconsider while outside, without fixing a direction, reversing or stopping.
    if(actionUntil>o.millis+2000)actionUntil=o.millis+2000;
   }else if(o.millis-lastGrey>=300)edgeLatched=false;
  }
  memory.observe(this,o);
  headingGate.observe(this,o);
  boundaryMotion.observe(this,o);
  long second=lastMs/1000;if(second!=historySecond){int h=historyN%16;
   if(historyN>=16){double travelled=distance-hd[h];loopProgress=travelled>200?Math.min(1,Math.hypot(pose.x-hx[h],pose.y-hy[h])/travelled):1;}
   hx[h]=pose.x;hy[h]=pose.y;hd[h]=distance;historyN++;historySecond=second;}
 }
 public boolean needsAction(long now){return !boundaryMotion.active &&(actionUntil<0||now>=actionUntil||Math.hypot(goalX-pose.x,goalY-pose.y)<25);}
 public void select(int a,long now){if(!headingGate.allowed(this,a)){invalid=true;guard="DIRECTION_REJECTED_STOP";return;}boolean wasEdge=edgeChoice;action=a;double angle=pose.a+turn(a);pathStartX=pose.x;pathStartY=pose.y;goalX=pose.x+length(a)*Math.cos(angle);goalY=pose.y+length(a)*Math.sin(angle);actionUntil=now+(edgeLatched?2000:8000);edgeChoice=false;turnPhase=Math.abs(turn(a))>Math.toRadians(135)?1:0;decisions++;boundaryMotion.begin(this,wasEdge,now);}
 public ParkingCore.Command command(){
  if(contact||invalid)return velocity(0,0);
  ParkingCore.Command pivot=boundaryMotion.command(this);if(pivot!=null)return pivot;
  if(needsAction(lastMs))return velocity(0,0);
  guard="CLEAR";double e=ParkingCore.wrap(Math.atan2(goalY-pose.y,goalX-pose.x)-pose.a);
  double px=goalX-pathStartX,py=goalY-pathStartY,len=Math.hypot(px,py);px/=len;py/=len;
  double projection=(pose.x-pathStartX)*px+(pose.y-pathStartY)*py;
  double lookahead=Math.min(len,150+.3*Math.max(0,filteredV)),carrot=ParkingCore.clip(projection+lookahead,lookahead,len);
  e=ParkingCore.wrap(Math.atan2(pathStartY+py*carrot-pose.y,pathStartX+px*carrot-pose.x)-pose.a);
  return steering(e);
 }
 // Curvature-regulated heading pursuit. Continuous forward arcs for <=135 degrees.
 // For a near reversal, first brake translation, then spin, then rejoin an arc.
 public ParkingCore.Command steering(double e){
  if(turnPhase==1){if(Math.abs(filteredV)>15)return velocity(0,0);turnPhase=2;}
  if(turnPhase==2){if(Math.abs(e)>Math.toRadians(60))return velocity(0,ParkingCore.clip(2.5*e,-1.5,1.5));turnPhase=0;}
  double w=ParkingCore.clip(2.5*e,-1.5,1.5);
  double v=Math.min(SPEED*Math.pow(Math.cos(e/2),2),250-45*Math.abs(w));
  // Limit v*|omega| as an assumed lateral-acceleration budget (not measured grip).
  if(Math.abs(w)>1e-6)v=Math.min(v,180/Math.abs(w));
  return velocity(Math.max(0,v),w);
 }

 static ParkingCore.Command velocity(double v,double w){return new ParkingCore.Command((v-45*w)/1.25,(v+45*w)/1.25);}
 double channel(double x,double y,int type,boolean coarse){int f=fine(x,y),c=region(x,y);if(c<0||f<0)return type==2?1:0;
  if(coarse)return type==0?Math.min(1,(visits[c]&255)/4.0):type==1?(sampled[c]&255)/16.0:bit(coarseBlocked,c)?1:0;
  return bit(type==0?body:type==1?seen:blocked,f)?1:0;}
 public float[] features(){
  float[] out=new float[OBS];double ca=Math.cos(pose.a),sa=Math.sin(pose.a);int k=0;
  // Complete coarse/local square patches, pooled from sparse evidence, never simulator pixels.
  for(int scale=0;scale<2;scale++){double step=scale==0?100:400;
   for(int y=-3;y<=3;y++)for(int x=-3;x<=3;x++)for(int ch=0;ch<3;ch++){
    double sum=0;for(double dx:new double[]{-.25,.25})for(double dy:new double[]{-.25,.25}){double u=(x+dx)*step,v=(y+dy)*step;sum+=channel(pose.x+ca*u-sa*v,pose.y+sa*u+ca*v,ch,scale==1);}out[k++]=(float)(sum/4);}}
  for(int a=0;a<ACTIONS;a++){
   k=MAP+a*FEATURES;double angle=pose.a+turn(a),cs=Math.cos(angle),sn=Math.sin(angle),len=length(a);
   for(int b=0;b<4;b++){
    double d=len*(b+1)/4;double bodyN=0,seenN=0,blockedN=0,regionN=0;
    for(double side:new double[]{-60,0,60}){double x=pose.x+d*cs-side*sn,y=pose.y+d*sn+side*cs;bodyN+=channel(x,y,0,false);seenN+=channel(x,y,1,false);blockedN+=channel(x,y,2,false);regionN+=channel(x,y,0,true);}
    out[k+b*4]=(float)(bodyN/3);out[k+b*4+1]=(float)(seenN/3);out[k+b*4+2]=(float)(blockedN/3);out[k+b*4+3]=(float)(regionN/3);
   }
   out[k+16]=(float)(Math.abs(turn(a))/Math.PI);out[k+17]=(float)(len/1000);out[k+18]=(float)Math.sin(turn(a));out[k+19]=(float)loopProgress;
   out[k+20]=(float)(cs*outwardX+sn*outwardY);out[k+21]=edgeChoice?1:0;
   int r=region(pose.x+len*cs,pose.y+len*sn);out[k+22]=r<0?1:lastVisit[r]==0?0:(float)Math.max(0,1-(lastMs*.001-lastVisit[r]+1)/60);
   out[k+23]=(float)ParkingCore.clip(filteredV/200,-1,1);out[k+24]=(float)ParkingCore.clip(filteredW/1.5,-1,1);out[k+26]=0;
  }
  return out;
 }

 public static final class Network {
  public final float[] ew=new float[MAP*16],eb=new float[16],aw=new float[43*16],ab=new float[16],ow=new float[16];public float ob;
  public Network(Path file)throws IOException{try(DataInputStream in=new DataInputStream(Files.newInputStream(file))){if(in.readInt()!=0x52535031)throw new IOException("Compact policy format mismatch");for(float[] w:new float[][]{ew,eb,aw,ab,ow})for(int i=0;i<w.length;i++)w[i]=in.readFloat();ob=in.readFloat();}}
  public double[] scores(float[] obs){double[] context=new double[16],result=new double[ACTIONS];
   for(int o=0;o<16;o++){double v=eb[o];for(int i=0;i<MAP;i++)v+=ew[o*MAP+i]*obs[i];context[o]=Math.tanh(v);}
   for(int a=0;a<ACTIONS;a++){int k=MAP+a*FEATURES;if(obs[k+26]>0){result[a]=-1e9;continue;}double score=ob;
    for(int o=0;o<16;o++){double v=ab[o];for(int i=0;i<16;i++)v+=aw[o*43+i]*context[i];for(int i=0;i<27;i++)v+=aw[o*43+16+i]*obs[k+i];score+=ow[o]*Math.tanh(v);}result[a]=score;}
   return result;
  }
  public int choose(float[] obs,Random random,boolean deterministic){return choose(obs,null,random,deterministic);}
  public int choose(float[] obs,double[] bias,Random random,boolean deterministic){double[] s=scores(obs);if(bias!=null)for(int i=0;i<ACTIONS;i++)if(s[i]>-1e8)s[i]+=bias[i];int best=0;for(int i=1;i<ACTIONS;i++)if(s[i]>s[best])best=i;
   if(s[best]<=-1e8)return -1;if(deterministic)return best;
   double temperature=Double.parseDouble(System.getProperty("parking.policyTemperature","1"));if(!Double.isFinite(temperature)||temperature<=0)throw new IllegalArgumentException("Policy temperature must be positive");
   double max=s[best],sum=0;for(int i=0;i<ACTIONS;i++){s[i]=obs[MAP+i*FEATURES+26]>0?0:Math.exp((s[i]-max)/temperature);sum+=s[i];}double r=random.nextDouble()*sum;int last=best;for(int i=0;i<ACTIONS;i++){if(s[i]==0)continue;last=i;r-=s[i];if(r<0)return i;}return last;}
 }
}
