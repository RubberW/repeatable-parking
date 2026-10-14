import java.io.*;import java.nio.file.*;
/** Conditional outer-wheel hold and inner-wheel reverse. Network gets two headings only. */
public final class BoundaryMotion {
 public final EdgeEstimate edge=new EdgeEstimate();
 final String mode=System.getProperty("parking.boundaryMotion","baseline");
 final double threshold=Double.parseDouble(System.getProperty("parking.parallelDegrees","25"));
 public boolean active=false,outerLeft=false;public int phase=0,starts=0,completed=0,lastEvent=-1;
 public double original=0,target=0,pivotYaw=0,advanceStart=0,holdCount=0,holdMaximum=0;
 public double speed=100,tilt=20,advance=50;public long startMs=0,resumeMillis=2000;Network network;
 public BoundaryMotion(){if(mode.equals("learned"))try{network=new Network(Paths.get(System.getProperty("parking.motionPolicy")));}catch(IOException e){throw new RuntimeException(e);}}
 public void observe(CompactSearch m,ParkingCore.Observation o){edge.observe(m,o);}
 public void begin(CompactSearch m,boolean wasEdge,long now){
  if(mode.equals("baseline")||active||!wasEdge||m.edges==lastEvent)return;
  lastEvent=m.edges;edge.fit(m);
  beginEstimated(m,now);
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
 }
 public ParkingCore.Command command(CompactSearch m){
  if(!active)return null;
  if(m.lastMs-startMs>8000){m.invalid=true;m.guard="BOUNDARY_MOTION_STALL";return CompactSearch.velocity(0,0);}
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
