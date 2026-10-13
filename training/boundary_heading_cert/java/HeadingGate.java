import java.util.*;
/** Sensor/encoder-only interior witness for convex tables. A straight requested
 * subgoal ends in an eroded hull of recent table samples. Not a wheel-arc check.
 */
public final class HeadingGate {
 static final int CAP=256;static final double PAD=8,MAX_AGE_MM=800,MIN_GOAL=40;
 final boolean enabled=Boolean.parseBoolean(System.getProperty("parking.hardHeading","true"));
 final double[] hx=new double[CAP],hy=new double[CAP],travel=new double[CAP],lastX=new double[5],lastY=new double[5];
 final long[] stamp=new long[CAP];
 final double[] lengths=new double[32];final boolean[] valid=new boolean[32];
 int count=0,cursor=0;long clearSince=-1;boolean active=false,blocked=false;
 ArrayList<double[]> hull=new ArrayList<>();
 public HeadingGate(){Arrays.fill(lastX,1e30);Arrays.fill(lastY,1e30);}
 static double cross(double[] a,double[] b,double[] c){return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]);}
 void build(CompactSearch m,long now){
  ArrayList<double[]> p=new ArrayList<>();
  for(int i=0;i<count;i++)if(m.distance-travel[i]<=MAX_AGE_MM&&now-stamp[i]<=10000)p.add(new double[]{hx[i],hy[i]});
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
 double rayLength(CompactSearch m,int action){
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
  if(!enabled||blocked)return;
  boolean edge=false;
  for(int i=0;i<5;i++){
   float v=o.sensor(i);boolean g=v>=900&&v<=1100,w=(v>=300&&v<=500)||(v>=1850&&v<=2150);edge|=g;
   if(w){double a=m.pose.a+Math.toRadians(ParkingCore.SENSOR_ANGLES[i]),x=m.pose.x+45*Math.cos(a),y=m.pose.y+45*Math.sin(a);
    if(Math.hypot(x-lastX[i],y-lastY[i])>=10){int k=cursor++%CAP;hx[k]=x;hy[k]=y;travel[k]=m.distance;stamp[k]=o.millis;count=Math.min(CAP,count+1);lastX[i]=x;lastY[i]=y;}
   }
  }
  if(edge&&!active){active=true;m.actionUntil=-1;}
  if(!active)return;
  if(edge)clearSince=-1;else if(clearSince<0)clearSince=o.millis;
  if(m.needsAction(o.millis)||clearSince>=0&&o.millis-clearSince>=300){
   build(m,o.millis);
   // Front sensors being inside alone does not certify the body origin.
   if(clearSince>=0&&o.millis-clearSince>=300&&clearance(m.pose.x,m.pose.y)>=PAD){active=false;m.headingOffset=0;return;}
   if(m.needsAction(o.millis))prepare(m);
  }
 }
 boolean allowed(CompactSearch m,int a){return a>=0&&a<32&&(!enabled||!active||!blocked&&valid[a]);}
 void mask(CompactSearch m,float[] f){if(enabled&&active)for(int a=0;a<32;a++)if(!allowed(m,a))f[CompactSearch.MAP+a*CompactSearch.FEATURES+26]=1;}
}
