import java.util.*;

/** Local edge handling only. No target/arena/truth access and no learned weights.
 * Consistent straight-edge half-planes are inferred from recent sensor samples.
 * Unknown geometry retains the baseline 70 mm fallback; this is empirical, not certified.
 * This is a simulation safety check under explicit error/dynamics bounds, not a
 * guarantee against late cliff detection, sensor misclassification, slip or tipping.
 */
public final class BoundaryGuard {
 // 3 mm spatial allowance + 200 mm/s * 50 ms sensor latency allowance.
 static final double ERROR=13, WHEEL_PAD=4, CLEARANCE=2, MAX_RETREAT=100;
 static final int NORMALS=72, CAP=160;
 final double[] px=new double[CAP],py=new double[CAP];final boolean[] grey=new boolean[CAP];int count=0;
 final double[] oldX=new double[30],oldY=new double[30];int history=0;
 final double[] nx=new double[NORMALS],ny=new double[NORMALS],bound=new double[NORMALS];int planes=0;
 final boolean[] allowed=new boolean[32];
 public boolean active=false,halted=false,checking=false;boolean holding=false,hadWhite=false;
 public int events=0,direct=0,overrides=0,stops=0,accepted=0;
 public double lastRetreat=0,totalRetreat=0,maxRetreat=0,normalSpread=360;
 public long firstTrigger=-1;long startMs,lastCheck=-1000;double startDistance,entryYaw,anchorX,anchorY;
 int clearTicks=0;boolean initialDecision=true,fallback=false;
 public int fallbacks=0;
 public long computeNanos=0,computeCalls=0,maxComputeNanos=0;
 void timing(long t){long dt=System.nanoTime()-t;computeNanos+=dt;computeCalls++;maxComputeNanos=Math.max(maxComputeNanos,dt);}

 void add(double x,double y,boolean g){if(count<CAP){px[count]=x;py[count]=y;grey[count++]=g;}}
 public void observe(CompactSearch m,ParkingCore.Observation o){
  if(halted)return;
  boolean edge=false;for(int i=0;i<5;i++)edge|=o.sensor(i)>=900&&o.sensor(i)<=1100;
  if(active&&!holding&&edge&&hadWhite&&o.millis-startMs>300){active=false;history=0;}
  if(edge&&!active){
   active=true;holding=true;initialDecision=true;fallback=false;hadWhite=false;clearTicks=0;startMs=o.millis;startDistance=m.signedDistance;entryYaw=m.pose.a;
   anchorX=m.pose.x;anchorY=m.pose.y;lastCheck=-1000;count=0;planes=0;events++;if(firstTrigger<0)firstTrigger=o.millis;
   for(int i=0;i<Math.min(history,30);i++)add(oldX[i],oldY[i],false);
   double gx=0,gy=0,wx=0,wy=0;int ng=0,nw=0;
   for(int i=0;i<5;i++){double a=m.pose.a+Math.toRadians(ParkingCore.SENSOR_ANGLES[i]);double x=m.pose.x+45*Math.cos(a),y=m.pose.y+45*Math.sin(a);float v=o.sensor(i);if(v>=900&&v<=1100){gx+=x;gy+=y;ng++;}else{wx+=x;wy+=y;nw++;}}
   m.outwardX=gx/ng-(nw>0?wx/nw:m.pose.x);m.outwardY=gy/ng-(nw>0?wy/nw:m.pose.y);double norm=Math.hypot(m.outwardX,m.outwardY);
   if(norm<1){m.outwardX=Math.cos(entryYaw);m.outwardY=Math.sin(entryYaw);}else{m.outwardX/=norm;m.outwardY/=norm;}
   m.recovery=1;m.actionUntil=-1;m.edges++;m.edgeChoice=true;
  }
  boolean white=true;
  for(int i=0;i<5;i++){
   float v=o.sensor(i);boolean g=v>=900&&v<=1100;
   boolean w=(v>=300&&v<=500)||(v>=1850&&v<=2150);white&=w;
   double a=m.pose.a+Math.toRadians(ParkingCore.SENSOR_ANGLES[i]),x=m.pose.x+45*Math.cos(a),y=m.pose.y+45*Math.sin(a);
   if(active&&count<CAP&&(g||w))add(x,y,g);
   if(!active&&w){int k=history++%30;oldX[k]=x;oldY[k]=y;}
  }
  if(!active)return;
  clearTicks=white?clearTicks+1:0;hadWhite|=white;
  lastRetreat=Math.max(0,startDistance-m.signedDistance);
  if(holding){
   m.recovery=1;m.guard="BOUNDARY_CHECK";
   if(o.millis-startMs>7000||lastRetreat>=MAX_RETREAT){halt(m,"BOUNDARY_UNCERTAIN_STOP");return;}
   if(o.millis-lastCheck>=100){
    long tick=System.nanoTime();lastCheck=o.millis;fit();Arrays.fill(allowed,false);
    if(planes>0){
     // Reject cheap geometrically impossible candidates before trajectory rollouts.
     int safe=0;checking=true;float[] existing=m.features();
     for(int a=0;a<32;a++)if(existing[CompactSearch.MAP+a*CompactSearch.FEATURES+26]==0&&inward(m.pose.a+m.turn(a))&&trajectorySafe(m,a)){allowed[a]=true;safe++;}
     checking=false;
     if(safe>0){holding=false;m.recovery=0;m.actionUntil=-1;m.edgeChoice=true;m.guard="BOUNDARY_SAFE_CHOICES";
      maxRetreat=Math.max(maxRetreat,lastRetreat);totalRetreat+=lastRetreat;if(lastRetreat<1)direct++;initialDecision=false;}
    }
    if(holding&&lastRetreat>=70){
     // Keep the established best-model recovery when sensor geometry is inconsistent.
     if(!Boolean.parseBoolean(System.getProperty("parking.virtualBoundary","true"))){halt(m,"UNCERTAIN_PHYSICAL_EDGE_STOP");timing(tick);return;}
     fallback=true;fallbacks++;float[] existing=m.features();for(int a=0;a<32;a++)allowed[a]=existing[CompactSearch.MAP+a*CompactSearch.FEATURES+26]==0;
     holding=false;m.recovery=0;m.actionUntil=-1;m.edgeChoice=true;m.guard="BASELINE_CLEARANCE_FALLBACK";
     maxRetreat=Math.max(maxRetreat,lastRetreat);totalRetreat+=lastRetreat;
    }
    timing(tick);
   }
  }else{
   // A new edge while still escaping invalidates the old straight-edge model.
   
   if(clearTicks>=20&&Math.hypot(m.pose.x-anchorX,m.pose.y-anchorY)>100){active=false;history=0;m.edgeChoice=false;}
  }
 }
 void fit(){
  planes=0;double sx=0,sy=0;
  for(int k=0;k<NORMALS;k++){
   double a=k*2*Math.PI/NORMALS,x=Math.cos(a),y=Math.sin(a);
   // Do not infer normal sign from heading: a sensor can cross while yawing.
   // Conditional recoverability assumption: both ideal wheel centres were
   // supported at first detection. This is NOT sensed/verified here. If false,
   // reactive recovery is already too late; the evaluator reports it separately.
   double sx0=45*Math.sin(entryYaw),sy0=45*Math.cos(entryYaw);
   double lo=x*anchorX+y*anchorY+Math.abs(-x*sx0+y*sy0),hi=1e30;boolean hasG=false,hasW=false;
   for(int i=0;i<count;i++){double p=x*px[i]+y*py[i];if(grey[i]){hi=Math.min(hi,p+ERROR);hasG=true;}else{lo=Math.max(lo,p-ERROR);hasW=true;}}
   if(hasG&&hasW&&lo<=hi){nx[planes]=x;ny[planes]=y;bound[planes++]=lo;sx+=x;sy+=y;}
  }
  double mean=Math.atan2(sy,sx);normalSpread=0;for(int i=0;i<planes;i++)normalSpread=Math.max(normalSpread,2*Math.abs(ParkingCore.wrap(Math.atan2(ny[i],nx[i])-mean))*180/Math.PI);
 }
 boolean inward(double yaw){if(planes==0)return false;for(int i=0;i<planes;i++)if(nx[i]*Math.cos(yaw)+ny[i]*Math.sin(yaw)>-.05)return false;return true;}
 double minimumCentreClearance(double x,double y){double d=1e30;for(int i=0;i<planes;i++)d=Math.min(d,bound[i]-nx[i]*x-ny[i]*y);return planes>0?d:-1e30;}
 boolean wheelsInside(double x,double y,double yaw){
  double sx=45*Math.sin(yaw),sy=45*Math.cos(yaw);
  for(int i=0;i<planes;i++)if(bound[i]-nx[i]*x-ny[i]*y-Math.abs(-nx[i]*sx+ny[i]*sy)<WHEEL_PAD+CLEARANCE)return false;
  return planes>0;
 }
 boolean trajectorySafe(CompactSearch m,int action){
  // Sweep both wheel contact discs through the unchanged smooth controller.
  // Three assumed motor responses; uncalibrated, explicitly bounded.
  for(int profile=0;profile<3;profile++){
   double tau=profile==0?.12:profile==1?.20:.08,acc=profile==0?500:profile==1?300:800,brake=profile==0?800:profile==1?500:1200;
   double x=m.pose.x,y=m.pose.y,yaw=m.pose.a,fv=m.filteredV,fw=m.filteredW;
   double l=fv-45*fw,r=fv+45*fw,dir=yaw+m.turn(action),ux=Math.cos(dir),uy=Math.sin(dir),length=m.length(action);
   double x0=x,y0=y,gx=x+length*ux,gy=y+length*uy;int phase=Math.abs(m.turn(action))>Math.toRadians(135)?1:0;
   boolean reached=false;
   for(int t=0;t<800;t++){
    if(!wheelsInside(x,y,yaw))return false;
    if(Math.hypot(gx-x,gy-y)<25){reached=true;break;}
    double projection=(x-x0)*ux+(y-y0)*uy,look=150+.3*Math.max(0,fv),carrot=ParkingCore.clip(projection+look,look,length);
    double e=ParkingCore.wrap(Math.atan2(y0+uy*carrot-y,x0+ux*carrot-x)-yaw),v=0,w=0;
    if(phase==1&&Math.abs(fv)<=15)phase=2;
    if(phase==2&&Math.abs(e)<=Math.PI/3)phase=0;
    if(phase==2)w=ParkingCore.clip(2.5*e,-1.5,1.5);
    if(phase==0){w=ParkingCore.clip(2.5*e,-1.5,1.5);v=Math.min(200*Math.pow(Math.cos(e/2),2),250-45*Math.abs(w));if(Math.abs(w)>1e-6)v=Math.min(v,180/Math.abs(w));}
    ParkingCore.Command c=CompactSearch.velocity(v,w);
    l=wheel(l,c.left*1.25,tau,acc,brake);r=wheel(r,c.right*1.25,tau,acc,brake);
    ParkingCore.Command actual=new ParkingCore.Command(l/1.25,r/1.25);
    double vl=actual.left*1.25,vr=actual.right*1.25,ds=(vl+vr)*.005,dw=(vr-vl)/90*.01;
    x+=ds*Math.cos(yaw);y+=ds*Math.sin(yaw);yaw=ParkingCore.wrap(yaw+dw);double blend=1-Math.exp(-.01/.05);fv+=blend*((vl+vr)/2-fv);fw+=blend*((vr-vl)/90-fw);
   }
   if(!reached||!inward(yaw))return false;
  }
  return true;
 }
 static double wheel(double now,double desired,double tau,double acc,double brake){boolean reverse=now*desired<0;double lim=(reverse||Math.abs(desired)<Math.abs(now))?brake:acc;
  double next=now+ParkingCore.clip((desired-now)*(1-Math.exp(-.01/tau)),-lim*.01,lim*.01);
  if(reverse&&now*next<0)next=0;if(Math.abs(next)<.5&&desired==0)next=0;return next;}
 public void mask(float[] features){if(!active||holding||halted)return;for(int a=0;a<32;a++)if(!allowed[a])features[CompactSearch.MAP+a*CompactSearch.FEATURES+26]=1;}
 public boolean accept(CompactSearch m,int action){
  if(!active)return true;
  long tick=System.nanoTime();boolean safe=!holding&&!halted&&allowed[action]&&(fallback||trajectorySafe(m,action));timing(tick);
  if(!safe){overrides++;halt(m,"UNSAFE_TURN_STOP");return false;}accepted++;return true;
 }
 void halt(CompactSearch m,String why){halted=true;holding=true;m.recovery=1;m.guard=why;stops++;}
 public ParkingCore.Command command(CompactSearch m){
  if(!active)return null;
  if(halted)return CompactSearch.velocity(0,0);
  if(holding){m.guard="ADAPTIVE_RETREAT";
   // Retreat without yaw along the incoming path, with a finite distance/time cap.
   if(Boolean.getBoolean("parking.reverseArc")&&planes>0&&normalSpread<70){
    double best=0,utility=0;
    for(double w:new double[]{-.8,.8}){
     double benefit=reverseArcBenefit(m,w);
     if(benefit>utility){utility=benefit;best=w;}
    }
    if(best!=0){m.guard="REVERSE_ARC";return CompactSearch.velocity(-80,best);}
   }
   return CompactSearch.velocity(-80,0);}
  return null;
 }
 double reverseArcBenefit(CompactSearch m,double omega){
  double worst=1e30;
  for(int profile=0;profile<3;profile++){
   double tau=profile==0?.12:profile==1?.20:.08,acc=profile==0?500:profile==1?300:800,brake=profile==0?800:profile==1?500:1200;
   double x=m.pose.x,y=m.pose.y,a=m.pose.a,l=m.filteredV-45*m.filteredW,r=m.filteredV+45*m.filteredW;
   double initial=minimumCentreClearance(x,y),alignment=0;
   for(int j=0;j<planes;j++)alignment+=nx[j]*Math.cos(a)+ny[j]*Math.sin(a);
   ParkingCore.Command desired=CompactSearch.velocity(-80,omega);
   for(int step=0;step<35;step++){
    if(!wheelsInside(x,y,a))return -1;
    l=wheel(l,desired.left*1.25,tau,acc,brake);r=wheel(r,desired.right*1.25,tau,acc,brake);
    double v=(l+r)/2;x+=v*Math.cos(a)*.01;y+=v*Math.sin(a)*.01;a=ParkingCore.wrap(a+(r-l)/90*.01);
   }
   if(!wheelsInside(x,y,a)||minimumCentreClearance(x,y)<initial+3)return -1;
   double after=0;for(int j=0;j<planes;j++)after+=nx[j]*Math.cos(a)+ny[j]*Math.sin(a);
   worst=Math.min(worst,(alignment-after)/planes);
  }
  return worst>.015?worst:-1;
 }
}
