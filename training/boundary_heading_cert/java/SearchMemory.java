import java.util.*;

/** Encoder-frame evidence only. No arena dimensions, target location, or truth.
 * Assumption: the operating region is convex with straight edges. A confirmed
 * observed edge defines an inward half-plane, not three invented other sides.
 */
public final class SearchMemory {
 final String mode=System.getProperty("parking.memory","off");
 public final boolean enabled=!mode.equals("off")&&!mode.equals("arc"),corridors=mode.contains("corridor");
 static final int POINTS=80,LINES=4,SEGMENTS=512;
 final double[] px=new double[POINTS],py=new double[POINTS],ix=new double[POINTS],iy=new double[POINTS];
 final int[] event=new int[POINTS];int points=0,cursor=0;
 final double[] nx=new double[LINES],ny=new double[LINES],b=new double[LINES];
 final int[] support=new int[LINES];public int lines=0,encounters=0,preemptions=0,filtered=0;
 final double[] whiteX=new double[72],whiteY=new double[72],whiteMax=new double[72],directionsX=new double[72],directionsY=new double[72];int whites=0;double lastWhiteX=1e20,lastWhiteY=1e20;
 public SearchMemory(){for(int j=0;j<72;j++){directionsX[j]=Math.cos(j*Math.PI/36);directionsY[j]=Math.sin(j*Math.PI/36);whiteMax[j]=-1e30;}}
 final double[] lastX=new double[5],lastY=new double[5];final boolean[] wasWhite=new boolean[5];
 boolean edgeActive=false,dirty=false;long clearSince=-1,lastFit=-1000,lastPreempt=-1000;
 final float[] ax=new float[SEGMENTS],ay=new float[SEGMENTS],bx=new float[SEGMENTS],by=new float[SEGMENTS],sd=new float[SEGMENTS];
 int segments=0,segmentCursor=0;double anchorX,anchorY,anchorA;boolean anchored=false;
 public double similarDistance=0;double previousDistance=0;
 public long decisionNanos=0,decisionCalls=0;

 void observe(CompactSearch m,ParkingCore.Observation o){
  if(!enabled)return;
  boolean edge=false;for(int k=0;k<5;k++)edge|=o.sensor(k)>=900&&o.sensor(k)<=1100;
  if(edge&&!edgeActive){encounters++;edgeActive=true;clearSince=-1;}
  if(!edge){if(clearSince<0)clearSince=o.millis;if(o.millis-clearSince>=300)edgeActive=false;}else clearSince=-1;
  for(int k=0;k<5;k++){
   float v=o.sensor(k);boolean white=(v>=300&&v<=500)||(v>=1850&&v<=2150),grey=v>=900&&v<=1100;
   double a=m.pose.a+Math.toRadians(ParkingCore.SENSOR_ANGLES[k]),x=m.pose.x+45*Math.cos(a),y=m.pose.y+45*Math.sin(a);
   // A white-to-grey transition brackets the edge; grey alone is only exterior evidence.
   if(grey&&wasWhite[k]&&Math.hypot(x-lastX[k],y-lastY[k])<=10)
    point((x+lastX[k])/2,(y+lastY[k])/2,m.pose.x,m.pose.y,encounters);
   wasWhite[k]=white;lastX[k]=x;lastY[k]=y;
   if(white){
    // Contradictory interior observations retract an inferred edge immediately.
    for(int j=lines-1;j>=0;j--)if(nx[j]*x+ny[j]*y>b[j]+25){lines--;nx[j]=nx[lines];ny[j]=ny[lines];b[j]=b[lines];support[j]=support[lines];}
    if(k==2&&Math.hypot(x-lastWhiteX,y-lastWhiteY)>80){
     // Keep extreme interior evidence permanently: a ring buffer can forget the
     // start region and then incorrectly accept a diagonal cutting it away.
     for(int z=0;z<72;z++){double projection=x*directionsX[z]+y*directionsY[z];if(projection>whiteMax[z]){whiteMax[z]=projection;whiteX[z]=x;whiteY[z]=y;}}
     whites=72;lastWhiteX=x;lastWhiteY=y;
    }
   }
  }
  if(dirty&&!mode.contains("audit")&&o.millis-lastFit>=200){fit();dirty=false;lastFit=o.millis;}
  double ds=Math.max(0,m.distance-previousDistance);previousDistance=m.distance;
  if(corridors){
   if(!anchored){anchorX=m.pose.x;anchorY=m.pose.y;anchorA=m.pose.a;anchored=true;}
   double ux=Math.cos(m.pose.a),uy=Math.sin(m.pose.a);
   similarDistance+=ds*similar(m.pose.x,m.pose.y,ux,uy,m.distance);
   if(Math.hypot(m.pose.x-anchorX,m.pose.y-anchorY)>=70||Math.abs(ParkingCore.wrap(m.pose.a-anchorA))>.2){
    if(Math.hypot(m.pose.x-anchorX,m.pose.y-anchorY)>10)addSegment(anchorX,anchorY,m.pose.x,m.pose.y,m.distance);
    anchorX=m.pose.x;anchorY=m.pose.y;anchorA=m.pose.a;
   }
  }
  // Interrupt a persistent subgoal before an already learned edge is touched again.
  if(!mode.contains("audit")&&!m.contact&&m.recovery==0&&(m.edgeSafety==null||!m.edgeSafety.active)&&o.millis-lastPreempt>=700&&m.filteredV>30){
   for(int j=0;j<lines;j++)if(b[j]-nx[j]*m.pose.x-ny[j]*m.pose.y<180&&nx[j]*Math.cos(m.pose.a)+ny[j]*Math.sin(m.pose.a)>.25){
    m.actionUntil=-1;m.outwardX=nx[j];m.outwardY=ny[j];m.edgeChoice=true;lastPreempt=o.millis;preemptions++;break;
   }
  }
 }
 void point(double x,double y,double insideX,double insideY,int ev){
  for(int k=0;k<points;k++)if(event[k]==ev&&Math.hypot(px[k]-x,py[k]-y)<12)return;
  int k=cursor++%POINTS;points=Math.min(POINTS,points+1);px[k]=x;py[k]=y;ix[k]=insideX;iy[k]=insideY;event[k]=ev;dirty=true;
 }
 void fit(){
  // Exhaustive bounded pair consensus (RANSAC-style inlier test, deterministic).
  // Require 3 distinct encounters AND spatial spread: 3 adjacent sensor ticks do not count.
  ArrayList<double[]> candidates=new ArrayList<>();
  for(int a=0;a<points;a++)for(int c=a+1;c<points;c++){
   double dx=px[c]-px[a],dy=py[c]-py[a],len=Math.hypot(dx,dy);if(len<180)continue;
   double x=-dy/len,y=dx/len,offset=x*px[a]+y*py[a];
   if(x*(ix[a]-px[a])+y*(iy[a]-py[a])>0){x=-x;y=-y;offset=-offset;}
   boolean contradiction=false;for(int k=0;k<whites;k++)if(x*whiteX[k]+y*whiteY[k]>offset+15){contradiction=true;break;}
   if(contradiction)continue;
   int n=0,evs=0,bad=0;double sum=0;int[] ids=new int[POINTS];
   for(int k=0;k<points;k++)if(Math.abs(x*px[k]+y*py[k]-offset)<12){
    if(x*(ix[k]-px[k])+y*(iy[k]-py[k])>8){bad++;continue;}
    n++;sum+=x*px[k]+y*py[k];boolean seen=false;for(int z=0;z<evs;z++)seen|=ids[z]==event[k];if(!seen)ids[evs++]=event[k];
   }
   if(evs<3||bad>0)continue;
   offset=sum/n;candidates.add(new double[]{x,y,offset,evs,len});
  }
  candidates.sort((a,c)->Double.compare(c[3]*10000+c[4],a[3]*10000+a[4]));
  for(double[] c:candidates){
   double x=c[0],y=c[1],offset=c[2];int evs=(int)c[3];
   // Explicit rectangle prior constrains orientation, never unobserved side offsets.
   if(lines>0){double dot=Math.abs(nx[0]*x+ny[0]*y);if(dot<Math.cos(Math.toRadians(6))&&dot>Math.sin(Math.toRadians(6)))continue;}
   int found=-1;
   for(int j=0;j<lines;j++)if(nx[j]*x+ny[j]*y>.97){found=j;break;}
   if(found<0){if(lines==LINES)continue;found=lines++;}
   if(evs>=support[found]){nx[found]=x;ny[found]=y;b[found]=offset;support[found]=evs;}
  }
 }
 boolean safeCandidate(CompactSearch m,int action){
  double angle=m.pose.a+m.turn(action),gx=m.pose.x+m.length(action)*Math.cos(angle),gy=m.pose.y+m.length(action)*Math.sin(angle);
  for(int j=0;j<lines;j++)if(b[j]-nx[j]*gx-ny[j]*gy<80)return false;
  // Check a short curved approach as well as its destination; position alone misses bulging turns.
  double x=m.pose.x,y=m.pose.y,a=m.pose.a,v=m.filteredV,w=m.filteredW;
  for(int step=0;step<40;step++){
   double e=ParkingCore.wrap(Math.atan2(gy-y,gx-x)-a),nw=ParkingCore.clip(2.5*e,-1.5,1.5);
   double nv=Math.min(200*Math.pow(Math.cos(e/2),2),Math.abs(nw)>1e-6?180/Math.abs(nw):200);
   if(Math.abs(e)>Math.PI*.75)nv=0;
   v+=(nv-v)*.3;w+=(nw-w)*.3;x+=v*Math.cos(a)*.05;y+=v*Math.sin(a)*.05;a=ParkingCore.wrap(a+w*.05);
   for(int j=0;j<lines;j++)if(b[j]-nx[j]*x-ny[j]*y<60)return false;
  }
  return true;
 }
 void addSegment(double x0,double y0,double x1,double y1,double distance){int k=segmentCursor++%SEGMENTS;segments=Math.min(SEGMENTS,segments+1);ax[k]=(float)x0;ay[k]=(float)y0;bx[k]=(float)x1;by[k]=(float)y1;sd[k]=(float)distance;}
 double similar(double x,double y,double ux,double uy,double distance){
  double best=0;
  for(int k=0;k<segments;k++){
   if(distance-sd[k]<150)continue;double dx=bx[k]-ax[k],dy=by[k]-ay[k],l2=dx*dx+dy*dy;if(l2<1)continue;
   // Absolute dot: a reverse traversal of the same corridor is redundant too.
   double alignment=Math.abs(dx*ux+dy*uy)/Math.sqrt(l2);if(alignment<.966)continue;
   double t=((x-ax[k])*dx+(y-ay[k])*dy)/l2;if(t<0||t>1)continue;
   double d=Math.hypot(x-ax[k]-t*dx,y-ay[k]-t*dy);
   best=Math.max(best,Math.max(0,1-d/70));
  }
  return best;
 }
 double corridorCost(CompactSearch m,int action){
  if(!corridors)return 0;double a=m.pose.a+m.turn(action),ux=Math.cos(a),uy=Math.sin(a),sum=0;int n=0,streak=0,longest=0;
  for(double d=100;d<=m.length(action);d+=50){double value=similar(m.pose.x+d*ux,m.pose.y+d*uy,ux,uy,m.distance);sum+=value;n++;streak=value>.4?streak+1:0;longest=Math.max(longest,streak);}
  // Penalize sustained nearby parallel travel, not the short shared junction
  // that is unavoidable when departing an existing path.
  return n>0&&longest>=4?sum/n:0;
 }
 int choose(CompactSearch m,CompactSearch.Network net,Random random){
  long start=System.nanoTime();float[] f=m.features();double[] score=net.scores(f);boolean[] safe=new boolean[32];int n=0;
  for(int a=0;a<32;a++)if(f[CompactSearch.MAP+a*CompactSearch.FEATURES+26]==0&&safeCandidate(m,a)){safe[a]=true;n++;}
  double max=-1e30;
  for(int a=0;a<32;a++){
   if(f[CompactSearch.MAP+a*CompactSearch.FEATURES+26]>0||(n>0&&!safe[a])){score[a]=-1e30;if(!safe[a])filtered++;continue;}
   score[a]-=3*corridorCost(m,a);max=Math.max(max,score[a]);
  }
  double total=0;for(int a=0;a<32;a++)total+=Math.exp(score[a]-max);
  double u=random.nextDouble()*total;int chosen=31;for(int a=0;a<32;a++){u-=Math.exp(score[a]-max);if(u<=0){chosen=a;break;}}
  decisionNanos+=System.nanoTime()-start;decisionCalls++;return chosen;
 }
}
