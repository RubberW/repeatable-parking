/**
 * 文件职责：保留访问路径；普通地图积累直边证据，复杂地图停用全局直边推断。
 * 输入：相对里程计和黑白边界观测；输出：访问特征、重复路径代价、已有边线证据。
 * 调用关系：CompactSearch 使用；SearchDecision 读取路径特征。
 */
import java.util.*;

/** Encoder-frame evidence only. No arena dimensions, target location, or truth.
 * Global line fitting is enabled only in ordinary rectangle mode. Complex
 * scenes keep visitation/corridors and never promote contacts into half-planes.
 */
public final class SearchMemory {
 public final boolean enabled=true,corridors=true;
 public final boolean straightEdges;
 static final int POINTS=80,LINES=4,SEGMENTS=512;
 final double[] px=new double[POINTS],py=new double[POINTS],ix=new double[POINTS],iy=new double[POINTS];
 final int[] event=new int[POINTS];int points=0,cursor=0;
 final double[] nx=new double[LINES],ny=new double[LINES],b=new double[LINES];
 final int[] support=new int[LINES];public int lines=0,encounters=0,preemptions=0,filtered=0;
 final double[] whiteX=new double[72],whiteY=new double[72],whiteMax=new double[72],directionsX=new double[72],directionsY=new double[72];int whites=0;double lastWhiteX=1e20,lastWhiteY=1e20;
 public SearchMemory(){this(true);}
 public SearchMemory(boolean straightEdges){this.straightEdges=straightEdges;for(int j=0;j<72;j++){directionsX[j]=Math.cos(j*Math.PI/36);directionsY[j]=Math.sin(j*Math.PI/36);whiteMax[j]=-1e30;}}
 final double[] lastX=new double[5],lastY=new double[5];final boolean[] wasWhite=new boolean[5];
 boolean edgeActive=false,dirty=false;long clearSince=-1,lastFit=-1000,lastPreempt=-1000;
 final float[] ax=new float[SEGMENTS],ay=new float[SEGMENTS],bx=new float[SEGMENTS],by=new float[SEGMENTS],sd=new float[SEGMENTS];
 int segments=0,segmentCursor=0;double anchorX,anchorY,anchorA;boolean anchored=false;
 public double similarDistance=0;double previousDistance=0;
 // 检测覆盖：黑框外框152×182毫米、传感器横向约±32毫米，框心离轨迹约110毫米内必被经过。
 // 按这一尺度判断“已搜过”，而不是按传感器自身的50毫米宽度。
 static final double DETECT_R=Double.parseDouble(System.getProperty("parking.detectRadius","110"));
 final byte[] detected=new byte[CompactSearch.SIDE*CompactSearch.SIDE/8];double markX=1e30,markY=1e30;
 public long decisionNanos=0,decisionCalls=0;

 void observe(CompactSearch m,ParkingCore.Observation o){
  if(!enabled)return;
  boolean edge=false;for(int k=0;k<5;k++)edge|=o.sensor(k)>=900&&o.sensor(k)<=1100;
  if(edge&&!edgeActive){encounters++;edgeActive=true;clearSince=-1;}
  if(!edge){if(clearSince<0)clearSince=o.millis;if(o.millis-clearSince>=300)edgeActive=false;}else clearSince=-1;
  if(straightEdges){for(int k=0;k<5;k++){
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
  if(dirty&&o.millis-lastFit>=200){fit();dirty=false;lastFit=o.millis;}}
  if(Math.hypot(m.pose.x-markX,m.pose.y-markY)>=25){markDetected(m);markX=m.pose.x;markY=m.pose.y;}
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
  if(straightEdges&&!m.contact&&o.millis-lastPreempt>=700&&m.filteredV>30){
   for(int j=0;j<lines;j++)if(b[j]-nx[j]*m.pose.x-ny[j]*m.pose.y<180&&nx[j]*Math.cos(m.pose.a)+ny[j]*Math.sin(m.pose.a)>.25){
    m.actionUntil=-1;m.outwardX=nx[j];m.outwardY=ny[j];m.edgeChoice=true;lastPreempt=o.millis;preemptions++;break;
   }
  }
 }
 void point(double x,double y,double insideX,double insideY,int ev){
  if(!straightEdges)return;
  for(int k=0;k<points;k++)if(event[k]==ev&&Math.hypot(px[k]-x,py[k]-y)<12)return;
  int k=cursor++%POINTS;points=Math.min(POINTS,points+1);px[k]=x;py[k]=y;ix[k]=insideX;iy[k]=insideY;event[k]=ev;dirty=true;
 }
 void fit(){
  if(!straightEdges)return;
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
 void markDetected(CompactSearch m){
  for(double dx=-DETECT_R;dx<=DETECT_R;dx+=CompactSearch.CELL)for(double dy=-DETECT_R;dy<=DETECT_R;dy+=CompactSearch.CELL)
   if(dx*dx+dy*dy<=DETECT_R*DETECT_R)CompactSearch.set(detected,m.fine(m.pose.x+dx,m.pose.y+dy));
 }
 boolean outsideKnownEdge(double x,double y){
  for(int j=0;j<lines;j++)if(nx[j]*x+ny[j]*y>b[j])return true;
  return false;
 }
 /** 候选方向检测带中没有新收益的比例：已搜过、已知边线外、或沿中线越过灰色观测之后的点。
  * 起点附近总在当前覆盖圈内，从圈外开始取样。 */
 double redundancy(CompactSearch m,int action){
  double a=m.pose.a+m.turn(action),ux=Math.cos(a),uy=Math.sin(a);int hit=0,n=0;boolean beyondGrey=false;
  for(double d=DETECT_R+40;d<=m.length(action);d+=50){
   if(CompactSearch.bit(m.blocked,m.fine(m.pose.x+d*ux,m.pose.y+d*uy)))beyondGrey=true;
   for(double side=-DETECT_R;side<=DETECT_R+1e-9;side+=DETECT_R/2){
    double x=m.pose.x+d*ux-side*uy,y=m.pose.y+d*uy+side*ux;int k=m.fine(x,y);n++;
    if(beyondGrey||k<0||outsideKnownEdge(x,y)||CompactSearch.bit(detected,k))hit++;
   }
  }
  return n>0?(double)hit/n:0;
 }
 double corridorCost(CompactSearch m,int action){
  if(!corridors)return 0;double a=m.pose.a+m.turn(action),ux=Math.cos(a),uy=Math.sin(a),sum=0;int n=0,streak=0,longest=0;
  for(double d=100;d<=m.length(action);d+=50){double value=similar(m.pose.x+d*ux,m.pose.y+d*uy,ux,uy,m.distance);sum+=value;n++;streak=value>.4?streak+1:0;longest=Math.max(longest,streak);}
  // Penalize sustained nearby parallel travel, not the short shared junction
  // that is unavoidable when departing an existing path.
  return n>0&&longest>=4?sum/n:0;
 }

}
