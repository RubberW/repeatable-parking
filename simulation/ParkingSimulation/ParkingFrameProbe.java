/**
 * 文件职责：首次触到停车黑框后，采集足够的框边观测供几何定位使用
 * 输入：编码器相对位姿和黑白反射率；输出：短时局部探测的速度、转速和框角证据。
 * 调用关系：仅由 ParkingCore 的停车定位阶段调用；无独立循迹任务或旧模式。
 */
import java.util.*;

/** Acquire the FIRST observed line before following its tangent.
 * Uses only encoder-frame coordinates and classified reflectance readings. */
public final class ParkingFrameProbe {
  final ArrayList<double[]> points=new ArrayList<>(), white=new ArrayList<>();
  double lastSide=1,initialHeading,travel=0,lastX,lastY,lastSampleX=1e9,lastSampleY=1e9,lastSampleA=1e9;
  long lastBlackMs,start,followingSince=0;
  int corners=0,phase=0;
  double cornerTravel=0;
  double seedX,seedY,lineX,lineY,tx,ty,firstAlong,advance;
  boolean seeded=false,aligned=false;
  ParkingFrameProbe(ParkingCore.Pose p,long now){initialHeading=p.a;lastX=p.x;lastY=p.y;lastBlackMs=start=now;}
  double[] drive(ParkingCore.Observation o,ParkingCore.Pose p){
    double[] vw=probeFrame(o,p);
    vw[0]*=1.8;vw[1]*=1.25;
    return vw;
  }
  double[] probeFrame(ParkingCore.Observation o,ParkingCore.Pose p){
    double moved=Math.hypot(p.x-lastX,p.y-lastY);travel+=moved;if(phase==2)cornerTravel+=moved;lastX=p.x;lastY=p.y;
    double sum=0;int n=0;
    boolean sample=Math.hypot(p.x-lastSampleX,p.y-lastSampleY)>=1 || Math.abs(ParkingCore.wrap(p.a-lastSampleA))>=Math.toRadians(1);
    for(int i=0;i<5;i++){
      double a=Math.toRadians(ParkingCore.SENSOR_ANGLES[i]),sx=45*Math.cos(a),sy=45*Math.sin(a);
      double x=p.x+Math.cos(p.a)*sx-Math.sin(p.a)*sy,y=p.y+Math.sin(p.a)*sx+Math.cos(p.a)*sy;
      if(o.sensor(i)<1400){if(sample && phase==0)white.add(new double[]{x,y});continue;}
      if(!seeded){seedX=x;seedY=y;seeded=true;}
      sum+=sy;n++;if(sample)points.add(new double[]{x,y});
    }
    if(sample){lastSampleX=p.x;lastSampleY=p.y;lastSampleA=p.a;}
    if(n>0){lastBlackMs=o.millis;if(Math.abs(sum/n)>1)lastSide=Math.signum(sum/n);}
    if(phase==0 && o.millis-start>4500){phase=3;followingSince=start-8000;}
    if(phase==0){
      if(o.millis-start<250)return new double[]{12,ParkingCore.clip(2*ParkingCore.wrap(initialHeading-p.a),-1,1)};
      if(o.millis-start>=950 && fitFirstLine()){
        phase=1;followingSince=o.millis;double heading=Math.atan2(ty,tx);
        if(Math.abs(ParkingCore.wrap(heading-initialHeading))>Math.PI/2){tx=-tx;ty=-ty;}
        firstAlong=(p.x-lineX)*tx+(p.y-lineY)*ty;
      }else return new double[]{0,1.2};
    }
    if(phase>=1 && o.millis-followingSince>3500){
      phase=3;aligned=true;
      if(n>0){double error=sum/n;if(o.sensor(2)>=1400)error=0;return new double[]{24,ParkingCore.clip(.055*error,-1.4,1.4)};}
      return new double[]{o.millis-lastBlackMs<500?10:0,lastSide*.9};
    }
    double along=(p.x-lineX)*tx+(p.y-lineY)*ty;
    double cross=-(p.x-lineX)*ty+(p.y-lineY)*tx;
    advance=Math.max(advance,along-firstAlong);
    double headingError=ParkingCore.wrap(Math.atan2(ty,tx)-p.a);
    if(Math.abs(cross)<7 && Math.abs(headingError)<Math.toRadians(20))aligned=true;
    // Keep the first line as reference through acquisition; crossing ink alone
    // cannot make the robot drive straight through a perpendicular first edge.
    if(phase==1 && advance>20 && findCorner(along)){
      phase=2;corners=1;aligned=false;cornerTravel=0;
      along=(p.x-lineX)*tx+(p.y-lineY)*ty;
    }
    double targetX=lineX+(along+25)*tx,targetY=lineY+(along+25)*ty;
    double error=ParkingCore.wrap(Math.atan2(targetY-p.y,targetX-p.x)-p.a);
    if(aligned && n==0 && o.millis-lastBlackMs>500){
      corners=1;return new double[]{0,lastSide*.9};
    }
    if(aligned && n>0 && Math.abs(headingError)>Math.toRadians(55))corners=1;
    return new double[]{Math.abs(error)>1.1?0:24*Math.max(.3,Math.cos(error)),ParkingCore.clip(2.3*error,-1.4,1.4)};
  }
  boolean fitFirstLine(){
    ArrayList<double[]> local=new ArrayList<>();
    for(double[] p:points)if(Math.hypot(p[0]-seedX,p[1]-seedY)<50)local.add(p);
    if(local.size()<8)return false;
    double best=-1,bx=0,by=0,btx=0,bty=0;
    int step=Math.max(1,local.size()/50);
    for(int i=0;i<local.size();i+=step)for(int j=i+step;j<local.size();j+=step){
      double[] a=local.get(i),b=local.get(j);double dx=b[0]-a[0],dy=b[1]-a[1],d=Math.hypot(dx,dy);if(d<25)continue;dx/=d;dy/=d;
      if(Math.abs(-(seedX-a[0])*dy+(seedY-a[1])*dx)>4)continue;
      double count=0;for(double[] q:local)if(Math.abs(-(q[0]-a[0])*dy+(q[1]-a[1])*dx)<3.5)count++;
      for(double[] q:white)if(Math.hypot(q[0]-seedX,q[1]-seedY)<50 && Math.abs(-(q[0]-a[0])*dy+(q[1]-a[1])*dx)<2.5)count-=3;
      if(count>best){best=count;bx=a[0];by=a[1];btx=dx;bty=dy;}
    }
    if(best<12)return false;
    double x=0,y=0,n=0;for(double[] p:local)if(Math.abs(-(p[0]-bx)*bty+(p[1]-by)*btx)<3.5){x+=p[0];y+=p[1];n++;}x/=n;y/=n;
    double xx=0,xy=0,yy=0;for(double[] p:local)if(Math.abs(-(p[0]-bx)*bty+(p[1]-by)*btx)<3.5){xx+=(p[0]-x)*(p[0]-x);xy+=(p[0]-x)*(p[1]-y);yy+=(p[1]-y)*(p[1]-y);}
    double angle=.5*Math.atan2(2*xy,xx-yy);lineX=x;lineY=y;tx=Math.cos(angle);ty=Math.sin(angle);return true;
  }
  boolean findCorner(double currentAlong){
    ArrayList<double[]> off=new ArrayList<>();
    for(double[] p:points){double a=(p[0]-lineX)*tx+(p[1]-lineY)*ty,b=-(p[0]-lineX)*ty+(p[1]-lineY)*tx;
      if(Math.abs(b)>10 && Math.abs(b)<70 && a>currentAlong-15 && a<currentAlong+65)off.add(new double[]{a,b});}
    double best=0,chosenA=0,chosenB=0;
    for(double[] seed:off){double n=0,a=0,b=0,lo=1e9,hi=-1e9;
      for(double[] q:off)if(Math.abs(q[0]-seed[0])<4){n++;a+=q[0];b+=q[1];lo=Math.min(lo,q[1]);hi=Math.max(hi,q[1]);}
      if(n>=8 && hi-lo>12 && n>best){best=n;chosenA=a/n;chosenB=b/n;}
    }
    if(best==0)return false;
    lineX+=chosenA*tx;lineY+=chosenA*ty;double oldTx=tx;tx=-Math.signum(chosenB)*ty;ty=Math.signum(chosenB)*oldTx;return true;
  }

}
