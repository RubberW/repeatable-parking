import java.util.*;
import java.nio.file.*;
import java.io.*;

/** Encoder/reflectance-only state, motion primitives, and exported policy inference.
 * No simulator, map, table, or target coordinates are accessible here. */
public final class NeuralSearch {
 public static final int ACTIONS=18,FEATURES=12,OBS=ACTIONS*FEATURES,SIDE=640;
 static final double CELL=20,OFFSET=SIDE/2.0,MAX_V=200;
 static final double[] ANGLES={0,Math.PI/4,-Math.PI/4,Math.PI/2,-Math.PI/2,Math.PI};
 static final double[] SPEEDS={80,140,200};
 final byte[] seen=new byte[SIDE*SIDE],blocked=new byte[SIDE*SIDE],centres=new byte[SIDE*SIDE];
 public final ParkingCore.Pose pose=new ParkingCore.Pose(0,0,0);
 long lastMs=-1,left,right,actionUntil=-1;
 int lastCell=-1,action=2,recovery=0;
 boolean repeatedCell=false;
 double desired=0,reverseStart=0,recoveryTurn=0;
 public int unique=0,edges=0;
 public double distance=0,repeatedDistance=0,lastVelocity=0,lastNovelty=0;
 public String guard="CLEAR";
 public boolean contact=false,invalid=false;
 int index(double x,double y){int xx=(int)Math.floor(x/CELL+OFFSET),yy=(int)Math.floor(y/CELL+OFFSET);return xx<0||xx>=SIDE||yy<0||yy>=SIDE?-1:yy*SIDE+xx;}
 boolean near(byte[] grid,double x,double y,int radius){
  int xx=(int)Math.floor(x/CELL+OFFSET),yy=(int)Math.floor(y/CELL+OFFSET);
  for(int dy=-radius;dy<=radius;dy++)for(int dx=-radius;dx<=radius;dx++){
   int a=xx+dx,b=yy+dy;if(a>=0&&a<SIDE&&b>=0&&b<SIDE&&grid[b*SIDE+a]!=0)return true;
  }return false;
 }
 public void observe(ParkingCore.Observation o){
  double ds=0;
  if(lastMs>=0){double l=(o.leftCount-left)*ParkingCore.MM_COUNT,r=(o.rightCount-right)*ParkingCore.MM_COUNT;
   ds=(l+r)/2;pose.x+=ds*Math.cos(pose.a);pose.y+=ds*Math.sin(pose.a);pose.a=ParkingCore.wrap(pose.a+(r-l)/ParkingCore.B);
   lastVelocity=Math.abs(ds)/Math.max(.001,(o.millis-lastMs)*.001);distance+=Math.abs(ds);
  }
  lastMs=o.millis;left=o.leftCount;right=o.rightCount;
  int cell=index(pose.x,pose.y);
  if(cell!=lastCell && cell>=0){repeatedCell=centres[cell]!=0;centres[cell]=1;lastCell=cell;}
  if(repeatedCell)repeatedDistance+=Math.abs(ds);
  int before=unique;boolean edge=false;
  for(int i=0;i<5;i++){
   float v=o.sensor(i);double a=pose.a+Math.toRadians(ParkingCore.SENSOR_ANGLES[i]);double x=pose.x+45*Math.cos(a),y=pose.y+45*Math.sin(a);int k=index(x,y);
   if(v>=1850 && v<=2150)contact=true;
   if(v>=900 && v<=1100){edge=true;if(k>=0)blocked[k]=1;}
   else if((v>=300&&v<=500)||(v>=1850&&v<=2150)){if(k>=0&&seen[k]==0){seen[k]=1;unique++;}}
   else invalid=true;
  }
  if(Math.abs(ds)>0)lastNovelty=.98*lastNovelty+.02*(unique>before?1:0);
  if(edge && recovery==0){recovery=1;reverseStart=distance;edges++;actionUntil=-1;}
 }
 public float[] features(){
  float[] obs=new float[OBS];
  for(int a=0;a<ACTIONS;a++){
   double angle=ANGLES[a/3],v=SPEEDS[a%3],direction=pose.a+angle,cs=Math.cos(direction),sn=Math.sin(direction);
   int fresh=0,farFresh=0,obstacle=0,farObstacle=0,n=0,nf=0;
   double reach=Math.max(50,(2-Math.abs(angle)/1.5)*v);
   for(int i=1;i<=12;i++){
    double travel=reach*i/12;
    for(double side:new double[]{-25,0,25}){
     double x=pose.x+(travel+35)*cs-side*sn,y=pose.y+(travel+35)*sn+side*cs;
     fresh+=near(seen,x,y,0)?0:1;obstacle+=near(blocked,x,y,3)?1:0;n++;
    }
    double far=400+800*i/12.0;
    double x=pose.x+far*cs,y=pose.y+far*sn;farFresh+=near(seen,x,y,1)?0:1;farObstacle+=near(blocked,x,y,4)?1:0;nf++;
   }
   int k=a*FEATURES;obs[k]=(float)fresh/n;obs[k+1]=(float)farFresh/nf;obs[k+2]=(float)obstacle/n;obs[k+3]=(float)farObstacle/nf;
   obs[k+4]=(float)(Math.abs(angle)/Math.PI);obs[k+5]=(float)(v/MAX_V);obs[k+6]=(float)Math.sin(angle);
   obs[k+7]=(float)(lastMs/480000.0);obs[k+8]=(float)Math.min(1,Math.hypot(pose.x,pose.y)/3000);
   obs[k+9]=(float)(lastVelocity/MAX_V);obs[k+10]=(float)lastNovelty;obs[k+11]=recovery==0?0:1;
  }return obs;
 }
 public void select(int a,long now){action=Math.max(0,Math.min(ACTIONS-1,a));desired=ParkingCore.wrap(pose.a+ANGLES[action/3]);actionUntil=now+2000;}
 public boolean needsAction(long now){return now>=actionUntil && recovery==0;}
 public ParkingCore.Command command(){
  if(invalid||contact)return new ParkingCore.Command(0,0);
  if(recovery==1){guard="RETRACE";if(distance-reverseStart<160)return velocity(-80,0);recovery=2;recoveryTurn=ParkingCore.wrap(pose.a+Math.PI);}
  if(recovery==2){guard="TURN_INWARD";double e=ParkingCore.wrap(recoveryTurn-pose.a);if(Math.abs(e)>.04)return velocity(0,ParkingCore.clip(2.5*e,-1.5,1.5));recovery=0;guard="CLEAR";actionUntil=-1;return velocity(0,0);}
  guard="CLEAR";if(actionUntil<0)return velocity(0,0);double e=ParkingCore.wrap(desired-pose.a);
  if(Math.abs(e)>.06)return velocity(0,ParkingCore.clip(3*e,-1.5,1.5));
  return velocity(SPEEDS[action%3],ParkingCore.clip(2*e,-.4,.4));
 }
 static ParkingCore.Command velocity(double v,double w){return new ParkingCore.Command((v-45*w)/1.25,(v+45*w)/1.25);}
 public static int greedy(float[] obs){
  int best=0;double score=-1e9;
  for(int a=0;a<ACTIONS;a++){int k=a*FEATURES;double s=obs[k]*1.2+obs[k+1]*.25-obs[k+2]*4-obs[k+3]*.5-obs[k+4]*.15+obs[k+5]*.35;
   if(s>score){score=s;best=a;}}
  return best;
 }
 public static final class Network {
  final float[][] w=new float[3][];final float[][] b=new float[3][];
  final int[] sizes={FEATURES,32,16,1};
  public Network(Path file)throws IOException{
   try(DataInputStream in=new DataInputStream(new BufferedInputStream(Files.newInputStream(file)))){
    if(in.readInt()!=0x4e4e5331)throw new IOException("Policy format mismatch");
    for(int j=0;j<3;j++){w[j]=new float[sizes[j]*sizes[j+1]];b[j]=new float[sizes[j+1]];for(int i=0;i<w[j].length;i++)w[j][i]=in.readFloat();for(int i=0;i<b[j].length;i++)b[j][i]=in.readFloat();}
   }
  }
  public double score(float[] obs,int action){
   double[] v=new double[FEATURES];for(int i=0;i<FEATURES;i++)v[i]=obs[action*FEATURES+i];
   for(int j=0;j<3;j++){double[] next=new double[sizes[j+1]];for(int o=0;o<next.length;o++){double sum=b[j][o];for(int i=0;i<v.length;i++)sum+=w[j][o*v.length+i]*v[i];next[o]=j==2?sum:Math.tanh(sum);}v=next;}return v[0];
  }
  public int sample(float[] obs,Random random){
   double[] logits=new double[ACTIONS];double max=-1e100;for(int a=0;a<ACTIONS;a++){logits[a]=score(obs,a);max=Math.max(max,logits[a]);}
   double sum=0;for(int a=0;a<ACTIONS;a++){logits[a]=Math.exp(logits[a]-max);sum+=logits[a];}
   double r=random.nextDouble()*sum;for(int a=0;a<ACTIONS;a++){r-=logits[a];if(r<=0)return a;}return ACTIONS-1;
  }
  public int choose(float[] obs){int best=0;double score=-1e100;for(int a=0;a<ACTIONS;a++){double s=score(obs,a);if(s>score){score=s;best=a;}}return best;}
 }
}
