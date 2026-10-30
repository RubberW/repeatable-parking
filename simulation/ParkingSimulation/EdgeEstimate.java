/**
 * 文件职责：根据近期传感器和编码器观测估计直边方向与夹角
 * 输入：黑白和边界观测的相对坐标；输出：边线法向、夹角及是否可确认。
 * 调用关系：BoundaryMotion 用来判断是否启用特殊动作。
 */
/** Local straight-edge evidence for actuator selection. Sensors and encoder frame only. */
public final class EdgeEstimate {
 final double[] x=new double[256],y=new double[256];final long[] time=new long[256];final boolean[] grey=new boolean[256];
 int count=0,cursor=0;public double nx=1,ny=0,alpha=90,spread=360,offset=0;public boolean reliable=false,split=false;
 public void observe(CompactSearch m,ParkingCore.Observation o){
  int runs=0;boolean previous=false;
  for(int i=0;i<5;i++){
   float v=o.sensor(i);boolean g=v>=900&&v<=1100,w=v>=300&&v<=500||v>=1850&&v<=2150;
   if(g&&!previous)runs++;previous=g;
   if(g||w){int k=cursor++%256;count=Math.min(256,count+1);double a=m.pose.a+Math.toRadians(ParkingCore.SENSOR_ANGLES[i]);x[k]=m.pose.x+45*Math.cos(a);y[k]=m.pose.y+45*Math.sin(a);time[k]=o.millis;grey[k]=g;}
  }
  split=runs>1;
 }
 public void fit(CompactSearch m){
  reliable=false;double gx=0,gy=0;int ng=0;
  for(int i=0;i<count;i++)if(grey[i]&&m.lastMs-time[i]<=100){gx+=x[i];gy+=y[i];ng++;}
  if(ng==0||split)return;gx/=ng;gy/=ng;
  double sx=0,sy=0,db=0;int n=0;double[] angles=new double[72];
  for(int k=0;k<72;k++){
   double a=k*Math.PI/36,ux=Math.cos(a),uy=Math.sin(a),lo=-1e30,hi=1e30;int nw=0,gg=0;
   for(int i=0;i<count;i++)if(m.lastMs-time[i]<=1200&&Math.hypot(x[i]-gx,y[i]-gy)<=150){
    double d=ux*x[i]+uy*y[i];if(grey[i]){hi=Math.min(hi,d+2);gg++;}else{lo=Math.max(lo,d-2);nw++;}
   }
   if(nw<5||gg==0||lo>hi||ux*m.outwardX+uy*m.outwardY<.25)continue;
   sx+=ux;sy+=uy;db+=(lo+hi)/2;angles[n++]=a;
  }
  if(n==0)return;double mean=Math.atan2(sy,sx);spread=0;
  for(int i=0;i<n;i++)spread=Math.max(spread,Math.toDegrees(Math.abs(ParkingCore.wrap(angles[i]-mean)))*2);
  nx=Math.cos(mean);ny=Math.sin(mean);offset=db/n;
  alpha=Math.toDegrees(Math.acos(Math.min(1,Math.abs(nx*Math.cos(m.pose.a)+ny*Math.sin(m.pose.a)))));
  reliable=spread<=65;
 }
}
