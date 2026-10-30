/**
 * 文件职责：重放完整场景，统计广域搜索质量：找到黑框时间、检测覆盖效率、近平行重复路程、触边折返和越线。
 * 输入：场景CSV、主模型权重、输出CSV；可选分片 起始序号 步长。输出：逐场景结果CSV。
 * 调用关系：experiments/search_coverage.py 调用；只用真实轨迹做评估，不进入控制器。
 */
import java.io.*;import java.nio.file.*;import java.util.*;import java.lang.reflect.*;
public final class SearchCoverageEvaluation {
 static final double R=110,CELL=20,STEP=20;
 public static void main(String[] args)throws Exception{
  EvaluationScenes.settings();System.setProperty("parking.policy",args[1]);System.setProperty("parking.localMethod","early_fit");System.setProperty("parking.offline","true");System.setProperty("parking.heuristicScore","false");
  int start=args.length>3?Integer.parseInt(args[3]):0,step=args.length>4?Integer.parseInt(args[4]):1;
  Field field=ParkingCore.Controller.class.getDeclaredField("compact");field.setAccessible(true);
  List<String> lines=Files.readAllLines(Paths.get(args[0]));
  try(PrintWriter out=new PrintWriter(Files.newBufferedWriter(Paths.get(args[2]),StandardOpenOption.CREATE_NEW))){
   out.println("scene,size,placement,status,first_black_s,total_s,parked,false_completion,search_path_m,coverage_eff,parallel_lt70,parallel_70_220,edge_contacts,reverse_gt150,search_full_wheel_s,search_critical_s,search_min_wheel_mm,total_full_wheel_s,total_critical_s,decisions");
   for(int row=1+start;row<lines.size();row+=step){String[] c=lines.get(row).split(",");System.setProperty("parking.policySeed",c[10]);SimulationRunner.Plant p=EvaluationScenes.scene(c);
    ParkingCore.Trial t=new ParkingCore.Trial(p,new ParkingCore.MapDescription(),c[1],c[2],new ParkingCore.Calibration(1000));
    double full=0,critical=0,min=0;boolean searched=false;CompactSearch m=null;String status="censored_resource_budget";
    ArrayList<double[]> samples=new ArrayList<>();ArrayList<Integer> edgeAt=new ArrayList<>();int lastEdges=0;
    for(int k=0;k<2000000&&!t.controller.done();k++){
     t.tick();m=(CompactSearch)field.get(t.controller);if(m!=null&&m.decisions>=4096&&t.controller.firstBlackMs<0){status="censored_decision_budget";break;}
     if(t.controller.firstBlackMs<0){
      double x=p.robot.odometry.preciseX,y=p.robot.odometry.preciseY;
      if(samples.isEmpty()||Math.hypot(x-samples.get(samples.size()-1)[0],y-samples.get(samples.size()-1)[1])>=STEP)samples.add(new double[]{x,y});
      if(m!=null&&m.edges>lastEdges){lastEdges=m.edges;edgeAt.add(samples.size()-1);}
     }
     if(!searched&&t.controller.firstBlackMs>=0){searched=true;full=p.risk.fullWheelSeconds;critical=p.risk.criticalSeconds;min=p.minWheel;}
     if(t.controller.firstBlackMs>=0&&p.robot.getMillis()-t.controller.firstBlackMs>120000){status="censored_local_evaluation_budget";break;}
    }
    if(t.controller.done())status=t.success?"parked":t.falsePositive?"false_completion":t.controller.failure;
    if(!searched){full=p.risk.fullWheelSeconds;critical=p.risk.criticalSeconds;min=p.minWheel;}
    double[] q=quality(samples,edgeAt);
    double first=t.controller.firstBlackMs<0?-1:t.controller.firstBlackMs*.001,total=p.robot.getMillis()*.001;
    out.printf(Locale.US,"%s,%s,%s,%s,%.3f,%.3f,%b,%b,%.3f,%.4f,%.4f,%.4f,%d,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%d%n",c[1],c[0],c[2],status,first,total,t.success,t.falsePositive,q[0],q[1],q[2],q[3],(int)q[4],(int)q[5],full,critical,min,p.risk.fullWheelSeconds,p.risk.criticalSeconds,m==null?0:m.decisions);out.flush();
   }
  }
 }
 /** 路程(米)、覆盖效率、近平行<70与70-220毫米占比、触边次数、折返>150度次数。 */
 static double[] quality(List<double[]> s,List<Integer> edgeAt){
  int n=s.size();if(n<2)return new double[]{0,0,0,0,edgeAt.size(),0};
  HashSet<Long> covered=new HashSet<>();int rc=(int)Math.ceil(R/CELL);
  for(double[] p:s){long cx=(long)Math.floor(p[0]/CELL),cy=(long)Math.floor(p[1]/CELL);
   for(long i=cx-rc;i<=cx+rc;i++)for(long j=cy-rc;j<=cy+rc;j++)if(Math.hypot((i+.5)*CELL-p[0],(j+.5)*CELL-p[1])<=R)covered.add(i*100000+j);}
  double path=STEP*(n-1),eff=covered.size()*CELL*CELL/(path*2*R);
  double[] heading=new double[n];for(int k=0;k<n;k++){int a=Math.max(0,k-1),b=Math.min(n-1,k+1);heading[k]=Math.atan2(s.get(b)[1]-s.get(a)[1],s.get(b)[0]-s.get(a)[0]);}
  HashMap<Long,ArrayList<Integer>> index=new HashMap<>();int close=0,mid=0;
  for(int k=0;k<n;k++){double x=s.get(k)[0],y=s.get(k)[1];long bx=(long)Math.floor(x/200),by=(long)Math.floor(y/200);double best=1e9;
   for(long i=bx-1;i<=bx+1;i++)for(long j=by-1;j<=by+1;j++)for(int z:index.getOrDefault(i*100000+j,new ArrayList<>())){
    if(k-z<30)continue;double da=Math.abs(ParkingCore.wrap(heading[k]-heading[z]));da=Math.min(da,Math.PI-da);if(da>Math.toRadians(20))continue;
    best=Math.min(best,Math.abs(-(x-s.get(z)[0])*Math.sin(heading[z])+(y-s.get(z)[1])*Math.cos(heading[z])));}
   if(best<70)close++;else if(best<2*R)mid++;
   index.computeIfAbsent(bx*100000+by,u->new ArrayList<>()).add(k);}
  int reverse=0;
  for(int e=0;e<edgeAt.size();e++){int i=edgeAt.get(e),j=i-15,end=e+1<edgeAt.size()?edgeAt.get(e+1):n-1;if(j<0)continue;
   double ain=Math.atan2(s.get(i)[1]-s.get(j)[1],s.get(i)[0]-s.get(j)[0]);
   for(int k=i+1;k<=end;k++)if(Math.hypot(s.get(k)[0]-s.get(i)[0],s.get(k)[1]-s.get(i)[1])>=300){
    double aout=Math.atan2(s.get(k)[1]-s.get(i)[1],s.get(k)[0]-s.get(i)[0]);if(Math.abs(ParkingCore.wrap(aout-ain))>Math.toRadians(150))reverse++;break;}}
  return new double[]{path/1000,eff,(double)close/n,(double)mid/n,edgeAt.size(),reverse};
 }
}
