import java.io.*;import java.util.*;import java.util.concurrent.*;import processing.data.*;

/** Isolated main-policy experiment. Exterior is supported; only first ink ends search. */
public final class BoundarySearchServer {
 static JSONObject config(double size,double x,double y,double yaw,long enc,double tx,double ty,double ta){
  JSONObject c=new JSONObject();c.setString("shape","rectangle");c.setDouble("width_mm",size*1000);c.setDouble("height_mm",size*1000);c.setJSONArray("polygon",new JSONArray());
  c.setDouble("target_x",tx);c.setDouble("target_y",ty);c.setDouble("target_yaw",ta);c.setDouble("robot_x",x);c.setDouble("robot_y",y);c.setDouble("robot_yaw",yaw);c.setLong("encoder_seed",enc);c.setDouble("margin_mm",0);c.setString("stop_mode","continue");return c;
 }
 static PanelWorker.Plant scene(String[] c){return new PanelWorker.Plant(config(Double.parseDouble(c[0]),Double.parseDouble(c[3]),Double.parseDouble(c[4]),Double.parseDouble(c[5]),Long.parseLong(c[6]),Double.parseDouble(c[7]),Double.parseDouble(c[8]),Double.parseDouble(c[9])));}
 static final class Env {
  final Random rng;final double edgeCost;final int index;int episode=0;
  PanelWorker.Plant plant;CompactSearch m;boolean terminal=false;int outcome=0;double reward=0,dt=0,episodeReturn=0;
  Env(long seed,int id,double cost){rng=new Random(seed+id*100003L);index=id;edgeCost=cost;reset();}
  void reset(){
   int stratum=(episode++ + index)%8,size=1+stratum%4;boolean near=stratum>=4;double half=size*500;
   double yaw=rng.nextDouble()*2*Math.PI-Math.PI,cs=Math.abs(Math.cos(yaw)),sn=Math.abs(Math.sin(yaw));
   double ex=105*cs+148.5*sn,ey=105*sn+148.5*cs,mx=Math.max(230,ex+70),my=Math.max(230,ey+70);
   double tx=(rng.nextDouble()*2-1)*(half-mx),ty=(rng.nextDouble()*2-1)*(half-my);
   if(near){int edge=rng.nextInt(4);double pad=10+40*rng.nextDouble();if(edge<2)tx=(edge==0?-1:1)*(half-ex-pad);else ty=(edge==2?-1:1)*(half-ey-pad);}
   double x,y;do{x=(rng.nextDouble()*2-1)*(half-70);y=(rng.nextDouble()*2-1)*(half-70);}while(Math.hypot(x-tx,y-ty)<200);
   plant=new PanelWorker.Plant(config(size,x,y,rng.nextDouble()*2*Math.PI-Math.PI,rng.nextLong(),tx,ty,yaw));
   m=new CompactSearch();m.observe(plant.observe());terminal=false;outcome=0;reward=0;dt=0;episodeReturn=0;
  }
  void advance(int action){
   int seen=m.unique,regions=m.regions,edges=m.edges;double similar=m.memory.similarDistance,costBefore=plant.risk.totalCost,directionCost=DirectionObjective.cost(plant,m,action);long start=m.lastMs;
   if(!m.needsAction(start))throw new IllegalStateException("Decision requested during recovery");m.select(action,start);
   do{if(m.contact||m.invalid||(m.edgeSafety!=null&&m.edgeSafety.halted))break;plant.step(m.command());m.observe(plant.observe());}while(!m.needsAction(m.lastMs));
   dt=(m.lastMs-start)*.001;boolean fault=m.invalid||(m.edgeSafety!=null&&m.edgeSafety.halted);
   reward=.05*(m.unique-seen)*.0025+.003*(m.regions-regions)-.005*dt-.05*(m.memory.similarDistance-similar)/1000-edgeCost*(m.edges-edges)+(m.contact&&!fault?1:0)-(fault?5:0)-DirectionObjective.SUPPORT_WEIGHT*(plant.risk.totalCost-costBefore)-directionCost;
   outcome=fault?2:m.contact?1:0;terminal=outcome!=0;episodeReturn+=reward;
  }
 }
 static void write(DataOutputStream out,Env e,boolean reset)throws IOException{
  float[] meta={(float)e.reward,e.terminal?1:0,e.outcome,e.m.lastMs*.001f,e.m.unique,(float)e.m.memory.similarDistance,(float)e.m.distance,e.m.edges,(float)e.episodeReturn,(float)e.dt};
  if(e.terminal&&reset)e.reset();for(float f:RewardInputs.observation(e.m))out.writeFloat(f);for(float f:meta)out.writeFloat(f);
  // Training targets follow the observation but are never actor/critic inputs.
  for(int a=0;a<32;a++)out.writeFloat(DirectionObjective.boundary(e.m)&&DirectionObjective.progress(e.plant,e.m,a)<-.05?1:0);
 }
 public static void settings(){System.setProperty("parking.memory","lines_corridor_arc");System.setProperty("parking.reverseArc","true");System.setProperty("parking.legacyBoundary","false");System.setProperty("parking.virtualBoundary","true");System.setProperty("smooth.steering","smooth");}
 public static void main(String[] args)throws Exception{
  settings();int n=Integer.parseInt(args[0]);long seed=Long.parseLong(args[1]);double cost=Double.parseDouble(args[2]);Env[] envs=new Env[n];for(int i=0;i<n;i++)envs[i]=new Env(seed,i,cost);
  DataInputStream in=new DataInputStream(new BufferedInputStream(System.in));DataOutputStream out=new DataOutputStream(new BufferedOutputStream(System.out));out.writeInt(CompactSearch.OBS);out.writeInt(32);out.writeInt(n);for(Env e:envs)write(out,e,false);out.flush();
  ExecutorService pool=Executors.newFixedThreadPool(Math.min(4,n));
  try{while(true){int[] actions=new int[n];actions[0]=in.readInt();if(actions[0]==-99)break;
   // Administrative training sampler refresh. No terminal/failure transition or reward is fabricated.
   if(actions[0]==-98){for(Env e:envs){e.reset();write(out,e,false);}out.flush();continue;}
   for(int i=1;i<n;i++)actions[i]=in.readInt();List<Callable<Void>> jobs=new ArrayList<>();for(int i=0;i<n;i++){final int k=i;jobs.add(()->{envs[k].advance(actions[k]);return null;});}for(Future<Void> f:pool.invokeAll(jobs))f.get();for(Env e:envs)write(out,e,true);out.flush();}}catch(EOFException end){}finally{pool.shutdownNow();}
 }
}
