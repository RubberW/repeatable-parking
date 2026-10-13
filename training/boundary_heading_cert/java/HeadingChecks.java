import java.nio.file.*;import java.util.*;
public final class HeadingChecks {
 static void check(boolean v,String s){if(!v)throw new AssertionError(s);}
 public static void main(String[] args)throws Exception{
  System.setProperty("parking.learnBoundary","true");BoundarySearchServer.settings();
  CompactSearch.Network net=new CompactSearch.Network(Paths.get(args[0]));int accepted=0,abstained=0;
  for(double yaw:new double[]{-1.3,-.8,-.4,0,.4,.8,1.3,Math.PI/4}){
   PanelWorker.Plant p=new PanelWorker.Plant(BoundarySearchServer.config(1,300,yaw==Math.PI/4?300:0,yaw,72,0,0,0));CompactSearch m=new CompactSearch();m.observe(p.observe());
   for(int t=0;t<2000&&!m.headingGate.active;t++){p.step(CompactSearch.velocity(200,0));m.observe(p.observe());}
   check(m.headingGate.active,"Missing real reflectance contact");
   float[] f=RewardInputs.observation(m);int allowed=0;
   for(int a=0;a<32;a++)if(f[CompactSearch.MAP+a*CompactSearch.FEATURES+26]==0){check(!DirectionObjective.wrong(p,m,a),"Accepted true outward heading");allowed++;}
   if(allowed==0){abstained++;check(net.choose(f,new Random(1),true)==-1,"Empty mask opened a greedy action");check(net.choose(f,new Random(1),false)==-1,"Empty mask opened a random action");}
   else{accepted++;for(int i=0;i<1000;i++){int a=net.choose(f,new Random(i),false);check(f[CompactSearch.MAP+a*CompactSearch.FEATURES+26]==0,"Random policy bypassed mask");}}
   float[] before=RewardInputs.observation(m);DirectionObjective.cost(p,m,0);check(Arrays.equals(before,RewardInputs.observation(m)),"Simulator truth changed observation");
  }
  check(accepted>=6,"Gate simply stops at most contacts");
  float[] empty=new float[CompactSearch.OBS];for(int a=0;a<32;a++)empty[CompactSearch.MAP+a*CompactSearch.FEATURES+26]=1;
  check(net.choose(empty,new Random(),true)==-1&&net.choose(empty,new Random(),false)==-1,"Empty mask fallback");
  System.out.println("PASS real contact headings, corner fixture, no outward admitted, 1000 samples per state, fail-closed empty mask, truth excluded; accepted="+accepted+", abstained="+abstained);
 }
}
