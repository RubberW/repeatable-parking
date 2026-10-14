public final class DirectionChecks {
 static void check(boolean v,String message){if(!v)throw new AssertionError(message);}
 public static void main(String[] args){
  System.setProperty("parking.learnBoundary","true");BoundarySearchServer.settings();
  PanelWorker.Plant p=new PanelWorker.Plant(BoundarySearchServer.config(1,480,0,0,72,0,0,0));
  CompactSearch m=new CompactSearch();m.observe(p.observe());
  check(DirectionObjective.boundary(m),"Policy sees edge");
  check(DirectionObjective.progress(p,m,0)<-.99,"Outward requested heading");
  check(DirectionObjective.progress(p,m,16)>.99,"Inward requested heading");
  check(Math.abs(DirectionObjective.progress(p,m,8))<.001,"Tangent allowed");
  check(DirectionObjective.cost(p,m,16)==0,"Inward goal must not be punished for its turning arc");
  check(DirectionObjective.cost(p,m,8)==0,"Do not force a fixed inward angle");
  check(DirectionObjective.cost(p,m,0)>.049,"Outward direction penalty missing");
  float[] before=RewardInputs.observation(m);DirectionObjective.cost(p,m,0);
  check(java.util.Arrays.equals(before,RewardInputs.observation(m)),"Truth leaked into observation");
  m.edgeChoice=false;check(DirectionObjective.cost(p,m,0)==0,"No blanket straight-heading penalty");
  m.edgeChoice=true;m.select(16,m.lastMs);
  for(int i=0;i<80;i++){p.step(m.command());m.observe(p.observe());}
  check(p.risk.totalCost>0,"Fixture must exercise correct inward goal with transient wheel crossing");
  check(DirectionObjective.SUPPORT_WEIGHT*p.risk.totalCost<p.risk.totalCost,"Light support cost absent");
  System.out.println("PASS direction sign, tangent freedom, inward heading independent of arc, no truth input, positive light excursion cost.");
 }
}
