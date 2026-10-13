public final class BoundaryChecks {
 public static void main(String[] args){
  System.setProperty("parking.learnBoundary","true");BoundarySearchServer.settings();
  PanelWorker.Plant p=new PanelWorker.Plant(BoundarySearchServer.config(1,480,0,0,72,0,0,0));CompactSearch m=new CompactSearch();m.observe(p.observe());
  if(m.edges!=1||m.recovery!=0||!m.needsAction(m.lastMs)||m.edgeSafety!=null)throw new AssertionError("Edge did not immediately reach policy");
  float[] f=RewardInputs.observation(m);for(int a=0;a<32;a++)if(f[CompactSearch.MAP+a*CompactSearch.FEATURES+26]!=0)throw new AssertionError("Direction still masked");
  m.select(8,m.lastMs);ParkingCore.Command turn=m.command();if(turn.left==turn.right||turn.left+turn.right<=0)throw new AssertionError("Policy tangent action was not executed");
  m.select(0,m.lastMs);
  // Deliberately select outward motion: no guard override, crossing failure or memory-extent termination.
  for(int i=0;i<3500;i++){if(m.needsAction(m.lastMs))m.select(0,m.lastMs);p.step(m.command());m.observe(p.observe());if(m.invalid||m.recovery!=0||p.supportLost())throw new AssertionError("Crossing stopped learned mode");}
  if(p.minWheel>=-100||Math.max(Math.abs(m.pose.x),Math.abs(m.pose.y))<4800)throw new AssertionError("Crossing/extent fixture did not exercise exterior");
  if(m.edges!=1)throw new AssertionError("Continuous grey repeatedly counted as contacts");
  System.out.println("PASS: immediate policy decision, 32 available directions, tangent executed, exterior/memory extent nonterminal, no repeated grey event spam.");
 }
}
