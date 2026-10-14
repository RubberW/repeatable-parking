public final class SupportRiskChecks {
 static SupportRisk sample(double l,double r,double c,double dt){SupportRisk s=new SupportRisk();s.update(l,r,c,dt);return s;}
 static void check(boolean ok,String why){if(!ok)throw new AssertionError(why);}
 public static void main(String[] args){
  check(sample(10,10,40,1).totalCost==0,"inside penalized");
  SupportRisk light=sample(4,30,30,1),half=sample(0,30,30,1),whole=sample(-5,30,30,1),centre=sample(-1,30,-.1,1),both=sample(-1,-1,10,1);
  check(light.totalCost>0&&half.totalCost>light.totalCost&&whole.totalCost>half.totalCost,"nonmonotonic wheel penalty");
  check(whole.totalCost==1&&centre.totalCost>1&&both.criticalEvents==1,"full wheel / centre / both-wheel tiers");
  SupportRisk one=sample(-6,30,-1,1),ten=new SupportRisk();for(int i=0;i<100;i++)ten.update(-6,30,-1,.01);
  check(Math.abs(one.totalCost-ten.totalCost)<1e-9&&ten.criticalEvents==1,"frame-rate dependence or repeated entry penalty");
  double old=ten.totalCost;ten.update(-20,30,-10,1);check(ten.totalCost>old&&ten.criticalEvents==1,"continued excursion stopped charging");
  for(int i=0;i<31;i++)ten.update(10,10,30,.01);ten.update(-6,30,-1,.01);check(ten.criticalEvents==2,"new excursion not charged");
  System.setProperty("parking.learnBoundary","true");BoundarySearchServer.settings();
  BoundarySearchServer.Env e=new BoundarySearchServer.Env(777,0,0);e.plant=new PanelWorker.Plant(BoundarySearchServer.config(1,520,0,0,1,0,0,0));e.m=new CompactSearch();e.m.observe(e.plant.observe());e.advance(0);
  check(!e.terminal&&!e.plant.supportLost()&&e.m.lastMs>0,"crossing terminated rollout");check(e.plant.risk.criticalEvents>0&&e.reward < 0,"light excursion and outward-heading penalties not applied");
  System.out.println("PASS zero interior cost, progressive penetration, full-wheel/centre/both-wheel severe tiers, time integration, repeated excursions, negative reward without termination.");
 }
}
