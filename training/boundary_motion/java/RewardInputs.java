/** Same sensor/encoder-derived inputs and safety mask in training and deployment. */
public final class RewardInputs {
 public static float[] observation(CompactSearch m){
  float[] f=m.features();boolean[] safe=new boolean[32];int count=0;
  for(int a=0;a<32;a++){
   int k=CompactSearch.MAP+a*CompactSearch.FEATURES;
   f[k+25]=(float)m.memory.corridorCost(m,a);
   if(f[k+26]==0&&m.memory.safeCandidate(m,a)){safe[a]=true;count++;}
  }
  // Preserve the existing fallback if uncertain map geometry masks all actions.
  if(!m.learnBoundary&&count>0)for(int a=0;a<32;a++)if(!safe[a])f[CompactSearch.MAP+a*CompactSearch.FEATURES+26]=1;
  m.headingGate.mask(m,f);return f;
 }
}
